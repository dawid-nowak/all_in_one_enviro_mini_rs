use bme280::i2c::BME280;
use chrono::Duration as ChronoDuration;
use embedded_graphics::image::Image;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::digital::v2::PinState;
use plotters::backend::PixelFormat;
use plotters::backend::RGBPixel;
use plotters::prelude::BitMapBackend;
use plotters::prelude::ChartBuilder;
use plotters::prelude::IntoDrawingArea;
use plotters::series::LineSeries;
use plotters::style::RED;
use ringbuffer::AllocRingBuffer;
use ringbuffer::RingBuffer;
use rppal::gpio::Gpio;
use rppal::gpio::Level;
use rppal::gpio::Trigger;
use rppal::hal::Delay;
use rppal::i2c::I2c;
use rppal::spi::Bus;
use rppal::spi::Mode as SPIMode;
use rppal::spi::SlaveSelect;
use rppal::spi::Spi;
use std::sync::atomic::AtomicBool;
use std::sync::atomic::Ordering;
use std::sync::Arc;
use std::sync::Mutex;
use std::thread;
use systemstat::Duration;
use timer::Timer;

use eg_seven_segment::SevenSegmentStyleBuilder;
use embedded_canvas::Canvas;
use embedded_graphics::draw_target::DrawTarget;
use embedded_graphics::geometry::Size;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::pixelcolor::RgbColor;
use embedded_graphics::primitives::{PrimitiveStyleBuilder, Rectangle};
use embedded_graphics::text::Text;
use embedded_graphics::{
    mono_font::{iso_8859_16::FONT_10X20, iso_8859_16::FONT_6X10, MonoTextStyle},
    prelude::*,
};
use embedded_layout::layout::linear::spacing::DistributeFill;
use embedded_layout::{layout::linear::LinearLayout, prelude::*};
use ltr_559::InterruptMode;
use ltr_559::InterruptPinPolarity;
use ltr_559::LedCurrent;
use ltr_559::LedDutyCycle;
use ltr_559::LedPulse;
use ltr_559::PsMeasRate;
use ltr_559::{AlsGain, AlsIntTime, AlsMeasRate, Ltr559, SlaveAddr};
use rand::Rng;
use st7735_lcd::Orientation;
use std::error::Error;
use systemstat::{Platform, System};
use tinybmp::Bmp;
use tracing_subscriber::{layer::SubscriberExt, util::SubscriberInitExt};

mod audio;

const DC_PIN: u8 = 9;
const BACKLIGHT_PIN: u8 = 12;

const SPI_SPEED: u32 = 10_000_000;
const DISPLAY_HORIZONTAL_OFFSET: i32 = 0;
const DISPLAY_VERTICAL_OFFSET: i32 = 25;
const DISPLAY_OFFSET: Point = Point::new(DISPLAY_HORIZONTAL_OFFSET, DISPLAY_VERTICAL_OFFSET);
const DISPLAY_SIZE_WIDTH: u32 = 160;
const DISPLAY_SIZE_HEIGTH: u32 = 80;
const DISPLAY_SIZE: Size = Size::new(DISPLAY_SIZE_WIDTH, DISPLAY_SIZE_HEIGTH);
const PROXIMITY_TRESHOLD: u16 = 1500;

struct NotConnected;

impl OutputPin for NotConnected {
    type Error = rppal::gpio::Error;
    fn set_low(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
    fn set_high(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn set_state(&mut self, state: PinState) -> Result<(), Self::Error> {
        match state {
            PinState::Low => self.set_low(),
            PinState::High => self.set_high(),
        }
    }
}

fn main() -> Result<(), Box<dyn Error>> {
    tracing_subscriber::registry()
        .with(tracing_subscriber::EnvFilter::new(
            std::env::var("RUST_LOG").unwrap_or_else(|_| "debug".to_string()),
        ))
        .with(tracing_subscriber::fmt::layer())
        .init();

    tracing::info!("Hello PI");
    //    check_audio()?;

    let sys = System::new();
    let gpio = Gpio::new()?;
    let _backlight = start_display(&gpio)?;

    let mut bme280 = init_pth_sensors(I2c::new()?)?;
    let mut light_sensor = init_light_sensor(I2c::new()?)?;
    let display = Arc::new(Mutex::new(init_display(&gpio)?));

    let buffer_out = Arc::new(Mutex::new(AllocRingBuffer::new(320)));
    let buffer_in = buffer_out.clone();
    let _handler = Arc::new(thread::spawn(move || loop {
        thread::sleep(Duration::from_millis(500));
        tracing::debug!("Producing data");
        let between = rand::distributions::Uniform::from(-40..40);
        let sample: Vec<i8> = rand::thread_rng().sample_iter(&between).take(40).collect();
        if let Ok(mut buf) = buffer_out.lock() {
            buf.extend(sample);
        }
    }));

    let timer = Timer::new();
    let error = Arc::new(AtomicBool::new(false));
    let handler_error = error.clone();
    let handler_display = display.clone();
    let audio = audio::Audio::new("adau7002")?;

    let _ = ctrlc::set_handler(move || {
        if let Ok(mut disp) = handler_display.lock() {
            clear_display(&mut disp);
        }
        std::process::exit(0);
    });

    let canvas = prepare_boot_page(DISPLAY_SIZE)?;

    if let Ok(mut disp) = display.lock() {
        canvas
            .place_at(DISPLAY_OFFSET)
            .draw(&mut *disp)
            .map_err(|_| "Can't draw")?;
    }

    thread::sleep(Duration::from_millis(500));

    let mut screen_indicator = 0;

    let guard = timer.schedule_repeating(ChronoDuration::milliseconds(500),move ||{

	let mut timer_task = ||->Result<(), Box<dyn Error>>{
            let status = light_sensor.get_status().map_err(|e| "Can't get light sensor status")?;

            tracing::debug!("Light sensor status {status:?}");
            let lux = if status.als_data_valid || status.als_interrupt_status {
		let (_lux_raw_0, _lux_raw_1) = light_sensor.get_als_raw_data().map_err(|_| "Can't get light raw data")?;
		let lux = light_sensor.get_lux().map_err(|_| "Can't get light lux")?;
		Some(lux)
            } else {
            tracing::warn!("No lux");
            None
            };

            let proximity = if status.ps_data_status || status.ps_interrupt_status {
		let proximity = light_sensor.get_ps_data().map_err(|_| "Can't get proximity")?;
		Some(proximity)
            } else {
		tracing::warn!("No prox");
		None
            };

            if let Some(proximity) = proximity {
		if proximity.0 > PROXIMITY_TRESHOLD {
                    screen_indicator += 1;
		    audio.stop_audio();
		}		
            }

            let cpu_temp = match sys.cpu_temp() {
		Ok(cpu_temp) => Some(cpu_temp),
		Err(x) => {
                    tracing::warn!("CPU temp: {}", x);
                    None
		}
            };

            let measurements = bme280.measure().map_err(|_| "Can't get measurements from bme280")?;
            let delta = cpu_temp.map(|temp| temp - measurements.temperature);

            tracing::debug!(
		"Lux {:?} Proximity {:?} Relative Humidity = {}% Sensor Temperature = {} CPU temp = {:?} delta {:?} deg C, Pressure = {} hPa",
		lux,
		proximity,
		measurements.humidity,
		measurements.temperature,
		cpu_temp,
		delta,
		measurements.pressure / 100.0
            );

            let page_canvas = match screen_indicator % 7 {
		0 => prepare_all_sensors_page(
                    DISPLAY_SIZE,
                    measurements.pressure / 100.0,
                    measurements.temperature,
                    measurements.humidity,
                    lux.unwrap_or(f32::NAN),
		)?,
		1 => prepare_sensor_page(
                    DISPLAY_SIZE,
                    "Temperature",
                    measurements.temperature,
                    "\u{00B0}C",
		)?,
		2 => prepare_sensor_page(
                    DISPLAY_SIZE,
                    "Pressure",
                    measurements.pressure / 100.0,
                    "hPa",
		)?,
		3 => prepare_sensor_page(DISPLAY_SIZE, "Humidity", measurements.humidity, "%")?,
		4 => prepare_sensor_page(DISPLAY_SIZE, "Brightness", lux.unwrap_or(f32::NAN), "lux")?,
		5 => {
                    if let Ok(buf) = buffer_in.lock() {
			let data = buf.to_vec();
			prepare_chart_page(DISPLAY_SIZE, &data)?
                    } else {
			prepare_error_page(DISPLAY_SIZE, "Error")?
                    }
		}
		6 => {
		    if audio.start_audio().is_ok(){
			prepare_noize_page(DISPLAY_SIZE)?
		    }else{
			prepare_error_page(DISPLAY_SIZE, "No Noise")?
		    }		    
		},

		_ => prepare_error_page(DISPLAY_SIZE, "Error")?,
            };
	    
	    display_page(&display, page_canvas)?;	    	    
	    Ok(())
	};
	
	if let Err(e) = timer_task(){
	    tracing::error!("Timer task {e:?}");
	    handler_error.store(true, Ordering::Relaxed);
	};
    }
    );

    while !error.load(Ordering::Relaxed) {
        thread::sleep(Duration::from_millis(100));
    }
    drop(guard);
    Ok(())
}

fn display_page(
    display: &Arc<Mutex<st7735_lcd::ST7735<Spi, rppal::gpio::OutputPin, NotConnected>>>,
    canvas: Canvas<Rgb565>,
) -> Result<(), Box<dyn Error>> {
    if let Ok(mut disp) = display.lock() {
        let canvas_at = canvas.place_at(DISPLAY_OFFSET);
        let bb = canvas_at.bounding_box();
        let pixels_iter = bb.points().filter_map(|point| canvas_at.get_pixel(point));
        disp.fill_contiguous(&bb, pixels_iter)
            .map_err(|_| "Can't draw".into())
    } else {
        Err("Can't lock".into())
    }
}

fn init_pth_sensors(i2c: I2c) -> Result<BME280<I2c, Delay>, Box<dyn Error>> {
    let mut bme280 = BME280::new_primary(i2c, Delay);
    bme280.init().map_err(|e| format!("{e:?}"))?;
    Ok(bme280)
}

fn init_light_sensor(i2c: I2c) -> Result<Ltr559<I2c, ltr_559::ic::Ltr559>, Box<dyn Error>> {
    let mut light_sensor = Ltr559::new_device(i2c, SlaveAddr::default());
    light_sensor
        .set_als_contr(AlsGain::Gain4x, true, false)
        .map_err(|e| format!("{e:?}"))?;
    thread::sleep(Duration::from_millis(100));
    light_sensor.reset_internal_driver_state();
    thread::sleep(Duration::from_millis(100));

    let light_sensor_manufacturer_id = light_sensor
        .get_manufacturer_id()
        .map_err(|e| format!("{e:?}"))?;
    let light_sensor_part_id = light_sensor.get_part_id().map_err(|e| format!("{e:?}"))?;

    tracing::info!(
        "LTR559 Manufactured by {light_sensor_manufacturer_id:x} : {light_sensor_part_id:x}"
    );

    light_sensor
        .set_interrupt(InterruptPinPolarity::Low, InterruptMode::Both)
        .map_err(|e| format!("{e:?}"))?;

    light_sensor
        .set_ps_led(LedPulse::Pulse30, LedDutyCycle::_100, LedCurrent::_50mA)
        .map_err(|e| format!("{e:?}"))?;
    light_sensor
        .set_ps_n_pulses(1)
        .map_err(|e| format!("{e:?}"))?;

    light_sensor
        .set_als_contr(AlsGain::Gain4x, false, true)
        .map_err(|e| format!("{e:?}"))?;
    light_sensor
        .set_ps_contr(true, true)
        .map_err(|e| format!("{e:?}"))?;

    light_sensor
        .set_ps_meas_rate(PsMeasRate::_100ms)
        .map_err(|e| format!("{e:?}"))?;
    light_sensor
        .set_als_meas_rate(AlsIntTime::_50ms, AlsMeasRate::_50ms)
        .map_err(|e| format!("{e:?}"))?;

    light_sensor
        .set_ps_offset(0)
        .map_err(|e| format!("{e:?}"))?;
    light_sensor
        .set_ps_low_limit_raw(0)
        .map_err(|e| format!("{e:?}"))?;
    light_sensor
        .set_ps_high_limit_raw(1000)
        .map_err(|e| format!("{e:?}"))?;
    Ok(light_sensor)
}

fn start_display(gpio: &Gpio) -> Result<rppal::gpio::OutputPin, Box<dyn Error>> {
    let mut backlight = gpio.get(BACKLIGHT_PIN)?.into_output();
    backlight.set_low();
    thread::sleep(Duration::from_millis(100));
    backlight.set_high();
    thread::sleep(Duration::from_millis(100));
    Ok(backlight)
}

fn init_display(
    gpio: &Gpio,
) -> Result<st7735_lcd::ST7735<Spi, rppal::gpio::OutputPin, NotConnected>, Box<dyn Error>> {
    let mut delay = Delay;
    let spi = Spi::new(Bus::Spi0, SlaveSelect::Ss1, SPI_SPEED, SPIMode::Mode0)?;

    let dc = gpio.get(DC_PIN)?.into_output();
    let mut display = st7735_lcd::ST7735::new(spi, dc, NotConnected, false, true, 162, 132);
    display
        .init(&mut delay)
        .map_err(|_| "Can't initialize the display")?;
    display
        .set_orientation(&Orientation::LandscapeSwapped)
        .map_err(|_| "Can't set the orientation")?;

    let color = Rgb565::BLACK;
    display
        .clear(color)
        .map_err(|_| "Can't clear the display")?;
    Ok(display)
}

fn clear_display(display: &mut st7735_lcd::ST7735<Spi, rppal::gpio::OutputPin, NotConnected>) {
    let color = Rgb565::BLACK;
    if let Err(e) = display.clear(color) {
        tracing::error!("Can't clear the screen {:?}", e);
    }
}

fn register_proximity_callback(gpio: &Gpio) -> Result<(), Box<dyn Error>> {
    let mut proximity = gpio.get(4)?.into_input_pullup();
    proximity.set_async_interrupt(Trigger::FallingEdge, |c: Level| {
        tracing::warn!("Got interrupt level {c:?}");
    })?;
    Ok(())
}

fn prepare_boot_page(size: Size) -> Result<Canvas<Rgb565>, Box<dyn Error>> {
    let rust_logo = include_bytes!("./rust-logo_80x80.bmp");
    let pimoroni_logo = include_bytes!("./pimoroni-logo_80x80.bmp");
    let rust_image =
        Bmp::from_slice(rust_logo).map_err(|e| format!("Can't open Rust image {:?}", e))?;
    let pimoroni_image =
        Bmp::from_slice(pimoroni_logo).map_err(|e| format!("Can't open Pimoroni image {:?}", e))?;
    let mut canvas = Canvas::with_default_color(size, Rgb565::BLACK);

    Image::new(&rust_image, Point::new(0, 0)).draw(&mut canvas)?;
    Image::new(&pimoroni_image, Point::new(80, 0)).draw(&mut canvas)?;
    Ok(canvas)
}

fn sensor_canvas(
    size: Size,
    name: &str,
    value: f32,
    unit: &str,
) -> Result<Canvas<Rgb565>, Box<dyn Error>> {
    let style = MonoTextStyle::new(&FONT_6X10, Rgb565::WHITE);
    let mut canvas = Canvas::with_default_color(size, Rgb565::BLACK);
    let val = format!("{value:9.2}");

    let t1 = Text::new(name, Point::zero(), style);
    let t2 = Text::new(&val, Point::zero(), style);
    let t3 = Text::new(unit, Point::zero(), style);

    let ll = LinearLayout::horizontal(Chain::new(t1).append(t2).append(t3))
        .with_alignment(vertical::Center)
        .with_spacing(DistributeFill(size.width - 10))
        .arrange();

    let style = PrimitiveStyleBuilder::new()
        .stroke_color(Rgb565::WHITE)
        .stroke_width(1)
        .build();

    let bounding_box = Rectangle::new(Point::new(5, 5), Size::new(size.width - 5, size.height - 5));
    let rect = Rectangle::new(Point::new(0, 0), size).into_styled(style);

    ll.align_to(&bounding_box, horizontal::Left, vertical::Center)
        .draw(&mut canvas)?;
    rect.draw(&mut canvas)?;

    Ok(canvas)
}

fn prepare_all_sensors_page(
    size: Size,
    pressure: f32,
    temperature: f32,
    humidity: f32,
    brightness: f32,
) -> Result<Canvas<Rgb565>, Box<dyn Error>> {
    let style = PrimitiveStyleBuilder::new()
        .stroke_color(Rgb565::WHITE)
        .stroke_width(4)
        .build();

    let mut canvas = Canvas::with_default_color(size, Rgb565::BLACK);
    let pressure = sensor_canvas(Size::new(size.width, 20), "Pressure", pressure, "hPa")?;
    let brightness = sensor_canvas(Size::new(size.width, 20), "Brightness", brightness, "Lux")?;
    let temperature = sensor_canvas(
        Size::new(size.width, 20),
        "Temperature",
        temperature,
        "\u{00b0}C",
    )?;
    let humidity = sensor_canvas(Size::new(size.width, 20), "Humidity", humidity, "%")?;
    temperature
        .place_at(Point::new(0, 00))
        .draw(&mut canvas)
        .map_err(|_| "Can't draw")?;
    pressure
        .place_at(Point::new(0, 20))
        .draw(&mut canvas)
        .map_err(|_| "Can't draw")?;
    humidity
        .place_at(Point::new(0, 40))
        .draw(&mut canvas)
        .map_err(|_| "Can't draw")?;
    brightness
        .place_at(Point::new(0, 60))
        .draw(&mut canvas)
        .map_err(|_| "Can't draw")?;
    Rectangle::new(Point::new(0, 0), size)
        .into_styled(style)
        .draw(&mut canvas)
        .map_err(|_| "Can't draw")?;

    Ok(canvas)
}

fn prepare_sensor_page(
    size: Size,
    name: &str,
    value: f32,
    unit: &str,
) -> Result<Canvas<Rgb565>, Box<dyn Error>> {
    let mut canvas = Canvas::with_default_color(size, Rgb565::BLACK);

    let text_style = MonoTextStyle::new(&FONT_6X10, Rgb565::WHITE);

    let unit_style = MonoTextStyle::new(&FONT_10X20, Rgb565::GREEN);

    let seven_segment_style = SevenSegmentStyleBuilder::new()
        .digit_size(Size::new(20, 40))
        .digit_spacing(2)
        .segment_width(5)
        .segment_color(Rgb565::GREEN)
        .build();

    let value = format!("{value:2.2}");
    let t1 = Text::new(name, Point::zero(), text_style);
    let t2 = Text::new(&value, Point::zero(), seven_segment_style);
    let t3 = Text::new(unit, Point::zero(), unit_style);

    let ll = LinearLayout::horizontal(Chain::new(t2).append(t3))
        .with_alignment(vertical::Center)
        .with_spacing(DistributeFill(size.width - 10))
        .arrange();
    let lv = LinearLayout::vertical(Chain::new(t1).append(ll))
        .with_alignment(horizontal::Left)
        //        .with_spacing(DistributeFill(size.height - 10))
        .arrange();

    let bounding_box = Rectangle::new(Point::new(0, 0), Size::new(size.width, size.height));
    let rectangle_style = PrimitiveStyleBuilder::new().stroke_width(0).build();
    let rect = Rectangle::new(Point::new(0, 0), size).into_styled(rectangle_style);

    lv.align_to(&bounding_box, horizontal::Center, vertical::Center)
        .draw(&mut canvas)?;
    rect.draw(&mut canvas)?;

    Ok(canvas)
}

fn prepare_error_page(size: Size, name: &str) -> Result<Canvas<Rgb565>, Box<dyn Error>> {
    let mut canvas = Canvas::with_default_color(size, Rgb565::BLACK);

    let seven_segment_style = SevenSegmentStyleBuilder::new()
        .digit_size(Size::new(30, 40))
        .digit_spacing(5)
        .segment_width(5)
        .segment_color(Rgb565::RED)
        .build();

    let t1 = Text::new(name, Point::zero(), seven_segment_style);

    let ll = LinearLayout::vertical(Chain::new(t1))
        .with_alignment(horizontal::Center)
        .arrange();

    let bounding_box = Rectangle::new(Point::new(0, 0), Size::new(size.width, size.height));
    let rectangle_style = PrimitiveStyleBuilder::new().stroke_width(0).build();
    let rect = Rectangle::new(Point::new(0, 0), size).into_styled(rectangle_style);

    ll.align_to(&bounding_box, horizontal::Left, vertical::Center)
        .draw(&mut canvas)?;
    rect.draw(&mut canvas)?;

    Ok(canvas)
}

fn prepare_noize_page(size: Size) -> Result<Canvas<Rgb565>, Box<dyn Error>> {
    prepare_error_page(size, "Noise")
}

fn prepare_chart_page(size: Size, sample: &[i8]) -> Result<Canvas<Rgb565>, Box<dyn Error>> {
    let w = size.width;
    let h = size.height;
    let pixel_size = <RGBPixel as PixelFormat>::EFFECTIVE_PIXEL_SIZE as u32;

    let mut buffer = vec![0; (w * h * pixel_size) as usize];
    {
        let root = BitMapBackend::with_buffer(&mut buffer, (w, h)).into_drawing_area();
        //        root.fill(&BLACK).unwrap();

        let mut chart_builder = ChartBuilder::on(&root);

        chart_builder.margin(0);
        chart_builder.margin_bottom(0);
        chart_builder.margin_top(0);
        chart_builder.set_left_and_bottom_label_area_size(0);
        chart_builder.set_all_label_area_size(0);
        let c = h as i32;
        let mut chart_context = chart_builder.build_cartesian_2d(0..320_u32, -c / 2..c / 2)?;

        chart_context.configure_mesh().disable_mesh().draw()?;

        let data: Vec<(u32, i32)> = sample
            .iter()
            .enumerate()
            .map(|(i, s)| (i as u32, *s as i32))
            //            .filter(|s| s.0 % 5 == 0)
            .collect();
        let ls = LineSeries::new(data, RED);
        chart_context.draw_series(ls)?;
        root.present()?;
    }

    let mut canvas = Canvas::with_default_color(size, Rgb565::BLACK);

    //    let mut pixels = vec![];
    let mut pixel_colors = vec![];
    for i in 0..h {
        for j in 0..w {
            let pos = (i * w + j) * pixel_size;
            let colors = (
                buffer.get(pos as usize),
                buffer.get((pos + 1) as usize),
                buffer.get((pos + 2) as usize),
            );
            if let (Some(r), Some(b), Some(g)) = colors {
                let pixel_color = Rgb565::new(*r, *g, *b);
                //                pixels.push(Pixel(Point::new(j as i32, i as i32), pixel_color));
                pixel_colors.push(pixel_color);
            } else {
                println!("No colors at pos {i} {j} {pos}");
            }
        }
    }
    let area = Rectangle::new(Point::new(0, 0), size);
    canvas.fill_contiguous(&area, pixel_colors)?;
    //.draw_iter(pixels)?;
    Ok(canvas)
}
