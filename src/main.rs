use bme280::i2c::BME280;
use chrono::Duration as ChronoDuration;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::digital::v2::PinState;
use ringbuffer::AllocRingBuffer;
use ringbuffer::RingBuffer;
use rppal::gpio::Gpio;
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

use embedded_canvas::Canvas;
use embedded_graphics::draw_target::DrawTarget;
use embedded_graphics::geometry::Size;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::pixelcolor::RgbColor;

use embedded_graphics::{
    prelude::*,
};

use ltr_559::InterruptMode;
use ltr_559::InterruptPinPolarity;
use ltr_559::LedCurrent;
use ltr_559::LedDutyCycle;
use ltr_559::LedPulse;
use ltr_559::PsMeasRate;
use ltr_559::{AlsGain, AlsIntTime, AlsMeasRate, Ltr559, SlaveAddr};
use st7735_lcd::Orientation;
use std::error::Error;
use systemstat::{Platform, System};
use tracing_subscriber::{layer::SubscriberExt, util::SubscriberInitExt};
mod audio;
mod pages;

use pages::PageTypes;
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

    let sys = System::new();
    let gpio = Gpio::new()?;
    let _backlight = start_display(&gpio)?;

    let mut bme280 = init_pth_sensors(I2c::new()?)?;
    let mut light_sensor = init_light_sensor(I2c::new()?)?;
    let display = Arc::new(Mutex::new(init_display(&gpio)?));

    let mut rb = AllocRingBuffer::new(audio::FRAMES as usize);
    rb.fill(0.0_f32);
    let buffer_producer = Arc::new(Mutex::new(rb));
    let buffer_consumer = buffer_producer.clone();

    let timer = Timer::new();
    let error = Arc::new(AtomicBool::new(false));
    let handler_error = error.clone();
    let handler_display = display.clone();

    let audio = audio::Audio::new(
        "adau7002",
        move |sample: Vec<f32>| -> Result<(), Box<dyn Error>> {
            //tracing::info!("got sample {:?}", &sample[..10]);
            if let Ok(mut buf) = buffer_producer.lock() {
                //	    sample.iter().for_each(|s| buf.push(*s));
                buf.extend(sample);
            }
            Ok(())
        },
    )?;

    let _ = ctrlc::set_handler(move || {
        if let Ok(mut disp) = handler_display.lock() {
            clear_display(&mut disp);
        }
        std::process::exit(0);
    });

    let pages = pages::Pages::new(DISPLAY_SIZE);
    
    display_page(&display, pages.draw_page(&PageTypes::Boot)?)?;
    
    thread::sleep(Duration::from_millis(500));
    let mut page_selector = PageTypes::Error("Start");
    
    
    let guard = timer.schedule_repeating(ChronoDuration::milliseconds(500),move ||{

	let mut timer_task = ||->Result<(), Box<dyn Error>>{
            let status = light_sensor.get_status().map_err(|_e| "Can't get light sensor status")?;
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

	    let cpu_temp = match sys.cpu_temp() {
		Ok(cpu_temp) => Some(cpu_temp),
		Err(x) => {
                    tracing::warn!("CPU temp: {}", x);
                    None
		}
            };

            let measurements: bme280::Measurements<rppal::i2c::Error> = bme280.measure().map_err(|_| "Can't get measurements from bme280")?;
            let delta = cpu_temp.map(|temp| temp - measurements.temperature);

	    let pressure = measurements.pressure;
	    let temperature = measurements.temperature;
	    let humidity = measurements.humidity;

            if let Some(proximity) = proximity {
		if proximity.0 > PROXIMITY_TRESHOLD {
		    page_selector = match page_selector {
			PageTypes::All(_) => PageTypes::Temperature(temperature),
			PageTypes::Temperature(_) => PageTypes::Pressure(pressure),
			PageTypes::Pressure(_) => PageTypes::Humidity(humidity),
			PageTypes::Humidity(_) => PageTypes::Brightness(lux),
			PageTypes::Brightness(_) =>
			{
			    if audio.start_audio().is_err(){
				PageTypes::Error("Error")
			    }else{
				let data = if let Ok(buf) = buffer_consumer.lock() {
				    buf.to_vec()
				}else{
				    vec![]
				};
				PageTypes::Noise(data)
			    }		    
			},
			PageTypes::Noise(_) => PageTypes::All((pressure, temperature, humidity, lux)),
			PageTypes::Error(_) => PageTypes::All((pressure, temperature, humidity, lux)),
			PageTypes::Boot => PageTypes::All((pressure, temperature, humidity, lux)),
			
		    };
		    tracing::info!("Selected page {page_selector}");
		    audio.stop_audio();
		}else{
		    page_selector = match page_selector {
			PageTypes::All(_) => PageTypes::All((pressure, temperature, humidity, lux)),
			PageTypes::Temperature(_) => PageTypes::Temperature(temperature),
			PageTypes::Pressure(_) => PageTypes::Pressure(pressure),
			PageTypes::Humidity(_) => PageTypes::Humidity(humidity),
			PageTypes::Brightness(_) => PageTypes::Brightness(lux),
			PageTypes::Noise(_) => {
			    if audio.start_audio().is_err(){
				PageTypes::Error("Error")
			    }else{
				let data = if let Ok(buf) = buffer_consumer.lock() {
				    buf.to_vec()
				}else{
				    vec![]
				};
				PageTypes::Noise(data)
			    }
			}
			PageTypes::Error(_) => PageTypes::All((pressure, temperature, humidity, lux)),
			PageTypes::Boot => PageTypes::Boot,
		    };
		};
	    };

	    
	    tracing::trace!(
		"Lux {:?} Proximity {:?} Relative Humidity = {}% Sensor Temperature = {} CPU temp = {:?} delta {:?} deg C, Pressure = {} hPa",
		lux,
		proximity,
		measurements.humidity,
		measurements.temperature,
		cpu_temp,
		delta,
		measurements.pressure / 100.0
            );
	    	    

	    let page_canvas = pages.draw_page(&page_selector)?;
	    	    
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



