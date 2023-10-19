use bme280::i2c::BME280;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::digital::v2::PinState;
use rppal::gpio::Gpio;
use rppal::gpio::Level;
use rppal::gpio::Trigger;
use rppal::hal::Delay;
use rppal::i2c::I2c;
use rppal::spi::Bus;
use rppal::spi::Mode as SPIMode;
use rppal::spi::SlaveSelect;
use rppal::spi::Spi;
use std::thread;
use std::time::Duration;

use embedded_canvas::Canvas;
use embedded_graphics::draw_target::DrawTarget;
use embedded_graphics::geometry::Size;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::pixelcolor::RgbColor;
use embedded_graphics::primitives::{PrimitiveStyleBuilder, Rectangle};
use embedded_graphics::text::Text;
use embedded_graphics::{
    mono_font::{
        ascii::{FONT_10X20, FONT_6X10},
        MonoTextStyle,
    },
    prelude::*,
};
use ltr_559::{AlsGain, AlsIntTime, AlsMeasRate, Ltr559, SlaveAddr};
use ltr_559::InterruptPinPolarity;
use ltr_559::InterruptMode;
use ltr_559::PsMeasRate;
use ltr_559::LedPulse;
use ltr_559::LedDutyCycle;
use ltr_559::LedCurrent;
use st7735_lcd::Orientation;
use std::error::Error;
use systemstat::{Platform, System};
use tracing_subscriber::{layer::SubscriberExt, util::SubscriberInitExt};



use portaudio as pa;


const INTERLEAVED: bool = true;
const LATENCY: pa::Time = 0.0; // Ignored by PortAudio::is_*_format_supported.
const STANDARD_SAMPLE_RATES: [f64; 13] = [
    8000.0, 9600.0, 11025.0, 12000.0, 16000.0, 22050.0, 24000.0, 32000.0, 44100.0, 48000.0,
    88200.0, 96000.0, 192000.0,
];



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

const SPI_SPEED: u32 = 10_000_000;
const DISPLAY_SIZE: Size = Size::new(160, 80);

fn check_audio()-> Result<(), Box<dyn Error>>{
    let pa = pa::PortAudio::new()?;
    tracing::info!("PortAudio version: {} {}", pa.version(), pa.version_text()?);
    let host_count = pa.host_api_count()?;
    tracing::info!("PortAudio host count {host_count}");
    let default_host = pa.default_host_api();
    let default_input = pa.default_input_device();
//    let input_info = pa.device_info(default_input);
    
    tracing::info!("PortAudio host count {host_count:?} {default_host:?} {default_input:?}");
    //    tracing::info!("PortAudio default input device info: {:#?}", &input_info);
    let devices = pa.device_count()?;
    tracing::info!("Devices {devices}");
    for device in pa.devices()? {
        let (idx, info) = device?;
        tracing::info!("--------------------------------------- {:?}", idx);
        tracing::info!("{:#?}", &info);

        let in_channels = info.max_input_channels;
        let input_params = pa::StreamParameters::<i16>::new(idx, in_channels, INTERLEAVED, LATENCY);
        let out_channels = info.max_output_channels;
        let output_params =
            pa::StreamParameters::<i16>::new(idx, out_channels, INTERLEAVED, LATENCY);

        tracing::info!(
            "Supported standard sample rates for half-duplex 16-bit {} channel input:",
            in_channels
        );
        for &sample_rate in &STANDARD_SAMPLE_RATES {
            if pa
                .is_input_format_supported(input_params, sample_rate)
                .is_ok()
            {
                tracing::info!("\t{}hz", sample_rate);
            }
        }

        tracing::info!(
            "Supported standard sample rates for half-duplex 16-bit {} channel output:",
            out_channels
        );
        for &sample_rate in &STANDARD_SAMPLE_RATES {
            if pa
                .is_output_format_supported(output_params, sample_rate)
                .is_ok()
            {
                tracing::info!("\t{}hz", sample_rate);
            }
        }

        tracing::info!("Supported standard sample rates for full-duplex 16-bit {} channel input, {} channel output:",
                 in_channels, out_channels);
        for &sample_rate in &STANDARD_SAMPLE_RATES {
            if pa
                .is_duplex_format_supported(input_params, output_params, sample_rate)
                .is_ok()
            {
                tracing::info!("\t{}hz", sample_rate);
            }
        }
    }
    
    Ok(())
}

fn main() -> Result<(), Box<dyn Error>> {
    tracing_subscriber::registry()
        .with(tracing_subscriber::EnvFilter::new(
            std::env::var("RUST_LOG").unwrap_or_else(|_| "debug".to_string()),
        ))
        .with(tracing_subscriber::fmt::layer())
        .init();

    tracing::info!("Hello PI");
    check_audio()?;

    let sys = System::new();    
    let gpio = Gpio::new()?;    
    let mut backlight = gpio.get(12)?.into_output();

    backlight.set_low();
    thread::sleep(Duration::from_millis(100));
    backlight.set_high();
    thread::sleep(Duration::from_millis(100));
    
    let mut bme280 = init_pth_sensors(I2c::new()?)?;
    let mut light_sensor = init_light_sensor(I2c::new()?)?;        
    let mut display = init_display(gpio)?;
    
    
    let small_style = MonoTextStyle::new(&FONT_6X10, Rgb565::WHITE);
    let style = PrimitiveStyleBuilder::new()
        .stroke_color(Rgb565::BLUE)
        .stroke_width(1)
        .fill_color(Rgb565::GREEN)
        .build();

    let mut canvas = Canvas::with_default_color(DISPLAY_SIZE, Rgb565::RED.into());

    for i in 0..8 {
        let w = 20;
        let h = 10;
        let x = i * w as i32;
        let y = i * h as i32;

        Rectangle::new(Point::new(x, y), Size::new(w, h))
            .into_styled(style)
            .draw(&mut canvas)
            .map_err(|_| "Can't draw")?;
    }

    for i in 0..8 {
        let w = 20;
        let h = 10;
        let x = i * w as i32;
        let y = i * h as i32;
        Text::new("Hello ROBIN", Point::new(x, y), small_style)
            .draw(&mut canvas)
            .map_err(|_| "Can't draw")?;
    }

    canvas
        .place_at(Point::new(0, 25))
        .draw(&mut display)
        .map_err(|_| "Can't draw")?;

    loop {
	thread::sleep(Duration::from_millis(1000));
	
        let status = light_sensor.get_status().unwrap();
	
        tracing::debug!("Light sensor status {status:?}");
        let lux = if status.als_data_valid || status.als_interrupt_status {
            let (_lux_raw_0, _lux_raw_1) = light_sensor.get_als_raw_data().unwrap();
            let lux = light_sensor.get_lux().unwrap();            
            Some(lux)
        } else {
            tracing::warn!("No lux");
            None
        };

	let proximity = if status.ps_data_status || status.ps_interrupt_status {
            let proximity = light_sensor.get_ps_data().unwrap();
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

        let measurements = bme280.measure().unwrap();
        let delta = if let Some(temp) = cpu_temp {
            Some(temp - measurements.temperature)
        } else {
            None
        };

        tracing::info!(
            "Lux {:?} Proximity {:?} Relative Humidity = {}% Sensor Temperature = {} CPU temp = {:?} delta {:?} deg C, Pressure = {} hPa",
	    lux,
	    proximity,
            measurements.humidity,
            measurements.temperature,
            cpu_temp,
            delta,
            measurements.pressure / 100.0
        );
    }
}

fn init_pth_sensors(i2c: I2c) -> Result<BME280<I2c,Delay>, Box<dyn Error> > {
    let mut bme280 = BME280::new_primary(i2c, Delay);
    bme280.init().map_err(|e| format!("{e:?}"))?;
    Ok(bme280)        
}

fn init_light_sensor(i2c : I2c) -> Result<Ltr559<I2c, ltr_559::ic::Ltr559>, Box<dyn Error> > {
    let mut light_sensor = Ltr559::new_device(i2c, SlaveAddr::default());
    light_sensor.set_als_contr(AlsGain::Gain4x, true, false).map_err(|e| format!("{e:?}"))?;
    thread::sleep(Duration::from_millis(100));
    light_sensor.reset_internal_driver_state();
    thread::sleep(Duration::from_millis(100));
    

    let light_sensor_manufacturer_id = light_sensor.get_manufacturer_id().map_err(|e| format!("{e:?}"))?;
    let light_sensor_part_id = light_sensor.get_part_id().map_err(|e| format!("{e:?}"))?;
    
    tracing::info!("LTR559 Manufactured by {light_sensor_manufacturer_id:x} : {light_sensor_part_id:x}");        
    
    light_sensor
        .set_interrupt(InterruptPinPolarity::Low, InterruptMode::Both)
        .map_err(|e| format!("{e:?}"))?;
            
    light_sensor.set_ps_led(LedPulse::Pulse30, LedDutyCycle::_100, LedCurrent::_50mA).map_err(|e| format!("{e:?}"))?;
    light_sensor.set_ps_n_pulses(1).map_err(|e| format!("{e:?}"))?;
    
    light_sensor.set_als_contr(AlsGain::Gain4x, false, true).map_err(|e| format!("{e:?}"))?;
    light_sensor
        .set_ps_contr(true,true)
        .map_err(|e| format!("{e:?}"))?;

    
    light_sensor.set_ps_meas_rate(PsMeasRate::_100ms).map_err(|e| format!("{e:?}"))?;
    light_sensor
        .set_als_meas_rate(AlsIntTime::_50ms, AlsMeasRate::_50ms)
        .map_err(|e| format!("{e:?}"))?;
    
    light_sensor.set_ps_offset(0).map_err(|e| format!("{e:?}"))?;
    light_sensor.set_ps_low_limit_raw(0).map_err(|e| format!("{e:?}"))?;
    light_sensor.set_ps_high_limit_raw(1000).map_err(|e| format!("{e:?}"))?;
    Ok(light_sensor)
}

fn init_display(gpio: Gpio) -> Result<st7735_lcd::ST7735<Spi,rppal::gpio::OutputPin , NotConnected>, Box<dyn Error> >{
    let mut delay = Delay;
    let spi = Spi::new(Bus::Spi0, SlaveSelect::Ss1, SPI_SPEED, SPIMode::Mode0)?;

    let dc = gpio.get(9)?.into_output();
    let mut display = st7735_lcd::ST7735::new(spi, dc, NotConnected, false, true, 162, 132);
    display
        .init(&mut delay).map_err(|_| "Can't initialize the display")?;
    display
        .set_orientation(&Orientation::LandscapeSwapped).map_err(|_| "Can't set the orientation")?;

    let color = Rgb565::BLUE;
    display.clear(color).map_err(|_| "Can't clear the display")?;
    let color = Rgb565::RED;
    display.clear(color).map_err(|_| "Can't clear the display")?;
    Ok(display)
}

fn register_proximity_callback(gpio: Gpio) -> Result<(),Box<dyn Error>>{
    let mut proximity = gpio.get(4)?.into_input_pullup();
    proximity.set_async_interrupt(Trigger::FallingEdge, |c: Level| {
        tracing::warn!("Got interrupt level {c:?}");
    })?;
    Ok(())
}
