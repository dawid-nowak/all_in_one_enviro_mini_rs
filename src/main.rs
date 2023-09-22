use bme280::i2c::BME280;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::digital::v2::PinState;
use rppal::gpio::Gpio;
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
use st7735_lcd::Orientation;
use std::error::Error;
use systemstat::{Platform, System};
use tracing_subscriber::{layer::SubscriberExt, util::SubscriberInitExt};

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

fn main() -> Result<(), Box<dyn Error>> {
    tracing_subscriber::registry()
        .with(tracing_subscriber::EnvFilter::new(
            std::env::var("RUST_LOG").unwrap_or_else(|_| "debug".to_string()),
        ))
        .with(tracing_subscriber::fmt::layer())
        .init();

    tracing::info!("Hello PI");
    
    let sys = System::new();

    let spi = Spi::new(Bus::Spi0, SlaveSelect::Ss1, SPI_SPEED, SPIMode::Mode0)?;
    let gpio = Gpio::new()?;
    let dc = gpio.get(9)?.into_output();
    let mut backlight = gpio.get(12)?.into_output();

    backlight.set_low();
    thread::sleep(Duration::from_millis(100));
    backlight.set_high();
    thread::sleep(Duration::from_millis(100));
    

    let i2c = I2c::new()?;
    let mut bme280 = BME280::new_primary(i2c, Delay);
    bme280.init().unwrap();


    let i2c = I2c::new()?;
    let mut sensor = Ltr559::new_device(i2c, SlaveAddr::default());
    sensor
        .set_als_meas_rate(AlsIntTime::_50ms, AlsMeasRate::_50ms)
        .unwrap();
    sensor.set_als_contr(AlsGain::Gain4x, false, true).unwrap();

    let mut delay = Delay;
    let mut display = st7735_lcd::ST7735::new(spi, dc, NotConnected, false, true, 162, 132);
    display
        .init(&mut delay)
        .expect("Failed to initialize display");
    display
        .set_orientation(&Orientation::LandscapeSwapped)
        .unwrap();
    let color = Rgb565::BLUE;
    display.clear(color).unwrap();
    let color = Rgb565::RED;
    display.clear(color).unwrap();

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
        Text::new("Hello PiWorld", Point::new(x, y), small_style)
            .draw(&mut canvas)
            .map_err(|_| "Can't draw")?;
    }

    canvas
        .place_at(Point::new(0, 25))
        .draw(&mut display)
        .map_err(|_| "Can't draw")?;

    
    loop {
        let status = sensor.get_status().unwrap();

        let (lux, proximity) = if status.als_data_valid {
            let (_lux_raw_0, _lux_raw_1) = sensor.get_als_raw_data().unwrap();
            let lux = sensor.get_lux().unwrap();
            let proximity = sensor.get_ps_data().unwrap();
            (Some(lux), Some(proximity))
        } else {
            tracing::warn!("No lux");
            (None, None)
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
