use ringbuffer::AllocRingBuffer;
use ringbuffer::RingBuffer;
use rppal::gpio::Gpio;
use std::sync::atomic::AtomicBool;
use std::sync::atomic::Ordering;
use std::sync::Arc;
use std::sync::Mutex;
use std::thread;
use systemstat::Duration;

use embedded_graphics::geometry::Size;
use std::error::Error;
use systemstat::{Platform, System};
use tracing_subscriber::{layer::SubscriberExt, util::SubscriberInitExt};

mod audio;
mod display;
mod pages;
mod sensors;

use display::Display;
use pages::PageTypes;
use sensors::Sensors;

const DISPLAY_SIZE_WIDTH: u32 = 160;
const DISPLAY_SIZE_HEIGTH: u32 = 80;
const DISPLAY_SIZE: Size = Size::new(DISPLAY_SIZE_WIDTH, DISPLAY_SIZE_HEIGTH);
const PROXIMITY_TRESHOLD: u16 = 1500;
const LUFS_WINDOW: usize = (audio::FRAMES * 64) as usize;
const NOISE_WINDOW: usize = (audio::FRAMES * 10) as usize;

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
    let backlight = Display::screen_on(&gpio)?;
    let display = Display::new(&gpio)?;
    let termination_flag = Arc::new(AtomicBool::new(false));
    let pages = pages::Pages::new(DISPLAY_SIZE);
    let mut sensors = Sensors::new()?;

    let mut rb = AllocRingBuffer::new(NOISE_WINDOW);
    rb.fill(0.0_f32);

    let mut power_rb = AllocRingBuffer::new(LUFS_WINDOW);
    power_rb.fill(0.0_f32);

    let buffer_producer = Arc::new(Mutex::new(rb));
    let buffer_consumer = buffer_producer.clone();

    let ctrl_termination_flag = termination_flag.clone();
    let handler_display = display.clone();

    let audio = audio::Audio::new(
        "adau7002",
        move |sample: Vec<f32>| -> Result<(), Box<dyn Error>> {
            if let Ok(mut buf) = buffer_producer.lock() {
                buf.extend(sample);
            }
            Ok(())
        },
    )?;

    signal_hook::flag::register(signal_hook::consts::SIGINT, ctrl_termination_flag.clone())?;
    signal_hook::flag::register(signal_hook::consts::SIGABRT, ctrl_termination_flag)?;

    display.display_page(pages.draw_page(&PageTypes::Boot)?)?;
    thread::sleep(Duration::from_millis(500));
    let mut page_selector = PageTypes::Error("Start");

    while !termination_flag.load(Ordering::Relaxed) {
        thread::sleep(Duration::from_millis(100));
        let mut task = || -> Result<(), Box<dyn Error>> {
            let light_measurement = sensors.measure_light_and_proximimty()?;
            let pressure_temp_humidity_measurement =
                sensors.measure_pressure_humidity_temperature()?;
            let proximity = light_measurement.proximity;
            let brightness = light_measurement.brightness;
            let pressure = pressure_temp_humidity_measurement.pressure;
            let temperature = pressure_temp_humidity_measurement.temperature;
            let humidity = pressure_temp_humidity_measurement.humidity;

            let cpu_temp = match sys.cpu_temp() {
                Ok(cpu_temp) => Some(cpu_temp),
                Err(x) => {
                    tracing::warn!("CPU temp: {}", x);
                    None
                }
            };
            let delta = cpu_temp.map(|temp| temp - temperature);

            if let Some(proximity) = proximity {
                let default_page = PageTypes::All((pressure, temperature, humidity, brightness));
                if proximity > PROXIMITY_TRESHOLD {
                    audio.stop_audio();
                    power_rb.clear();
                    power_rb.fill(0.0);
                    if let Ok(mut buf) = buffer_consumer.lock() {
                        buf.clear();
                        buf.fill(0.0);
                    }
                    page_selector = match page_selector {
                        PageTypes::All(_) => PageTypes::Temperature(temperature),
                        PageTypes::Temperature(_) => PageTypes::Pressure(pressure),
                        PageTypes::Pressure(_) => PageTypes::Humidity(humidity),
                        PageTypes::Humidity(_) => PageTypes::Brightness(brightness),
                        PageTypes::Brightness(_) => {
                            if audio.start_audio().is_err() {
                                PageTypes::Error("Error")
                            } else {
                                let data = if let Ok(buf) = buffer_consumer.lock() {
                                    buf.to_vec()
                                } else {
                                    vec![]
                                };
                                power_rb.extend(data.clone());
                                let lufs = audio::calculate_lufs(&power_rb.to_vec());
                                PageTypes::Noise((data, Some(lufs)))
                            }
                        }
                        PageTypes::Noise(_) => default_page,
                        PageTypes::Error(_) => default_page,
                        PageTypes::Boot => default_page,
                    };
                    tracing::info!("Selected page {page_selector}");
                } else {
                    page_selector = match page_selector {
                        PageTypes::All(_) => default_page,
                        PageTypes::Temperature(_) => PageTypes::Temperature(temperature),
                        PageTypes::Pressure(_) => PageTypes::Pressure(pressure),
                        PageTypes::Humidity(_) => PageTypes::Humidity(humidity),
                        PageTypes::Brightness(_) => PageTypes::Brightness(brightness),
                        PageTypes::Noise(_) => {
                            if audio.start_audio().is_err() {
                                PageTypes::Error("Error")
                            } else {
                                let data = if let Ok(buf) = buffer_consumer.lock() {
                                    buf.to_vec()
                                } else {
                                    vec![]
                                };
                                power_rb.extend(data.clone());
                                let lufs = audio::calculate_lufs(&power_rb.to_vec());
                                PageTypes::Noise((data, Some(lufs)))
                            }
                        }
                        PageTypes::Error(_) => default_page,
                        PageTypes::Boot => PageTypes::Boot,
                    };
                };
            };

            tracing::trace!(
		"Lux {:?} Proximity {:?} Relative Humidity = {}% Sensor Temperature = {} CPU temp = {:?} delta {:?} deg C, Pressure = {} hPa",
		brightness,
		proximity,
		humidity,
		temperature,
		cpu_temp,
		delta,
		pressure / 100.0
        );

            let page_canvas = pages.draw_page(&page_selector)?;

            handler_display.display_page(page_canvas)?;
            Ok(())
        };
        if let Err(e) = task() {
            tracing::error!("Timer {e:?}");
            termination_flag.store(true, Ordering::Relaxed);
        };
    }

    display.clear();
    Display::screen_off(backlight)?;
    Ok(())
}
