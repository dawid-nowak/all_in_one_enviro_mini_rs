use chrono::Duration as ChronoDuration;
use ringbuffer::AllocRingBuffer;
use ringbuffer::RingBuffer;
use rppal::gpio::Gpio;
use std::sync::atomic::AtomicBool;
use std::sync::atomic::Ordering;
use std::sync::Arc;
use std::sync::Mutex;
use std::thread;
use systemstat::Duration;
use timer::Timer;

use embedded_graphics::geometry::Size;

use std::error::Error;
use systemstat::{Platform, System};
use tracing_subscriber::{layer::SubscriberExt, util::SubscriberInitExt};
mod audio;
mod pages;
mod display;
mod sensors;

use pages::PageTypes;
use display::Display;
use sensors::Sensors;

const DISPLAY_SIZE_WIDTH: u32 = 160;
const DISPLAY_SIZE_HEIGTH: u32 = 80;
const DISPLAY_SIZE: Size = Size::new(DISPLAY_SIZE_WIDTH, DISPLAY_SIZE_HEIGTH);
const PROXIMITY_TRESHOLD: u16 = 1500;


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
    let display = Display::new(&gpio)?;    
    let timer = Timer::new();
    let error = Arc::new(AtomicBool::new(false));
    let pages = pages::Pages::new(DISPLAY_SIZE);
    let mut sensors = Sensors::new()?;
    Display::screen_on(&gpio)?;
    
    let mut rb = AllocRingBuffer::new(audio::FRAMES as usize);
    rb.fill(0.0_f32);
    let buffer_producer = Arc::new(Mutex::new(rb));
    let buffer_consumer = buffer_producer.clone();

    
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
        handler_display.clear();
        std::process::exit(0);
    });

    display.display_page(pages.draw_page(&PageTypes::Boot)?)?;    
    thread::sleep(Duration::from_millis(500));
    let mut page_selector = PageTypes::Error("Start");      
       
    let guard = timer.schedule_repeating(ChronoDuration::milliseconds(500),move ||{

	let mut timer_task = ||->Result<(), Box<dyn Error>>{
	    
	    let light_measurement = sensors.measure_light_and_proximimty()?;
	    let pressure_temp_humidity_measurement =  sensors.measure_pressure_humidity_temperature()?;	    
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
		    page_selector = match page_selector {
			PageTypes::All(_) => PageTypes::Temperature(temperature),
			PageTypes::Temperature(_) => PageTypes::Pressure(pressure),
			PageTypes::Pressure(_) => PageTypes::Humidity(humidity),
			PageTypes::Humidity(_) => PageTypes::Brightness(brightness),
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
			PageTypes::Noise(_) => default_page,
			PageTypes::Error(_) => default_page,
			PageTypes::Boot => default_page,
			
		    };
		    tracing::info!("Selected page {page_selector}");
		    audio.stop_audio();
		}else{
		    page_selector = match page_selector {
			PageTypes::All(_) => default_page,
			PageTypes::Temperature(_) => PageTypes::Temperature(temperature),
			PageTypes::Pressure(_) => PageTypes::Pressure(pressure),
			PageTypes::Humidity(_) => PageTypes::Humidity(humidity),
			PageTypes::Brightness(_) => PageTypes::Brightness(brightness),
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
	    	    
	    display.display_page(page_canvas)?;
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






