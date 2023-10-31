use embedded_canvas::Canvas;
use embedded_graphics::draw_target::DrawTarget;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::pixelcolor::RgbColor;
use embedded_graphics::prelude::*;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::digital::v2::PinState;
use rppal::gpio::Gpio;
use rppal::hal::Delay;
use rppal::spi::Bus;
use rppal::spi::Mode as SPIMode;
use rppal::spi::SlaveSelect;
use rppal::spi::Spi;
use st7735_lcd::Orientation;
use st7735_lcd::ST7735;
use std::error::Error;
use std::sync::Arc;
use std::sync::Mutex;
use std::thread;
use systemstat::Duration;

const DC_PIN: u8 = 9;
const SPI_SPEED: u32 = 10_000_000;
const DISPLAY_HORIZONTAL_OFFSET: i32 = 0;
const DISPLAY_VERTICAL_OFFSET: i32 = 25;
const DISPLAY_OFFSET: Point = Point::new(DISPLAY_HORIZONTAL_OFFSET, DISPLAY_VERTICAL_OFFSET);
const BACKLIGHT_PIN: u8 = 12;

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

#[derive(Clone)]
pub struct Display {
    inner: Arc<Mutex<ST7735<Spi, rppal::gpio::OutputPin, NotConnected>>>,
}

impl Display {
    pub fn new(gpio: &Gpio) -> Result<Self, Box<dyn Error>> {
        let mut delay = Delay;
        let spi = Spi::new(Bus::Spi0, SlaveSelect::Ss1, SPI_SPEED, SPIMode::Mode0)?;

        let dc = gpio.get(DC_PIN)?.into_output();
        let mut display = ST7735::new(spi, dc, NotConnected, false, true, 162, 132);
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

        Ok(Self {
            inner: Arc::new(Mutex::new(display)),
        })
    }

    pub fn clear(&self) {
        let color = Rgb565::BLACK;
        if let Ok(mut disp) = self.inner.lock() {
            if let Err(e) = disp.clear(color) {
                tracing::error!("Can't clear the screen {:?}", e);
            }
        }
    }

    pub fn display_page(&self, canvas: Canvas<Rgb565>) -> Result<(), Box<dyn Error>> {
        if let Ok(mut disp) = self.inner.lock() {
            let canvas_at = canvas.place_at(DISPLAY_OFFSET);
            let bb = canvas_at.bounding_box();
            let pixels_iter = bb.points().filter_map(|point| canvas_at.get_pixel(point));
            disp.fill_contiguous(&bb, pixels_iter)
                .map_err(|_| "Can't draw".into())
        } else {
            Err("Can't lock".into())
        }
    }

    pub fn screen_on(gpio: &Gpio) -> Result<(), Box<dyn Error>> {
        let mut backlight = gpio.get(BACKLIGHT_PIN)?.into_output();
        backlight.set_low();
        thread::sleep(Duration::from_millis(100));
        backlight.set_high();
        thread::sleep(Duration::from_millis(100));
        Ok(())
    }
}
