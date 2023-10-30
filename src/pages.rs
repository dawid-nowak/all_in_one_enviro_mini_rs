use eg_seven_segment::SevenSegmentStyleBuilder;
use embedded_canvas::Canvas;
use embedded_graphics::geometry::Size;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::pixelcolor::RgbColor;
use embedded_graphics::primitives::{PrimitiveStyleBuilder, Rectangle};
use embedded_graphics::text::Text;
use embedded_graphics::{
    mono_font::{iso_8859_16::FONT_10X20, iso_8859_16::FONT_6X10, MonoTextStyle},
    prelude::*,
};
use embedded_graphics::image::Image;
use embedded_layout::layout::linear::spacing::DistributeFill;
use embedded_layout::{layout::linear::LinearLayout, prelude::*};
use plotters::backend::PixelFormat;
use plotters::backend::RGBPixel;
use plotters::prelude::BitMapBackend;
use plotters::prelude::ChartBuilder;
use plotters::prelude::IntoDrawingArea;
use plotters::series::LineSeries;
use plotters::style::GREEN;
use std::error::Error;
use core::fmt;
use tinybmp::Bmp;

#[derive(Clone)]
pub enum PageTypes {
    All((f32,f32,f32, Option<f32>)),
    Temperature(f32),
    Pressure(f32),
    Humidity(f32),
    Brightness(Option<f32>),
    Noise(Vec<f32>),
    Error(&'static str),
    Boot
}


impl fmt::Display for PageTypes {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
	let desc = match self{
	    Self::All(_) => "All",
	    Self::Temperature(_) => "Temperature",
	    Self::Pressure(_) => "Pressure",
	    Self::Humidity(_) => "Humidity",
	    Self::Brightness(_) => "Brightness",
	    Self::Noise(_) => "Noise",
	    Self::Error(_) => "Error",
	    Self::Boot => "Boot"
	};	    
        write!(f, "PageTypes({desc})")
    }
}


pub struct Pages {
    size: Size,
}

impl Pages {
    pub fn new(size: Size)->Self{
	Self{size}
    }
    pub fn draw_page(&self, page_selector: &PageTypes) -> Result<Canvas<Rgb565>, Box<dyn Error>> {
        match page_selector {
            PageTypes::All((pressure, temp, humidity, lux)) => Self::prepare_all_sensors_page(
                self.size,
                *pressure / 100.0,
                *temp,
                *humidity,
                lux.unwrap_or(f32::NAN),
            ),
            PageTypes::Temperature(temp) => {
                Self::prepare_sensor_page(self.size, "Temperature", *temp, "\u{00B0}C")
            }
            PageTypes::Pressure(pressure) => {
                Self::prepare_sensor_page(self.size, "Pressure", *pressure / 100.0, "hPa")
            }
            PageTypes::Humidity(humidity) => {
                Self::prepare_sensor_page(self.size, "Humidity", *humidity, "%")
            }
            PageTypes::Brightness(lux) => {
                Self::prepare_sensor_page(self.size, "Brightness", lux.unwrap_or(f32::NAN), "lux")
            }
            PageTypes::Noise(data) => {               
                    Self::prepare_noise_chart_page(self.size, &data)
            }
            PageTypes::Error(msg) => 
                Self::prepare_error_page(self.size, msg),
	    
            PageTypes::Boot => 
                Self::prepare_boot_page(self.size),
        }	
    }

    

    pub fn prepare_all_sensors_page(
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
        let pressure = Self::sensor_canvas(Size::new(size.width, 20), "Pressure", pressure, "hPa")?;
        let brightness =
            Self::sensor_canvas(Size::new(size.width, 20), "Brightness", brightness, "Lux")?;
        let temperature = Self::sensor_canvas(
            Size::new(size.width, 20),
            "Temperature",
            temperature,
            "\u{00b0}C",
        )?;
        let humidity = Self::sensor_canvas(Size::new(size.width, 20), "Humidity", humidity, "%")?;
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

        let bounding_box =
            Rectangle::new(Point::new(5, 5), Size::new(size.width - 5, size.height - 5));
        let rect = Rectangle::new(Point::new(0, 0), size).into_styled(style);

        ll.align_to(&bounding_box, horizontal::Left, vertical::Center)
            .draw(&mut canvas)?;
        rect.draw(&mut canvas)?;

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

    fn prepare_noise_chart_page(
        size: Size,
        sample: &[f32],
    ) -> Result<Canvas<Rgb565>, Box<dyn Error>> {
        let w = size.width;
        let h = size.height;
        let pixel_size = <RGBPixel as PixelFormat>::EFFECTIVE_PIXEL_SIZE as u32;

        let mut buffer = vec![0; (w * h * pixel_size) as usize];
        {
            let root = BitMapBackend::with_buffer(&mut buffer, (w, h)).into_drawing_area();
            let mut chart_builder = ChartBuilder::on(&root);

            chart_builder.margin(0);
            chart_builder.margin_bottom(0);
            chart_builder.margin_top(0);
            chart_builder.set_left_and_bottom_label_area_size(0);
            chart_builder.set_all_label_area_size(0);
            let mut chart_context =
                chart_builder.build_cartesian_2d(0..sample.len() as u32, -1.0_f32..1.0_f32)?;

            chart_context.configure_mesh().disable_mesh().draw()?;

            let data: Vec<(u32, f32)> = sample
                .iter()
                .enumerate()
                .map(|(i, s)| (i as u32, *s))
                .collect();
            let ls = LineSeries::new(data, GREEN);
            chart_context.draw_series(ls)?;
            root.present()?;
        }

        let mut canvas = Canvas::with_default_color(size, Rgb565::BLACK);

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
                    pixel_colors.push(pixel_color);
                } else {
                    tracing::warn!("No colors at pos {i} {j} {pos}");
                }
            }
        }
        let area = Rectangle::new(Point::new(0, 0), size);
        canvas.fill_contiguous(&area, pixel_colors)?;
        Ok(canvas)
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
}
