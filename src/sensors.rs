use bme280::i2c::BME280;
use ltr_559::InterruptMode;
use ltr_559::InterruptPinPolarity;
use ltr_559::LedCurrent;
use ltr_559::LedDutyCycle;
use ltr_559::LedPulse;
use ltr_559::PsMeasRate;
use ltr_559::{AlsGain, AlsIntTime, AlsMeasRate, Ltr559, SlaveAddr};
use rppal::hal::Delay;
use rppal::i2c::I2c;
use std::error::Error;

use std::thread;
use systemstat::Duration;

pub struct LightMeasurement {
    pub brightness: Option<f32>,
    pub proximity: Option<u16>,
}

pub struct PressureHumidityTemperatureMeasurement {
    pub pressure: f32,
    pub humidity: f32,
    pub temperature: f32,
}

pub struct Sensors {
    bme280: BME280<I2c, Delay>,
    ltr559: Ltr559<I2c, ltr_559::ic::Ltr559>,
}
impl Sensors {
    pub fn new() -> Result<Self, Box<dyn Error>> {
        let bme280 = Self::init_bme_sensors()?;
        let ltr559 = Self::init_light_sensor()?;
        Ok(Self { bme280, ltr559 })
    }

    pub fn measure_light_and_proximimty(&mut self) -> Result<LightMeasurement, Box<dyn Error>> {
        let status = self
            .ltr559
            .get_status()
            .map_err(|_e| "Can't get light sensor status")?;
        let lux = if status.als_data_valid || status.als_interrupt_status {
            let (_lux_raw_0, _lux_raw_1) = self
                .ltr559
                .get_als_raw_data()
                .map_err(|_| "Can't get light raw data")?;
            let lux = self.ltr559.get_lux().map_err(|_| "Can't get light lux")?;
            Some(lux)
        } else {
            tracing::warn!("No lux");
            None
        };

        let proximity = if status.ps_data_status || status.ps_interrupt_status {
            let proximity = self
                .ltr559
                .get_ps_data()
                .map_err(|_| "Can't get proximity")?;
            Some(proximity.0)
        } else {
            tracing::warn!("No prox");
            None
        };
        Ok(LightMeasurement {
            brightness: lux,
            proximity,
        })
    }

    pub fn measure_pressure_humidity_temperature(
        &mut self,
    ) -> Result<PressureHumidityTemperatureMeasurement, Box<dyn Error>> {
        let measurements: bme280::Measurements<rppal::i2c::Error> = self
            .bme280
            .measure()
            .map_err(|_| "Can't get measurements from bme280")?;
        Ok(PressureHumidityTemperatureMeasurement {
            humidity: measurements.humidity,
            pressure: measurements.pressure,
            temperature: measurements.temperature,
        })
    }

    fn init_bme_sensors() -> Result<BME280<I2c, Delay>, Box<dyn Error>> {
        let mut bme280 = BME280::new_primary(I2c::new()?, Delay);
        bme280.init().map_err(|e| format!("{e:?}"))?;
        Ok(bme280)
    }

    fn init_light_sensor() -> Result<Ltr559<I2c, ltr_559::ic::Ltr559>, Box<dyn Error>> {
        let mut light_sensor = Ltr559::new_device(I2c::new()?, SlaveAddr::default());
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
}
