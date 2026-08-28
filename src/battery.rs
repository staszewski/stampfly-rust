use anyhow::Result;
use ina3221::INA3221;

use crate::shared_i2c::SharedI2cBus;

pub struct BatteryMonitor {
    ina3221_driver: INA3221<SharedI2cBus>,
}

impl BatteryMonitor {
    pub fn new(i2c_bus: SharedI2cBus) -> Self {
        let ina3221_driver = INA3221::new(i2c_bus, 0x40);
        BatteryMonitor { ina3221_driver }
    }

    pub fn voltage(&self) -> Result<f32> {
        Ok(self.ina3221_driver.get_bus_voltage(1)?.volts())
    }
}
