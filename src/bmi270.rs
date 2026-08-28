use std::borrow::Borrow;

use anyhow::Result;
use esp_idf_hal::{
    delay::{Ets, FreeRtos},
    spi::{SpiDeviceDriver, SpiDriver},
};

const REG_CHIP_ID: u8 = 0x00;
const REG_STATUS: u8 = 0x03;
const REG_ACC_DATA: u8 = 0x0C;
const REG_GYR_DATA: u8 = 0x12;
const REG_INTERNAL_STATUS: u8 = 0x21;
const REG_ACC_CONF: u8 = 0x40;
const REG_ACC_RANGE: u8 = 0x41;
const REG_GYR_CONF: u8 = 0x42;
const REG_GYR_RANGE: u8 = 0x43;
const REG_INIT_CTRL: u8 = 0x59;
const REG_INIT_ADDR_0: u8 = 0x5B;
const REG_INIT_ADDR_1: u8 = 0x5C;
const REG_INIT_DATA: u8 = 0x5E;
const REG_PWR_CONF: u8 = 0x7C;
const REG_PWR_CTRL: u8 = 0x7D;
const REG_CMD: u8 = 0x7E;

const INTERNAL_STATUS_INIT_OK: u8 = 0x01;
const STATUS_ACC_DRDY: u8 = 0x80;
const STATUS_GYR_DRDY: u8 = 0x40;
const PWR_CONF_ADV_POWER_SAVE: u8 = 0x01;
const PWR_CTRL_ACC_EN: u8 = 0x04;
const PWR_CTRL_GYR_EN: u8 = 0x02;
const INIT_CTRL_PREPARE_LOAD: u8 = 0x00;
const INIT_CTRL_COMPLETE_LOAD: u8 = 0x01;
const CMD_SOFT_RESET: u8 = 0xB6;

const ACC_CONF_400HZ_8G: u8 = 0xAA;
const ACC_RANGE_8G: u8 = 0x02;
const ACC_8G_COUNTS_PER_G: f32 = 4096.0;

const GYR_CONF_400HZ_2000DPS: u8 = 0xEA;
const GYR_RANGE_2000DPS: u8 = 0x00;
const GYR_2000DPS_SCALE: f32 = 2000.0 / 32768.0;

pub struct ImuSampleRaw {
    pub accel_g: (i16, i16, i16),
    pub gyro_dps: (i16, i16, i16),
}

pub struct ImuSample {
    pub accel_g: (f32, f32, f32),
    pub gyro_dps: (f32, f32, f32),
}

pub struct GyroStats {
    pub mean_dps: (f32, f32, f32),
    pub max_abs_dps: (f32, f32, f32),
}

pub struct Bmi270<'d, T>
where
    T: Borrow<SpiDriver<'d>> + 'd,
{
    spi_device_driver: SpiDeviceDriver<'d, T>,
    gyro_bias_dps: (f32, f32, f32),
}

impl<'d, T> Bmi270<'d, T>
where
    T: Borrow<SpiDriver<'d>> + 'd,
{
    pub fn new(spi_device_driver: SpiDeviceDriver<'d, T>) -> Self {
        Self {
            spi_device_driver,
            gyro_bias_dps: (0.0, 0.0, 0.0),
        }
    }

    pub fn initialize_config(&mut self, config: &[u8]) -> Result<()> {
        self.chip_id()?;
        self.soft_reset()?;
        // dummy read so we get 36 in the next read, not 255
        self.chip_id()?;
        self.chip_id()?;
        self.disable_advanced_power_save()?;
        self.prepare_config_load()?;
        self.upload_config_file(config)?;
        self.complete_config_load()?;

        // playing it safe; polling for internal status 1
        for _ in 0..20 {
            let status = self.internal_status()?;

            if status == INTERNAL_STATUS_INIT_OK {
                return Ok(());
            }

            Ets::delay_ms(1);
        }

        Err(anyhow::anyhow!("BMI270 config init timed out"))
    }

    pub fn initialize_accel_400hz_8g(&mut self) -> Result<()> {
        self.configure_accel_400hz_8g()?;
        self.enable_accel()?;
        Ets::delay_ms(5);

        for _ in 0..100 {
            let status = self.status()?;

            if status & STATUS_ACC_DRDY != 0 {
                return Ok(());
            }

            FreeRtos::delay_ms(1);
        }

        Err(anyhow::anyhow!("BMI270 accel data-ready timed out"))
    }

    pub fn imu_sample_corrected(&mut self) -> Result<ImuSample> {
        let imu_sample = self.accel_and_gyro_raw()?;

        let accel_x = imu_sample.accel_g.0 as f32 / ACC_8G_COUNTS_PER_G;
        let accel_y = imu_sample.accel_g.1 as f32 / ACC_8G_COUNTS_PER_G;
        let accel_z = imu_sample.accel_g.2 as f32 / ACC_8G_COUNTS_PER_G;

        let gyro_x = imu_sample.gyro_dps.0 as f32 * GYR_2000DPS_SCALE;
        let gyro_y = imu_sample.gyro_dps.1 as f32 * GYR_2000DPS_SCALE;
        let gyro_z = imu_sample.gyro_dps.2 as f32 * GYR_2000DPS_SCALE;

        let gyro_x_corrected = gyro_x - self.gyro_bias_dps.0;
        let gyro_y_corrected = gyro_y - self.gyro_bias_dps.1;
        let gyro_z_corrected = gyro_z - self.gyro_bias_dps.2;

        Ok(ImuSample {
            accel_g: (accel_x, accel_y, accel_z),
            gyro_dps: (gyro_x_corrected, gyro_y_corrected, gyro_z_corrected),
        })
    }

    pub fn initialize_gyro_400hz_2000dps(&mut self) -> Result<()> {
        self.write_register(REG_GYR_CONF, GYR_CONF_400HZ_2000DPS)?;
        self.write_register(REG_GYR_RANGE, GYR_RANGE_2000DPS)?;

        let gyr_conf = self.gyr_conf()?;
        let gyr_range = self.gyr_range()?;

        if gyr_conf != GYR_CONF_400HZ_2000DPS || gyr_range != GYR_RANGE_2000DPS {
            return Err(anyhow::anyhow!("Gyro configuration not accepted"));
        }

        let pwr_ctrl = self.enable_gyro()?;
        if pwr_ctrl & PWR_CTRL_GYR_EN == 0 {
            return Err(anyhow::anyhow!("Gyro enabled bit is missing"));
        }

        Ets::delay_ms(5);

        for _ in 0..100 {
            let status = self.status()?;

            if status & STATUS_GYR_DRDY != 0 {
                return Ok(());
            }

            FreeRtos::delay_ms(1);
        }

        Err(anyhow::anyhow!("BMI270 gyro data-ready timed out"))
    }

    pub fn gyro_dps(&mut self) -> Result<(f32, f32, f32)> {
        let (x_raw, y_raw, z_raw) = self.gyro_raw()?;
        let x = x_raw as f32 * GYR_2000DPS_SCALE;
        let y = y_raw as f32 * GYR_2000DPS_SCALE;
        let z = z_raw as f32 * GYR_2000DPS_SCALE;

        Ok((x, y, z))
    }

    pub fn corrected_gyro_stats(&mut self, samples: u32, delay: u32) -> Result<GyroStats> {
        if samples == 0 {
            return Err(anyhow::anyhow!("Cannot compute gyro stats on 0 samples"));
        }

        let mut x_sum = 0.0;
        let mut y_sum = 0.0;
        let mut z_sum = 0.0;
        let mut x_max_abs = 0.0;
        let mut y_max_abs = 0.0;
        let mut z_max_abs = 0.0;

        for _ in 0..samples {
            let imu = self.imu_sample_corrected()?;
            let (x, y, z) = imu.gyro_dps;
            x_sum += x;
            y_sum += y;
            z_sum += z;
            if x.abs() > x_max_abs {
                x_max_abs = x.abs();
            }
            if y.abs() > y_max_abs {
                y_max_abs = y.abs();
            }
            if z.abs() > z_max_abs {
                z_max_abs = z.abs();
            }
            FreeRtos::delay_ms(delay);
        }

        Ok(GyroStats {
            mean_dps: (
                x_sum / samples as f32,
                y_sum / samples as f32,
                z_sum / samples as f32,
            ),
            max_abs_dps: (x_max_abs, y_max_abs, z_max_abs),
        })
    }

    pub fn calibrate_gyro_bias(&mut self, samples: u32, delay: u32) -> Result<(f32, f32, f32)> {
        if samples == 0 {
            return Err(anyhow::anyhow!("Cannot calibrate on 0 samples"));
        }
        let mut x_sum = 0.0;
        let mut y_sum = 0.0;
        let mut z_sum = 0.0;
        for _ in 0..samples {
            let (x, y, z) = self.gyro_dps()?;
            x_sum += x;
            y_sum += y;
            z_sum += z;
            FreeRtos::delay_ms(delay);
        }
        let bias_x = x_sum / samples as f32;
        let bias_y = y_sum / samples as f32;
        let bias_z = z_sum / samples as f32;

        self.gyro_bias_dps = (bias_x, bias_y, bias_z);

        Ok((bias_x, bias_y, bias_z))
    }

    fn enable_gyro(&mut self) -> Result<u8> {
        let pwr_ctrl = self.pwr_ctrl()?;
        let new_pwr_ctrl = pwr_ctrl | PWR_CTRL_GYR_EN;
        self.write_register(REG_PWR_CTRL, new_pwr_ctrl)?;
        let current_pwr_ctrl = self.pwr_ctrl()?;
        Ok(current_pwr_ctrl)
    }

    fn accel_and_gyro_raw(&mut self) -> Result<ImuSampleRaw> {
        let mut data = [0u8; 12];
        self.read_12_registers(REG_ACC_DATA, &mut data)?;
        let accel_x = i16::from_le_bytes([data[0], data[1]]);
        let accel_y = i16::from_le_bytes([data[2], data[3]]);
        let accel_z = i16::from_le_bytes([data[4], data[5]]);

        let gyro_x = i16::from_le_bytes([data[6], data[7]]);
        let gyro_y = i16::from_le_bytes([data[8], data[9]]);
        let gyro_z = i16::from_le_bytes([data[10], data[11]]);

        Ok(ImuSampleRaw {
            accel_g: (accel_x, accel_y, accel_z),
            gyro_dps: (gyro_x, gyro_y, gyro_z),
        })
    }

    fn gyro_raw(&mut self) -> Result<(i16, i16, i16)> {
        let mut data = [0u8; 6];
        self.read_6_registers(REG_GYR_DATA, &mut data)?;
        let x = i16::from_le_bytes([data[0], data[1]]);
        let y = i16::from_le_bytes([data[2], data[3]]);
        let z = i16::from_le_bytes([data[4], data[5]]);

        Ok((x, y, z))
    }

    pub fn gyr_conf(&mut self) -> Result<u8> {
        let gyr_conf = self.read_register(REG_GYR_CONF)?;
        Ok(gyr_conf)
    }

    pub fn gyr_range(&mut self) -> Result<u8> {
        let gyr_range = self.read_register(REG_GYR_RANGE)?;
        Ok(gyr_range)
    }

    fn chip_id(&mut self) -> Result<u8> {
        let chip_id = self.read_register(REG_CHIP_ID)?;
        Ok(chip_id)
    }

    fn status(&mut self) -> Result<u8> {
        let status = self.read_register(REG_STATUS)?;
        Ok(status)
    }

    fn internal_status(&mut self) -> Result<u8> {
        let internal_status = self.read_register(REG_INTERNAL_STATUS)?;
        Ok(internal_status)
    }

    fn pwr_conf(&mut self) -> Result<u8> {
        let pwr_conf = self.read_register(REG_PWR_CONF)?;
        Ok(pwr_conf)
    }

    fn pwr_ctrl(&mut self) -> Result<u8> {
        let pwr_ctrl = self.read_register(REG_PWR_CTRL)?;
        Ok(pwr_ctrl)
    }

    fn init_ctrl(&mut self) -> Result<u8> {
        let init_ctrl = self.read_register(REG_INIT_CTRL)?;
        Ok(init_ctrl)
    }

    fn acc_conf(&mut self) -> Result<u8> {
        let acc_conf = self.read_register(REG_ACC_CONF)?;
        Ok(acc_conf)
    }

    fn acc_range(&mut self) -> Result<u8> {
        let acc_range = self.read_register(REG_ACC_RANGE)?;
        Ok(acc_range)
    }

    fn prepare_config_load(&mut self) -> Result<u8> {
        self.write_register(REG_INIT_CTRL, INIT_CTRL_PREPARE_LOAD)?;
        let init_ctrl = self.init_ctrl()?;
        Ok(init_ctrl)
    }

    fn set_init_addr(&mut self, index: u16) -> Result<()> {
        let addr = index / 2;
        let init_addr_0 = (addr & 0x0F) as u8;
        let init_addr_1 = (addr >> 4) as u8;

        self.write_register(REG_INIT_ADDR_0, init_addr_0)?;
        self.write_register(REG_INIT_ADDR_1, init_addr_1)?;

        Ok(())
    }

    fn disable_advanced_power_save(&mut self) -> Result<u8> {
        let pwr_conf = self.pwr_conf()?;
        let new_value = pwr_conf & !PWR_CONF_ADV_POWER_SAVE;
        self.write_register(REG_PWR_CONF, new_value)?;
        Ets::delay_us(450);
        let new_pwr_conf = self.pwr_conf()?;
        Ok(new_pwr_conf)
    }

    fn configure_accel_400hz_8g(&mut self) -> Result<(u8, u8)> {
        self.write_register(REG_ACC_CONF, ACC_CONF_400HZ_8G)?;
        self.write_register(REG_ACC_RANGE, ACC_RANGE_8G)?;

        let acc_conf = self.acc_conf()?;
        let acc_range = self.acc_range()?;

        Ok((acc_conf, acc_range))
    }

    fn enable_accel(&mut self) -> Result<u8> {
        let pwr_ctrl = self.pwr_ctrl()?;
        let new_pwr_ctrl = pwr_ctrl | PWR_CTRL_ACC_EN;
        self.write_register(REG_PWR_CTRL, new_pwr_ctrl)?;
        let current_pwr_ctrl = self.pwr_ctrl()?;
        Ok(current_pwr_ctrl)
    }

    fn soft_reset(&mut self) -> Result<()> {
        self.write_register(REG_CMD, CMD_SOFT_RESET)?;
        Ets::delay_ms(10);

        Ok(())
    }

    fn complete_config_load(&mut self) -> Result<u8> {
        self.write_register(REG_INIT_CTRL, INIT_CTRL_COMPLETE_LOAD)?;
        let internal_status = self.internal_status()?;

        Ok(internal_status)
    }

    fn upload_config_file(&mut self, config: &[u8]) -> Result<()> {
        let mut index = 0;
        let chunk_size = 32;

        while index < config.len() {
            let end = (index + chunk_size).min(config.len());
            let chunk = &config[index..end];
            self.upload_config_chunk(index as u16, chunk)?;
            index = end;
        }
        Ok(())
    }

    fn upload_config_chunk(&mut self, index: u16, data: &[u8]) -> Result<()> {
        self.set_init_addr(index)?;
        self.write_init_data(data)?;

        Ok(())
    }

    fn write_init_data(&mut self, data: &[u8]) -> Result<()> {
        self.write_registers(REG_INIT_DATA, data)?;

        Ok(())
    }

    fn read_register(&mut self, address: u8) -> Result<u8> {
        let mut read = [0x00, 0x00, 0x00];
        let write = [address | 0x80, 0x00, 0x00];
        self.spi_device_driver.transfer(&mut read, &write)?;

        Ok(read[2])
    }

    fn read_12_registers(&mut self, address: u8, data: &mut [u8]) -> Result<()> {
        let mut read = [0u8; 14];
        let mut write = [0u8; 14];
        write[0] = address | 0x80;
        self.spi_device_driver.transfer(&mut read, &write)?;
        data.copy_from_slice(&read[2..]);

        Ok(())
    }

    fn read_6_registers(&mut self, address: u8, data: &mut [u8]) -> Result<()> {
        let mut read = [0u8; 8];
        let mut write = [0u8; 8];
        write[0] = address | 0x80;
        self.spi_device_driver.transfer(&mut read, &write)?;
        data.copy_from_slice(&read[2..]);

        Ok(())
    }

    fn write_register(&mut self, address: u8, value: u8) -> Result<()> {
        let mut read = [0x00, 0x00];
        let write = [address & 0x7F, value];
        self.spi_device_driver.transfer(&mut read, &write)?;

        Ok(())
    }

    fn write_registers(&mut self, address: u8, data: &[u8]) -> Result<()> {
        let mut write = Vec::with_capacity(data.len() + 1);
        write.push(address & 0x7F);
        write.extend_from_slice(data);
        let mut read = vec![0x00; write.len()];
        self.spi_device_driver.transfer(&mut read, &write)?;

        Ok(())
    }
}
