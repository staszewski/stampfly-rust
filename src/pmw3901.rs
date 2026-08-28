use anyhow::{Result, bail};
use esp_idf_hal::{
    delay::Ets,
    spi::{Operation, SpiDeviceDriver, SpiDriver},
};

use std::borrow::Borrow;
use std::{
    sync::atomic::{AtomicBool, AtomicI16, AtomicU8, AtomicU32, Ordering},
    thread,
    time::Duration,
};

const REGISTER_READ_ADDRESS_TO_DATA_DELAY_NS: u32 = 50_000;
const WRITE_ADDRESS_TO_DATA_DELAY_NS: u32 = 50_000;
const REGISTER_COMMAND_DELAY_US: u32 = 50;
const PMW3901_PRODUCT_ID: u8 = 0x49;
const PMW3901_INVERSE_PRODUCT_ID: u8 = 0xb6;
const REG_MOTION_BURST: u8 = 0x16;
const MOTION_BURST_BYTES: usize = 12;
const SAMPLER_TASK_PRIORITY: u32 = 4;

const INIT_REGISTERS_BEFORE_DELAY: &[(u8, u8)] = &[
    (0x7f, 0x00),
    (0x61, 0xad),
    (0x7f, 0x03),
    (0x40, 0x00),
    (0x7f, 0x05),
    (0x41, 0xb3),
    (0x43, 0xf1),
    (0x45, 0x14),
    (0x5b, 0x32),
    (0x5f, 0x34),
    (0x7b, 0x08),
    (0x7f, 0x06),
    (0x44, 0x1b),
    (0x40, 0xbf),
    (0x4e, 0x3f),
    (0x7f, 0x08),
    (0x65, 0x20),
    (0x6a, 0x18),
    (0x7f, 0x09),
    (0x4f, 0xaf),
    (0x5f, 0x40),
    (0x48, 0x80),
    (0x49, 0x80),
    (0x57, 0x77),
    (0x60, 0x78),
    (0x61, 0x78),
    (0x62, 0x08),
    (0x63, 0x50),
    (0x7f, 0x0a),
    (0x45, 0x60),
    (0x7f, 0x00),
    (0x4d, 0x11),
    (0x55, 0x80),
    (0x74, 0x1f),
    (0x75, 0x1f),
    (0x4a, 0x78),
    (0x4b, 0x78),
    (0x44, 0x08),
    (0x45, 0x50),
    (0x64, 0xff),
    (0x65, 0x1f),
    (0x7f, 0x14),
    (0x65, 0x60),
    (0x66, 0x08),
    (0x63, 0x78),
    (0x7f, 0x15),
    (0x48, 0x58),
    (0x7f, 0x07),
    (0x41, 0x0d),
    (0x43, 0x14),
    (0x4b, 0x0e),
    (0x45, 0x0f),
    (0x44, 0x42),
    (0x4c, 0x80),
    (0x7f, 0x10),
    (0x5b, 0x02),
    (0x7f, 0x07),
    (0x40, 0x41),
    (0x70, 0x00),
];

const INIT_REGISTERS_AFTER_DELAY: &[(u8, u8)] = &[
    (0x32, 0x44),
    (0x7f, 0x07),
    (0x40, 0x40),
    (0x7f, 0x06),
    (0x62, 0xf0),
    (0x63, 0x00),
    (0x7f, 0x0d),
    (0x48, 0xc0),
    (0x6f, 0xd5),
    (0x7f, 0x00),
    (0x5b, 0xa0),
    (0x4e, 0xa8),
    (0x5a, 0x50),
    (0x40, 0x80),
];

#[derive(Copy, Clone, Default)]
pub struct FlowSample {
    pub motion: u8,
    pub delta_x: i16,
    pub delta_y: i16,
    pub squal: u8,
}

#[derive(Copy, Clone, Default)]
pub struct CachedFlowSample {
    pub sampled: bool,
    pub sample: FlowSample,
    pub sequence: u32,
}

pub struct Pmw3901Sampler {
    ready: &'static AtomicBool,
    sequence: &'static AtomicU32,
    motion: &'static AtomicU8,
    delta_x: &'static AtomicI16,
    delta_y: &'static AtomicI16,
    squal: &'static AtomicU8,
}

static PMW_READY: AtomicBool = AtomicBool::new(false);
static PMW_SEQUENCE: AtomicU32 = AtomicU32::new(0);
static PMW_MOTION: AtomicU8 = AtomicU8::new(0);
static PMW_DELTA_X: AtomicI16 = AtomicI16::new(0);
static PMW_DELTA_Y: AtomicI16 = AtomicI16::new(0);
static PMW_SQUAL: AtomicU8 = AtomicU8::new(0);

pub struct Pmw3901<'d, T>
where
    T: Borrow<SpiDriver<'d>> + 'd,
{
    spi_device_driver: SpiDeviceDriver<'d, T>,
}

impl<'d, T> Pmw3901<'d, T>
where
    T: Borrow<SpiDriver<'d>> + 'd,
{
    pub fn new(spi_device_driver: SpiDeviceDriver<'d, T>) -> Self {
        Self { spi_device_driver }
    }

    pub fn initialize(&mut self) -> Result<()> {
        self.write_register(0x3a, 0x5a)?;
        Ets::delay_ms(5);

        let product_id = self.product_id()?;
        let inverse_product_id = self.inverse_product_id()?;
        if product_id != PMW3901_PRODUCT_ID || inverse_product_id != PMW3901_INVERSE_PRODUCT_ID {
            bail!(
                "PMW3901 identity check failed: product_id={product_id:#04x}, inverse={inverse_product_id:#04x}"
            );
        }

        for register in 0x02..=0x06 {
            let _ = self.read_register(register)?;
        }
        Ets::delay_ms(1);

        self.write_registers(INIT_REGISTERS_BEFORE_DELAY)?;
        Ets::delay_ms(100);
        self.write_registers(INIT_REGISTERS_AFTER_DELAY)?;

        Ok(())
    }

    pub fn product_id(&mut self) -> Result<u8> {
        let product_id = self.read_register(0x00)?;
        Ok(product_id)
    }

    pub fn inverse_product_id(&mut self) -> Result<u8> {
        self.read_register(0x5f)
    }

    pub fn motion(&mut self) -> Result<FlowSample> {
        let address = [REG_MOTION_BURST & 0x7f];
        let dummy = [0x00; MOTION_BURST_BYTES];
        let mut data = [0x00; MOTION_BURST_BYTES];

        let mut operations = [
            Operation::Write(&address),
            Operation::DelayNs(50_000),
            Operation::Transfer(&mut data, &dummy),
        ];
        self.spi_device_driver.transaction(&mut operations)?;

        Ok(FlowSample {
            motion: data[0],
            delta_x: i16::from_le_bytes([data[2], data[3]]),
            delta_y: i16::from_le_bytes([data[4], data[5]]),
            squal: data[6],
        })
    }

    pub fn flush_motion_samples(&mut self, samples: u32) -> Result<()> {
        for _ in 0..samples {
            let _ = self.motion()?;
        }

        Ok(())
    }

    pub fn start_sampler(mut self, sample_hz: u32) -> Result<Pmw3901Sampler>
    where
        Self: Send + 'static,
    {
        let sampler = Pmw3901Sampler {
            ready: &PMW_READY,
            sequence: &PMW_SEQUENCE,
            motion: &PMW_MOTION,
            delta_x: &PMW_DELTA_X,
            delta_y: &PMW_DELTA_Y,
            squal: &PMW_SQUAL,
        };
        let sample_period = Duration::from_millis(u64::from(1000 / sample_hz.max(1)));

        thread::Builder::new()
            .name("pmw-sampler".to_string())
            .stack_size(4096)
            .spawn(move || {
                unsafe {
                    esp_idf_hal::sys::vTaskPrioritySet(
                        core::ptr::null_mut(),
                        SAMPLER_TASK_PRIORITY,
                    );
                }
                loop {
                    match self.motion() {
                        Ok(sample) => {
                            PMW_MOTION.store(sample.motion, Ordering::Release);
                            PMW_DELTA_X.store(sample.delta_x, Ordering::Release);
                            PMW_DELTA_Y.store(sample.delta_y, Ordering::Release);
                            PMW_SQUAL.store(sample.squal, Ordering::Release);
                            PMW_SEQUENCE.fetch_add(1, Ordering::AcqRel);
                            PMW_READY.store(true, Ordering::Release);
                        }
                        Err(err) => {
                            log::warn!("PMW3901 background read failed: {err:?}");
                            PMW_MOTION.store(0, Ordering::Release);
                            PMW_DELTA_X.store(0, Ordering::Release);
                            PMW_DELTA_Y.store(0, Ordering::Release);
                            PMW_SQUAL.store(0, Ordering::Release);
                            PMW_SEQUENCE.fetch_add(1, Ordering::AcqRel);
                            PMW_READY.store(true, Ordering::Release);
                        }
                    }
                    thread::sleep(sample_period);
                }
            })?;

        Ok(sampler)
    }

    fn read_register(&mut self, address: u8) -> Result<u8> {
        let address = [address & 0x7f];
        let dummy = [0x00];
        let mut value = [0x00];

        let mut operations = [
            Operation::Write(&address),
            Operation::DelayNs(REGISTER_READ_ADDRESS_TO_DATA_DELAY_NS),
            Operation::Transfer(&mut value, &dummy),
        ];
        self.spi_device_driver.transaction(&mut operations)?;

        Ets::delay_us(REGISTER_COMMAND_DELAY_US);
        Ok(value[0])
    }

    fn write_registers(&mut self, registers: &[(u8, u8)]) -> Result<()> {
        for &(address, value) in registers {
            self.write_register(address, value)?;
        }

        Ok(())
    }

    fn write_register(&mut self, address: u8, value: u8) -> Result<()> {
        let address = [address | 0x80];
        let value = [value];

        let mut operations = [
            Operation::Write(&address),
            Operation::DelayNs(WRITE_ADDRESS_TO_DATA_DELAY_NS),
            Operation::Write(&value),
        ];
        self.spi_device_driver.transaction(&mut operations)?;

        Ets::delay_us(REGISTER_COMMAND_DELAY_US);

        Ok(())
    }
}

impl Pmw3901Sampler {
    pub fn latest_if_newer(&self, last_sequence: u32) -> CachedFlowSample {
        let sequence = self.sequence.load(Ordering::Acquire);
        if !self.ready.load(Ordering::Acquire) || sequence == last_sequence {
            return CachedFlowSample {
                sampled: false,
                sequence: last_sequence,
                ..CachedFlowSample::default()
            };
        }

        CachedFlowSample {
            sampled: true,
            sample: FlowSample {
                motion: self.motion.load(Ordering::Acquire),
                delta_x: self.delta_x.load(Ordering::Acquire),
                delta_y: self.delta_y.load(Ordering::Acquire),
                squal: self.squal.load(Ordering::Acquire),
            },
            sequence,
        }
    }
}
