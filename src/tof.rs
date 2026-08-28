use core::{
    cell::RefCell,
    slice,
    sync::atomic::{AtomicBool, AtomicI16, AtomicU8, AtomicU32, Ordering},
};
use std::{thread, time::Duration};

use anyhow::{Result, bail};
use esp_idf_hal::{
    delay::FreeRtos,
    gpio::{Output, OutputPin, PinDriver},
    i2c::I2cDriver,
    sys::TickType_t,
};

const TOF_I2C_TIMEOUT_TICKS: TickType_t = 50;
const SAMPLER_TASK_PRIORITY: u32 = 4;
static mut I2C_BUS: *const RefCell<I2cDriver<'static>> = core::ptr::null();

#[derive(Copy, Clone, Debug, Default)]
pub struct TofIdentity {
    pub model_id: u8,
    pub module_type: u8,
}

#[derive(Copy, Clone, Debug, Default)]
pub struct TofSample {
    pub range_mm: i16,
    pub status: u8,
    pub objects: u8,
}

pub struct DownwardTof<'d> {
    _xshut_bottom: PinDriver<'d, Output>,
    _xshut_front: PinDriver<'d, Output>,
}

#[derive(Copy, Clone, Debug, Default)]
pub struct CachedTofSample {
    pub sampled: bool,
    pub range_mm: i16,
    pub status: u8,
    pub objects: u8,
    pub sequence: u32,
    pub timestamp_us: u32,
}

pub struct TofSampler {
    ready: &'static AtomicBool,
    sequence: &'static AtomicU32,
    range_mm: &'static AtomicI16,
    status: &'static AtomicU8,
    objects: &'static AtomicU8,
    timestamp_us: &'static AtomicU32,
}

static TOF_READY: AtomicBool = AtomicBool::new(false);
static TOF_SEQUENCE: AtomicU32 = AtomicU32::new(0);
static TOF_RANGE_MM: AtomicI16 = AtomicI16::new(0);
static TOF_STATUS: AtomicU8 = AtomicU8::new(255);
static TOF_OBJECTS: AtomicU8 = AtomicU8::new(0);
static TOF_TIMESTAMP_US: AtomicU32 = AtomicU32::new(0);

unsafe extern "C" {
    fn rust_fly_tof_bottom_init() -> i32;
    fn rust_fly_tof_bottom_model_id(model_id: *mut u8, module_type: *mut u8) -> i32;
    fn rust_fly_tof_bottom_read_range(
        range_mm: *mut i16,
        range_status: *mut u8,
        objects: *mut u8,
    ) -> i32;
}

pub fn set_i2c_bus(bus: &'static RefCell<I2cDriver<'static>>) {
    unsafe {
        I2C_BUS = bus;
    }
}

impl<'d> DownwardTof<'d> {
    pub fn new(
        xshut_bottom_pin: impl OutputPin + 'd,
        xshut_front_pin: impl OutputPin + 'd,
    ) -> Result<Self> {
        let mut xshut_bottom = PinDriver::output(xshut_bottom_pin)?;
        let mut xshut_front = PinDriver::output(xshut_front_pin)?;

        xshut_bottom.set_low()?;
        xshut_front.set_low()?;
        FreeRtos::delay_ms(10);

        xshut_bottom.set_high()?;
        FreeRtos::delay_ms(100);

        let status = unsafe { rust_fly_tof_bottom_init() };
        if status != 0 {
            bail!("VL53L3 bottom init failed with status {status}");
        }

        Ok(Self {
            _xshut_bottom: xshut_bottom,
            _xshut_front: xshut_front,
        })
    }

    pub fn identity(&mut self) -> Result<TofIdentity> {
        let mut model_id = 0;
        let mut module_type = 0;

        let status = unsafe { rust_fly_tof_bottom_model_id(&mut model_id, &mut module_type) };
        if status != 0 {
            bail!("VL53L3 bottom identity read failed with status {status}");
        }

        Ok(TofIdentity {
            model_id,
            module_type,
        })
    }

    pub fn read_range(&mut self) -> Result<TofSample> {
        let mut range_mm = 0;
        let mut status = 255;
        let mut objects = 0;

        let driver_status =
            unsafe { rust_fly_tof_bottom_read_range(&mut range_mm, &mut status, &mut objects) };
        if driver_status != 0 {
            bail!("VL53L3 bottom range read failed with status {driver_status}");
        }

        Ok(TofSample {
            range_mm,
            status,
            objects,
        })
    }

    pub fn start_sampler(mut self, sample_hz: u32) -> Result<TofSampler>
    where
        Self: Send + 'static,
    {
        let sampler = TofSampler {
            ready: &TOF_READY,
            sequence: &TOF_SEQUENCE,
            range_mm: &TOF_RANGE_MM,
            status: &TOF_STATUS,
            objects: &TOF_OBJECTS,
            timestamp_us: &TOF_TIMESTAMP_US,
        };
        let sample_period = Duration::from_millis(u64::from(1000 / sample_hz.max(1)));

        thread::Builder::new()
            .name("tof-sampler".to_string())
            .stack_size(4096)
            .spawn(move || {
                unsafe {
                    esp_idf_hal::sys::vTaskPrioritySet(
                        core::ptr::null_mut(),
                        SAMPLER_TASK_PRIORITY,
                    );
                }
                loop {
                    match self.read_range() {
                        Ok(sample) => {
                            // VL53L3 reports RANGE_STATUS_NONE with zero
                            // objects while a conversion is still running.
                            // That is not a bad measurement and must not
                            // overwrite the last valid height used by flow.
                            if sample.objects == 0 && sample.range_mm == 0 {
                                thread::sleep(sample_period);
                                continue;
                            }
                            TOF_RANGE_MM.store(sample.range_mm, Ordering::Release);
                            TOF_STATUS.store(sample.status, Ordering::Release);
                            TOF_OBJECTS.store(sample.objects, Ordering::Release);
                            TOF_TIMESTAMP_US.store(timestamp_us(), Ordering::Release);
                            TOF_SEQUENCE.fetch_add(1, Ordering::AcqRel);
                            TOF_READY.store(true, Ordering::Release);
                        }
                        Err(err) => {
                            log::warn!("VL53L3 bottom background read failed: {err:?}");
                            TOF_STATUS.store(255, Ordering::Release);
                            TOF_OBJECTS.store(0, Ordering::Release);
                            TOF_TIMESTAMP_US.store(timestamp_us(), Ordering::Release);
                            TOF_SEQUENCE.fetch_add(1, Ordering::AcqRel);
                            TOF_READY.store(true, Ordering::Release);
                        }
                    }
                    thread::sleep(sample_period);
                }
            })?;

        Ok(sampler)
    }
}

impl TofSampler {
    pub fn latest_if_newer(&self, last_sequence: u32) -> CachedTofSample {
        let sequence = self.sequence.load(Ordering::Acquire);
        if !self.ready.load(Ordering::Acquire) || sequence == last_sequence {
            return CachedTofSample {
                sampled: false,
                sequence: last_sequence,
                ..CachedTofSample::default()
            };
        }

        CachedTofSample {
            sampled: true,
            range_mm: self.range_mm.load(Ordering::Acquire),
            status: self.status.load(Ordering::Acquire),
            objects: self.objects.load(Ordering::Acquire),
            sequence,
            timestamp_us: self.timestamp_us.load(Ordering::Acquire),
        }
    }
}

fn timestamp_us() -> u32 {
    unsafe { esp_idf_hal::sys::esp_timer_get_time() as u32 }
}

#[unsafe(no_mangle)]
extern "C" fn rust_fly_tof_i2c_write(address: u8, data: *const u8, len: u32) -> i32 {
    let Some(bus) = tof_i2c_bus() else {
        return -1;
    };
    if data.is_null() {
        return -1;
    }

    let bytes = unsafe { slice::from_raw_parts(data, len as usize) };
    match bus
        .borrow_mut()
        .write(address, bytes, TOF_I2C_TIMEOUT_TICKS)
    {
        Ok(()) => 0,
        Err(err) => err.code(),
    }
}

#[unsafe(no_mangle)]
extern "C" fn rust_fly_tof_i2c_write_read(
    address: u8,
    write_data: *const u8,
    write_len: u32,
    read_data: *mut u8,
    read_len: u32,
) -> i32 {
    let Some(bus) = tof_i2c_bus() else {
        return -1;
    };
    if write_data.is_null() || read_data.is_null() {
        return -1;
    }

    let write = unsafe { slice::from_raw_parts(write_data, write_len as usize) };
    let read = unsafe { slice::from_raw_parts_mut(read_data, read_len as usize) };

    match bus
        .borrow_mut()
        .write_read(address, write, read, TOF_I2C_TIMEOUT_TICKS)
    {
        Ok(()) => 0,
        Err(err) => err.code(),
    }
}

#[unsafe(no_mangle)]
extern "C" fn rust_fly_tof_delay_us(delay_us: u32) {
    esp_idf_hal::delay::Ets::delay_us(delay_us);
}

#[unsafe(no_mangle)]
extern "C" fn rust_fly_tof_millis() -> u32 {
    let micros = unsafe { esp_idf_hal::sys::esp_timer_get_time() };
    (micros / 1000) as u32
}

fn tof_i2c_bus() -> Option<&'static RefCell<I2cDriver<'static>>> {
    let bus = unsafe { I2C_BUS.as_ref() };
    bus
}
