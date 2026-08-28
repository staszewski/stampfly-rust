use core::cell::RefCell;

use embedded_hal::i2c::{I2c, Operation, SevenBitAddress};
use esp_idf_hal::i2c::I2cDriver;

#[derive(Clone, Copy)]
pub struct SharedI2cBus {
    bus: &'static RefCell<I2cDriver<'static>>,
}

impl SharedI2cBus {
    pub fn new(bus: &'static RefCell<I2cDriver<'static>>) -> Self {
        Self { bus }
    }

    pub fn bus(self) -> &'static RefCell<I2cDriver<'static>> {
        self.bus
    }
}

impl embedded_hal::i2c::ErrorType for SharedI2cBus {
    type Error = <I2cDriver<'static> as embedded_hal::i2c::ErrorType>::Error;
}

impl I2c<SevenBitAddress> for SharedI2cBus {
    fn read(&mut self, address: u8, read: &mut [u8]) -> Result<(), Self::Error> {
        let mut bus = self.bus.borrow_mut();
        <I2cDriver<'static> as I2c<SevenBitAddress>>::read(&mut *bus, address, read)
    }

    fn write(&mut self, address: u8, write: &[u8]) -> Result<(), Self::Error> {
        let mut bus = self.bus.borrow_mut();
        <I2cDriver<'static> as I2c<SevenBitAddress>>::write(&mut *bus, address, write)
    }

    fn write_read(
        &mut self,
        address: u8,
        write: &[u8],
        read: &mut [u8],
    ) -> Result<(), Self::Error> {
        let mut bus = self.bus.borrow_mut();
        <I2cDriver<'static> as I2c<SevenBitAddress>>::write_read(&mut *bus, address, write, read)
    }

    fn transaction(
        &mut self,
        address: u8,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        let mut bus = self.bus.borrow_mut();
        <I2cDriver<'static> as I2c<SevenBitAddress>>::transaction(&mut *bus, address, operations)
    }
}
