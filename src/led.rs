use core::time::Duration;

use anyhow::Result;
use esp_idf_hal::{
    gpio::Gpio21,
    rmt::{
        PinState, Symbol, TxChannelDriver,
        config::{MemoryAccess, TxChannelConfig},
        encoder::simple_encoder::{EncoderCallback, NotEnoughSpace, SimpleEncoder, SymbolBuffer},
    },
    units::Hertz,
};

const RMT_LED_RESOLUTION_HZ: Hertz = Hertz(10_000_000);

const T0H: Duration = Duration::from_nanos(350);
const T0L: Duration = Duration::from_nanos(800);
const T1H: Duration = Duration::from_nanos(700);
const T1L: Duration = Duration::from_nanos(600);
const TRESET: Duration = Duration::from_micros(281);

#[derive(Clone, Copy)]
pub struct Color {
    red: u8,
    green: u8,
    blue: u8,
}

impl Color {
    pub const fn new(red: u8, green: u8, blue: u8) -> Self {
        Self { red, green, blue }
    }

    fn to_vec(self) -> Vec<Symbol> {
        let mut result = Vec::with_capacity(24);
        for byte in [self.green, self.red, self.blue] {
            result.extend(byte_to_symbols(byte));
        }
        result
    }
}

pub struct OnboardLed<'d> {
    channel: TxChannelDriver<'d>,
}

impl<'d> OnboardLed<'d> {
    pub fn new(pin: Gpio21<'d>) -> Result<Self> {
        let channel = TxChannelDriver::new(
            pin,
            &TxChannelConfig {
                clock_source: Default::default(),
                memory_access: MemoryAccess::Indirect {
                    memory_block_symbols: 64,
                },
                resolution: RMT_LED_RESOLUTION_HZ,
                transaction_queue_depth: 4,
                ..Default::default()
            },
        )?;

        Ok(Self { channel })
    }

    pub fn set_color(&mut self, color: Color) -> Result<()> {
        let encoder = SimpleEncoder::with_config(LedEncoder::new(), &Default::default())?;
        self.channel
            .send_and_wait(encoder, &[color], &Default::default())?;

        Ok(())
    }
}

struct LedEncoder {
    input_position: usize,
}

impl LedEncoder {
    fn new() -> Self {
        Self { input_position: 0 }
    }
}

impl EncoderCallback for LedEncoder {
    type Item = Color;

    fn encode(
        &mut self,
        input_data: &[Self::Item],
        buffer: &mut SymbolBuffer<'_>,
    ) -> core::result::Result<(), NotEnoughSpace> {
        if buffer.position() == 0 {
            self.input_position = 0;
        }

        let reset =
            Symbol::new_half_split(RMT_LED_RESOLUTION_HZ, PinState::Low, PinState::Low, TRESET)
                .unwrap();

        for color in &input_data[self.input_position..] {
            let mut symbols = color.to_vec();
            symbols.push(reset);
            buffer.write_all(&symbols)?;
            self.input_position += 1;
        }

        Ok(())
    }
}

fn byte_to_symbols(byte: u8) -> impl IntoIterator<Item = Symbol> {
    let zero = Symbol::new_with(
        RMT_LED_RESOLUTION_HZ,
        PinState::High,
        T0H,
        PinState::Low,
        T0L,
    )
    .unwrap();

    let one = Symbol::new_with(
        RMT_LED_RESOLUTION_HZ,
        PinState::High,
        T1H,
        PinState::Low,
        T1L,
    )
    .unwrap();

    (0..8)
        .map(|i| 0x80 >> i)
        .map(move |bitmask| (byte & bitmask) != 0)
        .map(move |is_one| if is_one { one } else { zero })
}
