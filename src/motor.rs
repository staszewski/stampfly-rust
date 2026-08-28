use esp_idf_hal::{
    gpio::{Gpio5, Gpio10, Gpio41, Gpio42},
    ledc::{
        CHANNEL0, CHANNEL1, CHANNEL2, CHANNEL3, LedcDriver, LedcTimerDriver, TIMER0,
        config::TimerConfig,
    },
    units::FromValueType,
};

use anyhow::{Result, bail};

pub const MAX_MOTOR_DUTY: u32 = 220;

pub struct MotorController<'a> {
    front_left: LedcDriver<'a>,
    front_right: LedcDriver<'a>,
    rear_left: LedcDriver<'a>,
    rear_right: LedcDriver<'a>,
    armed: bool,
}

pub struct MotorResources<'d> {
    pub timer: TIMER0<'d>,
    pub channel_front_left: CHANNEL0<'d>,
    pub channel_front_right: CHANNEL1<'d>,
    pub channel_rear_left: CHANNEL2<'d>,
    pub channel_rear_right: CHANNEL3<'d>,
    pub pin_front_left: Gpio5<'d>,
    pub pin_front_right: Gpio42<'d>,
    pub pin_rear_left: Gpio10<'d>,
    pub pin_rear_right: Gpio41<'d>,
}

#[derive(Copy, Clone, Default)]
pub struct MotorDuty {
    pub front_left: f32,
    pub front_right: f32,
    pub rear_left: f32,
    pub rear_right: f32,
}

impl<'d> MotorController<'d> {
    pub fn new(motor_resources: MotorResources<'d>) -> Result<Self> {
        let ledc_timer_driver = LedcTimerDriver::new(
            motor_resources.timer,
            &TimerConfig::default().frequency(150_u32.kHz().into()),
        )?;
        let mut front_left = LedcDriver::new(
            motor_resources.channel_front_left,
            &ledc_timer_driver,
            motor_resources.pin_front_left,
        )?;
        let mut front_right = LedcDriver::new(
            motor_resources.channel_front_right,
            &ledc_timer_driver,
            motor_resources.pin_front_right,
        )?;

        let mut rear_left = LedcDriver::new(
            motor_resources.channel_rear_left,
            &ledc_timer_driver,
            motor_resources.pin_rear_left,
        )?;
        let mut rear_right = LedcDriver::new(
            motor_resources.channel_rear_right,
            &ledc_timer_driver,
            motor_resources.pin_rear_right,
        )?;

        front_left.set_duty(0)?;
        front_right.set_duty(0)?;
        rear_left.set_duty(0)?;
        rear_right.set_duty(0)?;

        Ok(Self {
            front_left,
            front_right,
            rear_left,
            rear_right,
            armed: false,
        })
    }

    pub fn disarm(&mut self) -> Result<()> {
        self.armed = false;
        // Attempt every channel even if one LEDC write fails.
        let front_left = self.front_left.set_duty(0);
        let front_right = self.front_right.set_duty(0);
        let rear_left = self.rear_left.set_duty(0);
        let rear_right = self.rear_right.set_duty(0);
        if front_left.is_err() || front_right.is_err() || rear_left.is_err() || rear_right.is_err()
        {
            bail!("One or more motor channels failed to disarm");
        }
        Ok(())
    }

    pub fn arm(&mut self) -> Result<()> {
        self.armed = true;

        Ok(())
    }

    pub fn apply_duty(&mut self, requested: MotorDuty) -> Result<MotorDuty> {
        if !self.armed {
            bail!("Refusing to apply motor duty while disarmed");
        }

        if !requested.front_left.is_finite()
            || !requested.front_right.is_finite()
            || !requested.rear_left.is_finite()
            || !requested.rear_right.is_finite()
        {
            self.disarm()?;
            bail!("Refusing non-finite motor duty");
        }

        let duty = Self::bounded_duty(requested);

        self.front_left.set_duty(duty.front_left as u32)?;
        self.front_right.set_duty(duty.front_right as u32)?;
        self.rear_left.set_duty(duty.rear_left as u32)?;
        self.rear_right.set_duty(duty.rear_right as u32)?;

        Ok(duty)
    }

    fn bounded_duty(requested: MotorDuty) -> MotorDuty {
        let max_duty_f32 = MAX_MOTOR_DUTY as f32;

        MotorDuty {
            front_left: requested.front_left.clamp(0.0, max_duty_f32),
            front_right: requested.front_right.clamp(0.0, max_duty_f32),
            rear_left: requested.rear_left.clamp(0.0, max_duty_f32),
            rear_right: requested.rear_right.clamp(0.0, max_duty_f32),
        }
    }
}
