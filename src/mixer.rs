pub struct MotorInput {
    pub thrust: f32,
    pub pitch: f32,
    pub roll: f32,
    pub yaw: f32,
}

pub struct MotorMix {
    pub front_left: f32,
    pub front_right: f32,
    pub rear_left: f32,
    pub rear_right: f32,
    pub collective_shift: f32,
    pub pitch_mix_factor: f32,
    pub roll_mix_factor: f32,
}

pub struct MotorMixer;

pub const MAX_MOTOR_DUTY: u32 = 220;

impl MotorMixer {
    pub fn mix(input: MotorInput) -> MotorMix {
        let pitch_mix_factor = ATTITUDE_MIX_FACTOR;
        let roll_mix_factor = ATTITUDE_MIX_FACTOR;
        let pitch = input.pitch * pitch_mix_factor;
        let roll = input.roll * roll_mix_factor;
        let yaw = input.yaw * YAW_MIX_FACTOR;

        let front_left = input.thrust + roll + pitch - yaw;
        let front_right = input.thrust - roll + pitch + yaw;
        let rear_left = input.thrust + roll - pitch + yaw;
        let rear_right = input.thrust - roll - pitch - yaw;

        // Preserve attitude differentials when collective plus correction would
        // exceed a motor bound. A common shift changes collective only; it does
        // not weaken the requested pitch, roll, or yaw torque.
        let minimum = front_left.min(front_right).min(rear_left).min(rear_right);
        let maximum = front_left.max(front_right).max(rear_left).max(rear_right);
        let minimum_shift = MIN_MOTOR_DUTY - minimum;
        let maximum_shift = MAX_MOTOR_DUTY as f32 - maximum;
        debug_assert!(minimum_shift <= maximum_shift);
        let collective_shift = if minimum_shift > 0.0 {
            minimum_shift
        } else if maximum_shift < 0.0 {
            maximum_shift
        } else {
            0.0
        };

        MotorMix {
            front_left: front_left + collective_shift,
            front_right: front_right + collective_shift,
            rear_left: rear_left + collective_shift,
            rear_right: rear_right + collective_shift,
            collective_shift,
            pitch_mix_factor,
            roll_mix_factor,
        }
    }
}

// Run194 accepted every clean flow sample and held a roughly -8 degree pitch
// target, but measured pitch stayed near level while the bounded controller
// already requested about -10..-20 correction. The vehicle reached the
// forward position clamp and struck an object before altitude could be judged.
// Give pitch and roll the same actuator leverage already validated for yaw so
// commanded attitude is realized sooner. Controller limits, motor bounds, and
// safety cutoffs remain unchanged.
const ATTITUDE_MIX_FACTOR: f32 = 0.5;
// Run204 gave the bounded heading controller enough actuator leverage to limit
// physical rotation without consuming the pitch/roll differentials. Restore
// the yaw authority used by the Run206 baseline.
const YAW_MIX_FACTOR: f32 = 0.75;
const MIN_MOTOR_DUTY: f32 = 0.0;
