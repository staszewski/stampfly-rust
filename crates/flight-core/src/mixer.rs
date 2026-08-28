use crate::types::{MotorInput, MotorMix};

#[derive(Debug, PartialEq, Eq)]
pub enum MixerError {
    NonFiniteInput,
    NonFiniteOutput,
}

pub struct MotorMixer;

const MAX_MOTOR_DUTY: f32 = 220.0;
const ATTITUDE_MIX_FACTOR: f32 = 0.5;
const YAW_MIX_FACTOR: f32 = 1.0;
const MIN_MOTOR_DUTY: f32 = 0.0;

impl MotorMixer {
    pub fn mix(input: MotorInput) -> Result<MotorMix, MixerError> {
        if !input.thrust.is_finite()
            || !input.pitch.is_finite()
            || !input.roll.is_finite()
            || !input.yaw.is_finite()
        {
            return Err(MixerError::NonFiniteInput);
        }
        let mut scale = 1.0;
        let pitch = input.pitch * ATTITUDE_MIX_FACTOR;
        let roll = input.roll * ATTITUDE_MIX_FACTOR;
        let yaw = input.yaw * YAW_MIX_FACTOR;

        let front_left = input.thrust + roll + pitch - yaw;
        let front_right = input.thrust - roll + pitch + yaw;
        let rear_left = input.thrust + roll - pitch + yaw;
        let rear_right = input.thrust - roll - pitch - yaw;

        let raw_minimum = front_left.min(front_right).min(rear_left).min(rear_right);
        let raw_maximum = front_left.max(front_right).max(rear_left).max(rear_right);

        let requested_spread = raw_maximum - raw_minimum;
        let available_spread = MAX_MOTOR_DUTY - MIN_MOTOR_DUTY;

        if requested_spread > available_spread {
            scale = available_spread / requested_spread;
        }

        let adjusted_fl = input.thrust + (front_left - input.thrust) * scale;
        let adjusted_fr = input.thrust + (front_right - input.thrust) * scale;
        let adjusted_rl = input.thrust + (rear_left - input.thrust) * scale;
        let adjusted_rr = input.thrust + (rear_right - input.thrust) * scale;

        let minimum = adjusted_fl
            .min(adjusted_fr)
            .min(adjusted_rl)
            .min(adjusted_rr);
        let maximum = adjusted_fl
            .max(adjusted_fr)
            .max(adjusted_rl)
            .max(adjusted_rr);
        let minimum_shift = MIN_MOTOR_DUTY - minimum;
        let maximum_shift = MAX_MOTOR_DUTY - maximum;

        let collective_shift = if minimum_shift > 0.0 {
            minimum_shift
        } else if maximum_shift < 0.0 {
            maximum_shift
        } else {
            0.0
        };

        let front_left = adjusted_fl + collective_shift;
        let front_right = adjusted_fr + collective_shift;
        let rear_left = adjusted_rl + collective_shift;
        let rear_right = adjusted_rr + collective_shift;

        if !front_left.is_finite()
            || !front_right.is_finite()
            || !rear_left.is_finite()
            || !rear_right.is_finite()
        {
            return Err(MixerError::NonFiniteOutput);
        }

        Ok(MotorMix {
            front_left,
            front_right,
            rear_left,
            rear_right,
            collective_shift,
            pitch_mix_factor: ATTITUDE_MIX_FACTOR * scale,
            roll_mix_factor: ATTITUDE_MIX_FACTOR * scale,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn axis_allocation() {
        let test_cases = [
            (
                MotorInput {
                    thrust: 100.0,
                    pitch: 20.0,
                    roll: 0.0,
                    yaw: 0.0,
                },
                Ok(MotorMix {
                    front_left: 110.0,
                    front_right: 110.0,
                    rear_left: 90.0,
                    rear_right: 90.0,
                    collective_shift: 0.0,
                    pitch_mix_factor: ATTITUDE_MIX_FACTOR,
                    roll_mix_factor: ATTITUDE_MIX_FACTOR,
                }),
            ),
            (
                MotorInput {
                    thrust: 100.0,
                    pitch: 0.0,
                    roll: 10.0,
                    yaw: 0.0,
                },
                Ok(MotorMix {
                    front_left: 105.0,
                    front_right: 95.0,
                    rear_left: 105.0,
                    rear_right: 95.0,
                    collective_shift: 0.0,
                    pitch_mix_factor: ATTITUDE_MIX_FACTOR,
                    roll_mix_factor: ATTITUDE_MIX_FACTOR,
                }),
            ),
            (
                MotorInput {
                    thrust: 100.0,
                    pitch: 0.0,
                    roll: 0.0,
                    yaw: 10.0,
                },
                Ok(MotorMix {
                    front_left: 90.0,
                    front_right: 110.0,
                    rear_left: 110.0,
                    rear_right: 90.0,
                    collective_shift: 0.0,
                    pitch_mix_factor: ATTITUDE_MIX_FACTOR,
                    roll_mix_factor: ATTITUDE_MIX_FACTOR,
                }),
            ),
        ];
        for (input, output) in test_cases {
            let result = MotorMixer::mix(input);
            assert_eq!(result, output);
        }
    }

    #[test]
    fn desaturation() {
        let test_cases = [
            (
                MotorInput {
                    thrust: 5.0,
                    pitch: 20.0,
                    roll: 0.0,
                    yaw: 0.0,
                },
                Ok(MotorMix {
                    front_left: 20.0,
                    front_right: 20.0,
                    rear_left: 0.0,
                    rear_right: 0.0,
                    collective_shift: 5.0,
                    pitch_mix_factor: ATTITUDE_MIX_FACTOR,
                    roll_mix_factor: ATTITUDE_MIX_FACTOR,
                }),
            ),
            (
                MotorInput {
                    thrust: 215.0,
                    pitch: 20.0,
                    roll: 0.0,
                    yaw: 0.0,
                },
                Ok(MotorMix {
                    front_left: 220.0,
                    front_right: 220.0,
                    rear_left: 200.0,
                    rear_right: 200.0,
                    collective_shift: -5.0,
                    pitch_mix_factor: ATTITUDE_MIX_FACTOR,
                    roll_mix_factor: ATTITUDE_MIX_FACTOR,
                }),
            ),
        ];
        for (input, output) in test_cases {
            let result = MotorMixer::mix(input);
            assert_eq!(result, output);
        }
    }

    #[test]
    fn feasibility_invariant() {
        let input = MotorInput {
            thrust: 110.0,
            pitch: 440.0,
            roll: 0.0,
            yaw: 0.0,
        };
        let output = Ok(MotorMix {
            front_left: 220.0,
            front_right: 220.0,
            rear_left: 0.0,
            rear_right: 0.0,
            collective_shift: 0.0,
            pitch_mix_factor: 0.25,
            roll_mix_factor: 0.25,
        });

        let result = MotorMixer::mix(input);
        assert_eq!(result, output);
    }

    #[test]
    fn saturation_test() {
        let input = MotorInput {
            thrust: 110.0,
            pitch: 240.0,
            roll: 200.0,
            yaw: 80.0,
        };
        let output = Ok(MotorMix {
            front_left: 220.0,
            front_right: 200.0,
            rear_left: 180.0,
            rear_right: 0.0,
            collective_shift: 40.0,
            pitch_mix_factor: 0.25,
            roll_mix_factor: 0.25,
        });

        let result = MotorMixer::mix(input);
        assert_eq!(result, output);
    }

    #[test]
    fn rejects_non_finite_inputs_and_outputs() {
        let test_cases = [
            (
                MotorInput {
                    thrust: f32::NAN,
                    pitch: f32::INFINITY,
                    roll: f32::NEG_INFINITY,
                    yaw: f32::NAN,
                },
                Err(MixerError::NonFiniteInput),
            ),
            (
                MotorInput {
                    thrust: f32::MAX,
                    pitch: f32::MAX,
                    roll: 0.0,
                    yaw: 0.0,
                },
                Err(MixerError::NonFiniteOutput),
            ),
        ];
        for (input, output) in test_cases {
            let result = MotorMixer::mix(input);
            assert_eq!(result, output);
        }
    }
}
