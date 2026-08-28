use crate::types::BodyAccelerationG;

const GRAVITY_MM_PER_SECOND_SQUARED: f32 = 9_806.65;

#[derive(Debug, PartialEq)]
pub enum EarthVerticalAccelerationError {
    NonFiniteInput,
    NonFiniteOutput,
}

pub fn earth_vertical_acceleration_mm_per_second_squared(
    pitch_deg: f32,
    roll_deg: f32,
    body_accel: BodyAccelerationG,
) -> Result<f32, EarthVerticalAccelerationError> {
    if !pitch_deg.is_finite()
        || !roll_deg.is_finite()
        || !body_accel.x_g.is_finite()
        || !body_accel.y_g.is_finite()
        || !body_accel.z_g.is_finite()
    {
        return Err(EarthVerticalAccelerationError::NonFiniteInput);
    }
    let pitch_rad = pitch_deg.to_radians();
    let roll_rad = roll_deg.to_radians();
    let projection = -roll_rad.sin() * body_accel.x_g
        + pitch_rad.sin() * roll_rad.cos() * body_accel.y_g
        + pitch_rad.cos() * roll_rad.cos() * body_accel.z_g;
    let candidate = (projection - 1.0) * GRAVITY_MM_PER_SECOND_SQUARED;
    if !candidate.is_finite() {
        return Err(EarthVerticalAccelerationError::NonFiniteOutput);
    }
    return Ok(candidate);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn flat_and_stationary_produces_zero() {
        let result = earth_vertical_acceleration_mm_per_second_squared(
            0.0,
            0.0,
            BodyAccelerationG {
                x_g: 0.0,
                y_g: 0.0,
                z_g: 1.0,
            },
        )
        .expect("shoul be valid");
        assert_eq!(result, 0.0);
    }

    #[test]
    fn stationary_at_know_tilt_produces_approx_zero() {
        let result = earth_vertical_acceleration_mm_per_second_squared(
            30.0,
            0.0,
            BodyAccelerationG {
                x_g: 0.0,
                y_g: 0.5,
                z_g: 0.8660254,
            },
        )
        .expect("should be valid");
        assert_eq!(result, 0.0);
    }

    #[test]
    fn upward_and_downward_accel_have_equal_opposite_outputs() {
        let upward_accel = BodyAccelerationG {
            x_g: 0.0,
            y_g: 0.0,
            z_g: 1.2,
        };

        let downward_accel = BodyAccelerationG {
            x_g: 0.0,
            y_g: 0.0,
            z_g: 0.8,
        };

        let pitch_deg = 0.0;
        let roll_deg = 0.0;

        const TOLERANCE_MM_PER_SECOND_SQUARED: f32 = 0.01;

        let upward_result =
            earth_vertical_acceleration_mm_per_second_squared(pitch_deg, roll_deg, upward_accel)
                .expect("should be valid");
        let downward_result =
            earth_vertical_acceleration_mm_per_second_squared(pitch_deg, roll_deg, downward_accel)
                .expect("should be valid");

        let expected_upward = 1961.33;
        let expected_downward = -1961.33;

        let upward_difference = (upward_result - expected_upward).abs();
        assert!(
            upward_difference <= TOLERANCE_MM_PER_SECOND_SQUARED,
            "upward acceleration mismatch: actual={upward_result}, \
       expected={expected_upward}, difference={upward_difference}"
        );

        let downward_difference = (downward_result - expected_downward).abs();
        assert!(
            downward_difference <= TOLERANCE_MM_PER_SECOND_SQUARED,
            "downward acceleration mismatch: actual={downward_result}, \
       expected={expected_downward}, difference={downward_difference}"
        );
    }
}
