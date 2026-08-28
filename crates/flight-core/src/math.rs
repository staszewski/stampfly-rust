pub fn wrap_degrees(angle_deg: f32) -> f32 {
    let wrapped = (angle_deg + 180.0).rem_euclid(360.0) - 180.0;
    if wrapped == -180.0 { 180.0 } else { wrapped }
}

pub fn decay_for_sample_dt(sample_dt_seconds: f32, time_constant_seconds: f32) -> f32 {
    (-sample_dt_seconds / time_constant_seconds).exp()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wrap_degrees_works() {
        let test_cases = [
            (0.0, 0.0),
            (90.0, 90.0),
            (-90.0, -90.0),
            (181.0, -179.0),
            (360.0, 0.0),
            (-540.0, 180.0),
        ];
        for (input, output) in test_cases {
            let result = wrap_degrees(input);
            assert_eq!(result, output);
        }
    }
}
