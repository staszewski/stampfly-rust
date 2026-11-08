use std::error::Error;
use std::fs;

const REQUIRED_COLUMNS: [&str; 4] = ["tick", "pitch_deg", "roll_deg", "yaw_heading_deg"];

#[derive(Debug)]
pub(crate) struct FlightSample {
    pub(crate) tick: u64,
    pub(crate) pitch_deg: f64,
    pub(crate) roll_deg: f64,
    pub(crate) yaw_heading_deg: f64,
}

pub(crate) fn read_samples(run_path: &str) -> Result<Vec<FlightSample>, Box<dyn Error>> {
    let run_content = fs::read_to_string(run_path)?;

    // Verify headers
    let mut line_iterator = run_content.lines();
    let headers = match line_iterator.next() {
        Some(headers) => headers,
        None => return Ok(Vec::new()),
    };

    let split_headers: Vec<&str> = headers.split(';').collect();
    let mut required_columns_positions: Vec<usize> = Vec::new();

    for required in REQUIRED_COLUMNS {
        let position = split_headers.iter().position(|header| *header == required);

        if let Some(position) = position {
            required_columns_positions.push(position);
        } else {
            panic!("We looked for a column, but there was no match. WE PANIC")
        }
    }

    // Do work on the rest of the file
    let mut samples: Vec<FlightSample> = Vec::new();
    for line in line_iterator {
        let split_line: Vec<&str> = line.split(';').collect();

        let tick = match split_line[required_columns_positions[0]].parse::<u64>() {
            Ok(tick) => tick,
            Err(error) => {
                return Err(format!(
                    "Something went wrong with getting a tick, we got error: {}",
                    error
                )
                .into());
            }
        };

        let pitch_deg = match split_line[required_columns_positions[1]].parse::<f64>() {
            Ok(pitch_deg) => pitch_deg,
            Err(error) => {
                return Err(format!(
                    "Something went wrong with getting a pitch_deg, we got error: {}",
                    error
                )
                .into());
            }
        };

        let roll_deg = match split_line[required_columns_positions[2]].parse::<f64>() {
            Ok(roll_deg) => roll_deg,
            Err(error) => {
                return Err(format!(
                    "Something went wrong with getting a roll_deg, we got error: {}",
                    error
                )
                .into());
            }
        };

        let yaw_heading_deg = match split_line[required_columns_positions[3]].parse::<f64>() {
            Ok(roll_deg) => roll_deg,
            Err(error) => {
                return Err(format!(
                    "Something went wrong with getting a yaw_heading_deg, we got error: {}",
                    error
                )
                .into());
            }
        };

        let sample = FlightSample {
            tick,
            pitch_deg,
            roll_deg,
            yaw_heading_deg,
        };

        samples.push(sample);
    }

    Ok(samples)
}
