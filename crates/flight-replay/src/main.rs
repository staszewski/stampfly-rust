mod log_reader;

use std::env;

fn main() {
    println!(
        "Flight Replay: setup ready. Next step: read one log record and print tick, pitch, roll and yaw."
    );

    let args: Vec<String> = env::args().collect();
    let run_path = &args[1];
    let _samples = match log_reader::read_samples(run_path) {
        Ok(samples) => samples,
        Err(error) => {
            eprintln!("{error}");
            return;
        }
    };
}
