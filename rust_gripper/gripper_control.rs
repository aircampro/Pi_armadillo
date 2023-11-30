// https://github.com/marcbone/libfranka-rs/tree/master/examples
// Library to control Franka Emika robots
//
// $ cargo add clap --features derive to add clap to the Cargo.toml
// returns -1 if the grip failed 
//

// Copyright (c) 2021 Marco Boneberger
// Licensed under the EUPL-1.2-or-later
//
// $ cargo run -- --help
// prints help line
//
// Usage: cargo run --gripper_hostname <NAME> --homing 0 --object_width 2.34
//
use clap::Parser;
use franka::{FrankaResult, Gripper};
use std::time::Duration;

/// Program to control FRANKA's gripper. e.g. gripper_control host do_homing object_width 
#[derive(Parser, Debug)]
#[clap(author, version, name = "grasp_object")]
struct CommandLineArguments {
    /// IP-Address or hostname of the gripper
    pub gripper_hostname: String,
    /// Perform homing before grasping to calibrate the gripper
    #[clap(long)]
    pub homing: bool,
    /// Width of the object in meter
    pub object_width: f64,
}
impl CommandLineArguments {
    /// Create a new CommandLineArguments.
    fn new(gripper_hostname: &str, homing: bool, object_width: Option<f64>) -> CommandLineArguments {
        CommandLineArguments { gripper_hostname: gripper_hostname.to_string(), homing: homing, object_width: object_width }
    }
    fn get_name(&self) -> &str {
        &self.gripper_hostname
    }
    fn get_homing(&self) -> bool {
        &self.homing.unwrap_or(0) as bool
    }
    fn get_width(&self) -> f64 {
        self.object_width.unwrap_or(0) as f64
    }
}

fn main() -> FrankaResult<()> {
    let args: CommandLineArguments = CommandLineArguments::parse();
    let mut gripper = Gripper::new(args.gripper_hostname.as_str())?;
    if args.homing {
        gripper.homing()?;
    }
    let state = gripper.read_once()?;
    if state.max_width < args.object_width {
        eprintln!("Object is too large for the current fingers on the gripper.");
        std::process::exit(-1);
    }
    gripper.grasp(args.object_width, 0.1, 60., None, None)?;
    std::thread::sleep(Duration::from_secs(3));
    let state = gripper.read_once()?;
    if !state.is_grasped {
        eprintln!("Object lost");
        println!("Object lost - unable to grip !");
        std::process::exit(-1);
    }
    println!("Grasped object, will release it now.");
    gripper.stop()?;
    Ok(())
}
