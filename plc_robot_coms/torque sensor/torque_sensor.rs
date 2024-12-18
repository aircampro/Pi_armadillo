//
// This rust code reads a chosen torque sensor the driver ref is below
//
// https://github.com/Amelia10007/dynpick-force-torque-sensor-rs/tree/master
//
// the product is shown here
// https://wacoh-tech.com/en/products/dynpick/
//
// you can also chose a Leptrino device as well
//
//
// Cargo.toml
// [package]
// name = "dynpick-force-torque-sensor"
// version = "0.1.0"
// authors = ["Amelia10007 <nat.horn.mk0426@gmail.com>"]
// edition = "2018"
// description = "Device driver for Wacoh-tech force-torque sensor written in pure Rust."
// repository = "https://github.com/Amelia10007/dynpick-force-torque-sensor-rs"
// keywords = ["driver", "sensor", "force", "dynpick", "wacohtech"]
// categories = ["hardware-support"]
// license = "MIT"
// readme = "README.md"

// # See more keys and their definitions at https://doc.rust-lang.org/cargo/reference/manifest.html

// [dependencies]
// easy-ext = "0.2.6"
// itertools = "0.10.0"
// pair_macro = "0.1.4"
// serialport = "4.0.0"
// clap = { version = "4", features = ["derive"] }

//[badges]
//travis-ci = { repository = "Amelia10007/dynpick-force-torque-sensor-rs", branch = "master" }

// ---------------------------------------- main Rust Code --------------------------------------------

use dynpick_force_torque_sensor::serialport;
use dynpick_force_torque_sensor::{DynpickSensorBuilder, Sensitivity, Triplet};
//use leptrino_force_torque_sensor::serialport;
use leptrino_force_torque_sensor::{LeptrinoSensor, Product};

extern crate clap;
use clap::{App, AppSettings, Arg};

// parse the command line arguments which choose the number of requests and the type of torque measurement device
fn build_app() -> clap::App<'static, 'static> {
  let program = std::env::args()
    .nth(0)
    .and_then(|s| {
      std::path::PathBuf::from(s)
        .file_stem()
        .map(|s| s.to_string_lossy().into_owned())
    })
    .unwrap();

  App::new(program)
    .about("read torque sensor")
    .version("0.0.1")
    .author("Author name <aircampro455@gmail.com>")
    .setting(AppSettings::VersionlessSubcommands)
    .arg(Arg::from_usage("-t --type=[WACOH] 'Manufacturer type of torque sensor device wacoh or leptrino'"))
    .arg(Arg::from_usage("-m --meas_cnt=1 'measurement count required'"))
    // ...
}

// look for the requested vendor and product device on the usb and return the port its connected to
fn search_usb_sensor_path(vendor: &str) -> Result<Option<String>, serialport::Error> {

    let vendor_id = 0;
    let product_id = 0;
	
    if (vendor.eq("wacoh")) {
        let vendor_id = 0x10c4;                              // Wacoh-tech vendor ID.
        let product_id = 0xea60;                             // NOTE: The following product-ID may be specific for WDF-6M200-3.
    else if (vendor.eq("leptrino")) {
        let vendor_id = 0x0483;                              // Leptrino vendor ID.
        let product_id = 0x5740;                             // NOTE: The following product-ID may be specific for PFS055YA251U6.
    }	
    let ports = serialport::available_ports()?;
    let path = ports
        .into_iter()
        .filter(move |port| match &port.port_type {
            // Takes only USB-connected device
            serialport::SerialPortType::UsbPort(usb) => {
                usb.vid == vendor_id && usb.pid == product_id
            }
            _ => false,
        })
        .map(|sensor_port| sensor_port.port_name)
        .next();

    Ok(path)
	
}

fn main() {

    let matches = build_app().get_matches();
    let type : Option<&str> = matches.value_of("type");
    let meas_cnt: Option<u32> = matches.value_of("meas_cnt");

    // let meas_cnt_s : Option<&str> = matches.value_of("meas_cnt");
    // let meas_cnt: u32 = meas_cnt_s.parse().unwrap();
  
    println!("torque-sensor demo started..............................");
    println!("Make sure that the sensor is connected to the computer.");
    println!("Make sure setting udev rule. See examples/setup_udev_rule.sh in detail.");

    // Search USB-connected for the sensor type specified in the command line arguments.
    let path = match search_usb_sensor_path(type) {
        Ok(Some(path)) => path,
        Ok(None) => {
            println!("No torque measurement sensor is connected.");
            return;
        }
        Err(e) => {
            println!("{}", e);
            return;
        }
    };
    println!("Found a sensor. Path: {}", path);

    // convert to lower case string and back to a pointer
    let type_s: String = type.to_lowercase();
    let type: &str = type_s.as_str();

    if type.eq("wacoh") {
	
        // Specify the sensitivity manually.
        let sensitivity = {
            let force = Triplet::new(24.9, 24.6, 24.5);
            let torque = Triplet::new(1664.7, 1639.7, 1638.0);
            Sensitivity::new(force, torque)
        };

        // ============= Connect the found sensor.Wacoh =================
        let sensor = DynpickSensorBuilder::open(path)
            .map(|b| b.set_sensitivity_manually(sensitivity))                // if you want built in change to    .set_sensitivity_by_builtin_data()
            .and_then(|b| b.build())
            .unwrap();
        let mut sensor = match sensor {
            Ok(s) => s,
            Err(e) => {
                println!("{}", e);
                return;
            }
        };
        println!("Successfully opened the sensor.");
	
        // Correct zero-point
        match sensor.zeroed_next() {
            Ok(_) => println!("Offset the sensor."),
            Err(e) => {
                println!("An error occurred during offset: {}", e);
                return;
            }
        }

        let wrench = sensor.update_wrench().unwrap();
        println!("Wacoh :: Force: {}, Torque: {}", wrench.force, wrench.torque);

        // Repeatedly receive wrenches from the sensor.
        let measurement_count = meas_cnt;
        for i in 0..measurement_count {
            std::thread::sleep(sensor.inner_port().timeout());

            match sensor.update() {
                Ok(w) => println!("[{}/{}] {:?}", i + 1, measurement_count, w),
                Err(e) => println!("[{}/{}] {}", i + 1, measurement_count, e),
            }
        }
	
        // Info
        println!("Product info: {:?}", sensor.receive_product_info());

        println!("dynpick-force-torque-sensor demo finished.");
		
    } else if type.eq("leptrino") {	
	
        // ============= Connect the found sensor. Leptrino type ================
        let product_kind = Product::Pfs055Ya251U6;
        let mut sensor = match LeptrinoSensor::open(product_kind, path) {
            Ok(sensor) => sensor,
            Err(e) => {
                println!("{}", e);
                return;
            }
        };
        println!("Successfully opened the sensor.");

        let wrench = sensor.update().unwrap();
        println!("Leptrino :: {:?}", wrench);

        // Repeatedly receive wrenches from the sensor.
        let measurement_count = meas_cnt;
        for i in 0..measurement_count {
            std::thread::sleep(sensor.inner_port().timeout());

            match sensor.update() {
                Ok(w) => println!("[{}/{}] {:?}", i + 1, measurement_count, w),
                Err(e) => println!("[{}/{}] {}", i + 1, measurement_count, e),
            }

            // Correct zero-point by using the first wrench.
            if i == 0 {
                sensor.zeroed();
            }
        }

        // Info
        println!("Product info: {:?}", sensor.receive_product_info());

        println!("leptrino-force-torque-sensor demo finished.");
    }
}