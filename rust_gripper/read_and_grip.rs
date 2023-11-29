// ================================================== HTTP Client =====================================================
//                                                      main.rs
//       Read label text from photo of label and ask server for object width then control gripper to pick that object
//
// ---- cargo.toml ---
//
// [dependencies]
// reqwest = { version = "0.11.9", features = ["json", "blocking"] }
// text-colorizer = "1"
// regex = "1"
// proconio = "0.3.6"
// tokio = { version = "1", features = ["full"] }
// serde = { version = "1.0", features = ["derive"] }
// serde_json = "1.0"
// ansi_term = "0.12.1"
// configparser = "3.0.0"
// async-std = { version = "1.7", features = ["attributes", "tokio1", "unstable"] }
// rusty-tesseract = "1.1.4"
//
// =====================================================================================================================
use std::time::Instant;
use async_std::task;
use text_colorizer::*;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

use std::fs;
use std::path::Path;
use rusty_tesseract::{LepTess, Image};

use std::process::Command;

// JSON sends position to get object name from server { "lat" : "23.4", "lon" : "0.032", "userId" : 1 } 
// receives back json { "obj_name" : "unknown", "width" : 0 } or { "obj_name" : "small_box_3", "width" : 23.2 }
//
async fn post_json_pos_request(url: &str, v1: &str, v2: &str) -> std::io::Result<serde_json::Value> {
    let echo_json: serde_json::Value = reqwest::Client::new()
        .post(url)
        .json(&serde_json::json!({
            "lat": v1,
            "lon": v2,
            "userId": 1
        }))
        .send()
        .await?
        .json()
        .await?;

    println!("{:#?}", echo_json);
    Ok(echo_json)
}
// JSON sends name to get width from server { "obj_name" : "large_log_6", "userId" : 1 }
// replies with a single value equivalent to the objects width or -1
//
async fn post_json_obj_request(url: &str, v1: &str) -> std::io::Result<f64; N> {
    let ret_width: String = reqwest::Client::new()
        .post(url)
        .json(&serde_json::json!({
            "obj_name": v1,
            "userId": 1
        }))
        .send()
        .await?
        .text()
        .await?;

    //let stri: String = String::from(ret_width); use if its not a text
    //let num: f64 = stri.parse().unwrap();
	let num: f64 = ret_width.parse().unwrap();
    println!("width is {}", num);
    Ok(num)
}
	
// The [cfg(not(target_arch = "wasm32"))] above prevent building the tokio::main function
// for wasm32 target, because tokio isn't compatible with wasm32.
// If you aren't building for wasm32, you don't need that line.
// The two lines below avoid the "'main' function not found" error when building for wasm32 target.
//
#[cfg(not(target_arch = "wasm32"))]
#[tokio::main]
fn main() -> Result<(), Box<std::error::Error>>) {

    // Read the picture for the label using tesseract and request the object width from the server
	// then use that width to grip it.
	//
	
    // Create an Image object from the image file which is a picture taken of the label
	// we store the image in src/img.jpeg before invoking this program
	//
    let img = Image::from_path(Path::new("src/img.jpeg")).unwrap();
    //let img = Image::from_path(Path::new(&fs_path)).unwrap();
			
    // Create a new LepTess instance with the English language
	//
    let mut lt = LepTess::new(None, "eng").unwrap();

    // Set the image to be used for OCR
	//
    lt.set_image_from_image(&img);

    // Get the recognized text
	//
    let text_label = lt.get_utf8_text().unwrap();
			
    // build a vector of the urls to use to get the width we do it 3 times here to the same sarver but they could be different ones 
	// in various potential areas that the robot can go or different paths on the same server meaning different tables e.g. building floors etc
	// or 2 servers for dual redundancy i.e. 2 different ip addresses
	//
    let requests = vec![
        "http://localhost:8080/getwidth".to_string(),
		"http://localhost:8080/getwidth2".to_string(),
        "http://localhost:8080/getwidth".to_string(),
		"http://localhost:8080/getwidth2".to_string(),
        "http://localhost:8080/getwidth".to_string(),
		"http://localhost:8080/getwidth2".to_string()
    ];

    // call get reqwest function up to 3 times to get the gripper width from the server if response is ok leave the loop
	// change the localhost address to the potential servers you will encounter and the number of retries
	//
    let start1 = Instant::now();
    for request in &requests {
        let result = post_json_obj_request(request, &text_label );
        match result {
            Ok(response) => {
                println!("{}", response.status());
	        // if result >= 0 exit the for loop !!!!!!!!!!!!!!!!!!!!!!!!!!!
		if result >= 0 {
	            break;
		}
            },
            Err(err) => {
                println!("NG");
                eprintln!("Err {}", err);
            },
        }
    }
    let end1 = start1.elapsed();
	
    // print the speed of response in case we have issues with latency
	//
    println!("{}: {}.{:03} [sec]", "Process time".blue().bold(), end1.as_secs(), end1.subsec_nanos() / 1_000_000);

    // ====== alert via speech and wait for a reply ? =========

    // now grip the object
    //	
	if ( result >= 0 ) {                                   // else it didnt find that object
	    let width_from_server: [f64; N] = result;
	
	    // call the rust gripper controller program with the given width try 3 times as its in rust it might be just as easy to add it here
	    // this is for flexible example purpose
	    //
        let output = Command::new("/bin/cd")
        .args(&["/home/robots/franka/rust/src"])
        .spawn()
        .expect("failed to start `ls`");	
        println!("status: {}", output.status);
        println!("stdout: {}", String::from_utf8_lossy(&output.stdout));
        println!("stderr: {}", String::from_utf8_lossy(&output.stderr));
        let output2 = Command::new("cargo")
        .args(&["run", "--gripper_hostname", "my_gripper1", "--homing", 1, "--object_width", width_from_server])
        .spawn()
        .expect("failed to start `ls`");	
        println!("status: {}", output2.status);
        println!("stdout: {}", String::from_utf8_lossy(&output2.stdout));
        println!("stderr: {}", String::from_utf8_lossy(&output2.stderr));
    } else {
	    println!("{} {}",".....object label read not found in database is it a ".red().bold(),text_label);
	}
}
