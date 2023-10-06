extern crate photon;
extern crate regex;
use regex::Regex;
use std::env;
	
fn usage() {
    println!("<progname> IN_FILENAME OUT_FILENAME R G B")
}

fn main() {

    let in_filename = match env::args().nth(1) {
        Some(in_filename) => in_filename,
        None => {
            usage();
            return;
        }
    };
    let out_filename = match env::args().nth(2) {
        Some(out_filename) => out_filename,
        None => {
            usage();
            return;
        }
    };
    let ns1 = match env::args().nth(3) {
        Some(ns1) => ns1,
        None => {
            usage();
            return;
        }
    };
    let r_: u32 = ns1.parse().unwrap();
	
    let ns2 = match env::args().nth(4) {
        Some(ns2) => ns2,
        None => {
            usage();
            return;
        }
    };
    let g_: u32 = ns2.parse().unwrap();
	
    let ns3 = match env::args().nth(5) {
        Some(ns3) => ns3,
        None => {
            usage();
            return;
        }
    };
    let b_: u32 = ns3.parse().unwrap();
	
    // Open the image. A PhotonImage is returned.
    //let img: PhotonImage = open_image("images/flowers.PNG");  filename
    let img: PhotonImage = photon::helpers::open_image(in_filename);  

    // For example to darken an image by 10% in the HSL colour space:
    // use photon::color_spaces::darken_hsl; 
    // photon_rs::colour_spaces::darken_hsl(&mut img, 0.1);
    use photon::effects;
    photon::conv::tint(img, r_, g_, b_);
	
	// save the image
    //photon::helpers::save_image(img, "new_image.PNG");
	photon::helpers::save_image(img, out_filename);
}