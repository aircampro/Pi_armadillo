extern crate photon;
extern crate regex;
use regex::Regex;
use std::env;
	
fn usage() {
    println!("photon_hsv_dark EFFECT_AMMOUNT IN_FILENAME OUT_FILENAME")
}

fn main() {

    let in_filename = match env::args().nth(2) {
        Some(in_filename) => in_filename,
        None => {
            usage();
            return;
        }
    };
    let out_filename = match env::args().nth(3) {
        Some(out_filename) => out_filename,
        None => {
            usage();
            return;
        }
    };
    let effect_val = match env::args().nth(1) {
        Some(effect_val) => effect_val,
        None => {
            usage();
            return;
        }
    };
	let f = effect_val.parse::<f32>().unwrap();
	
    // Open the image. A PhotonImage is returned.
    //let img: PhotonImage = open_image("images/flowers.PNG");  filename
    let img: PhotonImage = photon::helpers::open_image(in_filename);  

    photon::colour_spaces::lighten_hsl(&mut img, f);
	
	// save the image
    //photon::helpers::save_image(img, "new_image.PNG");
	photon::helpers::save_image(img, out_filename);
}