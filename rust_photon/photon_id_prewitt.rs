extern crate photon;
extern crate regex;
use regex::Regex;
use std::env;
	
fn usage() {
    println!("<progname> IN_FILENAME OUT_FILENAME")
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
	
    // Open the image. A PhotonImage is returned.
    //let img: PhotonImage = open_image("images/flowers.PNG");  filename
    let img: PhotonImage = photon::helpers::open_image(in_filename);  

    // For example to darken an image by 10% in the HSL colour space:
    // use photon::color_spaces::darken_hsl; 
    // photon_rs::colour_spaces::darken_hsl(&mut img, 0.1);
    use photon::conv;
    photon::conv::prewitt_horizontal(img);
	
	// save the image
    //photon::helpers::save_image(img, "new_image.PNG");
	photon::helpers::save_image(img, out_filename);
}