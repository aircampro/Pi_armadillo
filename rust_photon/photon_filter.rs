extern crate photon;
extern crate regex;
use regex::Regex;
use std::env;
	
fn usage() {
    println!("<progname> IN_FILENAME OUT_FILENAME NUMBER_OF_FILTER")
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
    let filter_choice: u16 = ns1.parse().unwrap();
	
    // Open the image. A PhotonImage is returned.
    //let img: PhotonImage = open_image("images/flowers.PNG");  filename
    let img: PhotonImage = photon::helpers::open_image(in_filename);  

    // For example to darken an image by 10% in the HSL colour space:
    // use photon::color_spaces::darken_hsl; 
    // photon_rs::colour_spaces::darken_hsl(&mut img, 0.1);
    use photon::filters;
    let j = match filter_choice {
 
        1 => "oceanic",
 
        2 => "islands",
		
		3 => "marine",
		
		4 => "seagreen",
		
		5 => "flagblue",
		
		6 => "liquid",
		
		7 => "diamante",
		
		8 => "radio",
		
		9 => "twenties",
		
		10 => "rosetint",
		
		11 => "mauve",
		
		12 => "bluechrome",
		
		13 => "vintage",
		
		14 => "perfume",
		
		15 => "serenity",
 
        _ => "oceanic",
 
    };
    photon::filters::filter(img, filter_choice);
	
	// save the image
    //photon::helpers::save_image(img, "new_image.PNG");
	photon::helpers::save_image(img, out_filename);
}