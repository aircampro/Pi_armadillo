//
// send video frames to pixelflut projector https://github.com/defnull/pixelflut?ysclid=mabg06qmp1727976195
//
use image::io::Reader as ImageReader;
use image::GenericImageView;
use std::io::prelude::*;
use std::net::TcpStream;
use std::time::Instant;
use tracing::Level;
use tracing_subscriber;
use rscam::{Camera, Config};

fn main() -> std::io::Result<()> {
    tracing_subscriber::fmt()
        // filter spans/events with level TRACE or higher.
        .with_max_level(Level::TRACE)
        .init();
    tracing::info!("smap");

    // v4l2 wrapper rate rscam to open the video device driver
    let mut camera = rscam::new("/dev/video0").unwrap();

    // start the video camera
    camera.start(&rscam::Config {
        interval: (1, 30),      // 30 fps.
        resolution: (1280, 720),
        format: b"MJPG",
        ..Default::default()
    }).unwrap();

    // read 2000 frames from the camera and send each pixel to the pixelflut server
    for i in 0..2000 {
        let img = camera.capture().unwrap();
        let img_width = img.resolution.0;
        tracing::info!("width: {img_width}");
        let img_height = img.resolution.1;
        tracing::info!("height: {img_height}");

        // to write files if you want them (will be slow)
	//let mut file = fs::File::create(&format!("frame-{}.jpg", i)).unwrap();
        //let mut file = fs::File::create("frame.jpg").unwrap();
        //file.write_all(&img[..]).unwrap();	
        //let img = ImageReader::open("frame.jpg")?.decode().unwrap();
        //let img_width = img.width();
        //tracing::info!("width: {img_width}");
        //let img_height = img.height();
        //tracing::info!("height: {img_height}");

        let mut pn_pic = String::new();
        for y in 0..img_height {
            for x in 0..img_width {
                let imgpx = img.get_pixel(x, y);

                let r = imgpx.0[0];
                let g = imgpx.0[1];
                let b = imgpx.0[2];

                // pixelflut command as described PX <x> <y> <rrggbb(aa)>: Draw a single pixel at position (x, y) 
		// with the specified hex color code. If the color code contains an alpha channel value, 
		// it is blended with the current color of the pixel.
				
                // write out each pixel to the projector
                let pnpx = format!("PX {x} {y} {:x?}{:x?}{:x?}\n", r, g, b);
                //let pnpx = format!("PX {x} {y} {:x?}{:x?}{:x?}\n", b, g, r);
                pn_pic.push_str(pnpx.as_str());
            }
        }

        //let cmd = "SIZE\n";
        //let cmd = "PX 23 42 00ff00\nPX 15 45 00ff00\n";
        let buffer = pn_pic.as_bytes();
        let start = Instant::now();
        let mut stream = TcpStream::connect("127.0.0.1:1337")?;
        stream.write(buffer)?;
        let duration = start.elapsed().as_millis();
        let bsize = buffer.len();
        tracing::info!("send {bsize} bytes in {duration} ms",);
    }
    Ok(())
}




