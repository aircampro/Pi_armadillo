// example read bosch bme280 on i2c and publish the data using zenoh
// ref:- https://zenoh.io/docs/migration_0.5_to_0.6/migrationguide-rust-v0.5.x-v0.6.x/
//
use std::thread;
use std::time::Duration;
use linux_embedded_hal as hal;
use linux_embedded_hal::{Delay, I2cdev};
use bme280::i2c::BME280;
use futures::prelude::*;
// using 0.6 async
use zenoh::prelude::r#async::*;

#[async_std::main]
async fn main() {
    let i2c_bus = I2cdev::new("/dev/i2c-1").unwrap();
    let mut delay = Delay;
    let mut bme280 = BME280::new_primary(i2c_bus);
    bme280.init(&mut delay).unwrap();                                                        // initialise the bme280 on i2c

    loop {
        //  ========= read bm280 i2c	========
        let measurements = bme280.measure(&mut delay).unwrap();                              // read bme280
        println!("Relative Humidity = {} %", measurements.humidity);
        println!("Temperature = {} C", measurements.temperature);
        println!("Pressure = {} hPa", measurements.pressure/100.0);
        //  ========= publish to zenoh	========
        // v0.5 let session = zenoh::open(zenoh::Config::default()).await.unwrap();
        let session = zenoh::open(zenoh::Config::default()).res().await.unwrap();             // we are using version 0.6
        session.put("environment/humidity", measurements.humidity).res().await.unwrap();
        session.put("environment/temperature", measurements.temperature).res().await.unwrap();
        session.put("environment/pressure", measurements.pressure/100.0).res().await.unwrap();	
        session.close().res().await.unwrap();
		thread::sleep(Duration::from_millis(100));
	}
}