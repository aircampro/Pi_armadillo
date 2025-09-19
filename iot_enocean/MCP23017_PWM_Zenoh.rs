// control MCP23017 i/o expander connected on i2c from values read using zenoh protocol
// control motor speed from value read from server using zenoh protocol
// saves data to to a Microchip 25AA1024 serial EEPROM using SPI.
//
use std::thread;
use std::time::Duration;
use futures::prelude::*;
// using 0.6 async
use zenoh::prelude::r#async::*;
use rppal::i2c::I2c;
use std::{thread, time};
use rppal::pwm::{Channel, Polarity, Pwm};
use std::error::Error;
use rppal::spi::{Bus, Mode, Segment, SlaveSelect, Spi};

// Instruction set.
const WRITE: u8 = 0b0010;             // Write data, starting at the selected address.
const READ: u8 = 0b0011;              // Read data, starting at the selected address.
const RDSR: u8 = 0b0101;              // Read the STATUS register.
const WREN: u8 = 0b0110;              // Set the write enable latch (enable write operations).

const WIP: u8 = 1;                    // Write-In-Process bit mask for the STATUS register.

// I2C
const ADDR: u16 = 0x20;
// direction
const REG_CTRL_IODIRA: u8 = 0x00;
// gpio control PORTA
const REG_CTRL_GPIOA: u8 = 0x12;

// toggle the expander output i/o LED's
fn setup_test_led() -> Result<u8, rppal::i2c::Error> {

    //I2C
    let mut i2c = I2c::new()?;
    i2c.set_timeout(100)?;
    i2c.set_slave_address(ADDR)?;

    // MCP23017 PORTA direction output
    i2c.smbus_write_word(REG_CTRL_IODIRA, 0x00)?;

    // wait 100ms
    let ten_millis = time::Duration::from_millis(100);

    // toggle output to test
    for _n in 1..10 {
        // 0xAA PORTA 0,2,4,8 chLOW , 1,3,5,7 HIGH
        i2c.smbus_write_word(REG_CTRL_GPIOA, 0xaa)?;
        // wait a little (10ms)
        thread::sleep(ten_millis);

        // 0x55 PORTA0,2,4,8ch HIGH, 1,3,5,7 LOW
        i2c.smbus_write_word(REG_CTRL_GPIOA, 0x55)?;
        // wait a little (10ms)
        thread::sleep(ten_millis);
    }

    // set all channels PORTA 0～7ch LOW
    i2c.smbus_write_word(REG_CTRL_GPIOA, 0x00)?;

    Ok(0)
}

fn main() {
    println!(" --- I2C Expander Test (Output) ---");
    let result = setup_test_led();
    match result {
        Ok(v) => println!(" Done : {}", v),
        Err(e) => println!("　Error : {}", e),
    };
    println!(" --- PWM Motor Test ---");
	// Enable PWM channel 0 (BCM GPIO 12, physical pin 32) at 2 Hz with a 25% duty cycle.
    let pwm = Pwm::with_frequency(Channel::Pwm0, 2.0, 0.25, Polarity::Normal, true)?;
	
    // set-up connection over i2c with the expander io
	let mut i2c = I2c::new()?;
    i2c.set_timeout(100)?;
    i2c.set_slave_address(ADDR)?;
    // set MCP23017 PORTA direction output
    i2c.smbus_write_word(REG_CTRL_IODIRA, 0x00)?;
    thread::sleep(Duration::from_millis(10));
    // connect to zenoh and write the outputs needed
    let session = zenoh::open(zenoh::Config::default()).res().await.unwrap();             // we are using version 0.6
    let portAiomap: i16 = session.get("drone/op_portA").res().await.unwrap() & 0xFF;      // get the portA i.on
    let mtApwm: f32 = session.get("drone/mtrApwmF").res().await.unwrap();                      // mtr A pwm frequency (speed)
    let mtAds: f32 = session.get("drone/mtrApwmDS").res().await.unwrap();                      // mtr A pwm frequency (speed)
    session.close().res().await.unwrap();
	
    i2c.smbus_write_word(REG_CTRL_GPIOA, portAiomap)?;                                    // set expander portA outputs
    pwm.set_frequency(mtApwm, mtAds)?;                                                    // set motor
    // Configure the SPI peripheral. The 24AA1024 clocks in data on the first
    // rising edge of the clock signal (SPI mode 0). At 3.3 V, clock speeds of up
    // to 10 MHz are supported.
    let mut spi = Spi::new(Bus::Spi0, SlaveSelect::Ss0, 8_000_000, Mode::Mode0)?;

    // Set the write enable latch using the WREN instruction. This is required
    // before any data can be written. The write enable latch is automatically
    // reset after a WRITE instruction is successfully executed.
    spi.write(&[WREN])?;

    // Use the WRITE instruction to select memory address 0 and write 3 values
    // (portAiomap, mtApwm, mtAds). Addresses are specified as 24-bit values, but the 7 most
    // significant bits are ignored.
    spi.write(&[WRITE, 0, 0, 0, portAiomap, mtApwm, mtAds])?;
    // Read the STATUS register by writing the RDSR instruction, and then reading
    // a single byte. Loop until the WIP bit is set to 0, indicating the write
    // operation is completed. transfer_segments() will keep the Slave Select line
    // active until both segments have been transferred.
    let mut buffer = [0u8; 1];
    loop {
        spi.transfer_segments(&[
            Segment::with_write(&[RDSR]),
            Segment::with_read(&mut buffer),
        ])?;

        if buffer[0] & WIP == 0 {
            break;
        }
    }
    // Use the READ instruction to select memory address 0, specified as a 24-bit
    // value, and then read 10 bytes. i16 f32 f32
    let mut buffer = [0u8; 10];
    spi.transfer_segments(&[
        Segment::with_write(&[READ, 0, 0, 0]),
        Segment::with_read(&mut buffer),
    ])?;
    println!("Bytes read: {:?}", buffer);
    Ok(())


}
