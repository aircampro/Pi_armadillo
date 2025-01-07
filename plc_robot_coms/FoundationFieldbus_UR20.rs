// driver from https://github.com/slowtec/ur20-modbus?ysclid=m5kzps1126935313846
// using Weidmüller UR20-FBC-MOD-TCP
// interface to foundation fieldbus using ur20
//
use std::error::Error;
use ur20_modbus::Coupler;

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
    let addr = "192.168.0.222:502".parse()?;
    let mut coupler = Coupler::connect(addr).await?;
    let id = coupler.id().await?;
    println!("Connected to {}", id);
    coupler.set_output(
        &ur20::Address {
            module: 4,
            channel: 2,
        },
        ur20::ChannelValue::Bit(true),
    )?;
    coupler.tick().await?;
    println!("mod 4 ch 2 set on");
    coupler.set_output(
        &ur20::Address {
            module: 4,
            channel: 1,
        },
        ur20::ChannelValue::Bit(true),
    )?;
    coupler.tick().await?;
    println!("mod 4 ch 1 set on");
    coupler.set_output(
        &ur20::Address {
            module: 3,
            channel: 1,
        },
        ur20::ChannelValue::Bit(true),
    )?;
    coupler.tick().await?;
    println!("mod 3 ch 1 set on");
    Ok(())
}