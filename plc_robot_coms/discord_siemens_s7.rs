// ======================================= main.rs ===============================================
//
// About :- Chatbot for a discord channel which drives outputs and control on a Siemens S7 PLC.
//          This channel is effectively a mobile robots server....
//
// Cargo.toml
//
//[dependencies]
//dotenv = "0.15.0"
//env_logger = "0.9.0"
//log = "0.4.14"
//poise = { git = "https://github.com/kangalioo/poise", rev = "9ebc7b5" }
//thiserror = "1.0.30"
//tokio = { version = "1.16.1", features = ["rt-multi-thread"] }
//s7 = "*"

#[macro_use]
extern crate log;

use std::env;

extern crate s7;

use s7::{client::Client, field::Bool, field::Fields, field::Float, tcp, transport::Connection};
use std::net::{IpAddr, Ipv4Addr};
use std::time::Duration;

#[derive(thiserror::Error, Debug)]
enum AppError {
    #[error("{0}")]
    Serenity(#[from] poise::serenity::Error),
}

type Context<'a> = poise::Context<'a, (), AppError>;

/// register the application commands
#[poise::command(prefix_command, hide_in_help)]
async fn register(ctx: Context<'_>, #[flag] global: bool) -> Result<(), AppError> {
    poise::builtins::register_application_commands(ctx, global).await?;
    Ok(())
}

/// toggle a contact on the s7 PLC which has been wired to the lights
#[poise::command(prefix_command, slash_command)]
async fn toggle_s7_lights(
    ctx: Context<'_>,
) -> Result<(), AppError> {
	let lv = toggle_s7_lights();
    poise::say_reply(ctx, format!("you toggled the light switch to ", lv)).await?;
    Ok(())
}

/// gets the temperature probe wired to the siemens PLC
#[poise::command(prefix_command, slash_command)]
async fn get_temperature_value(
    ctx: Context<'_>,
) -> Result<(), AppError> {
	let tv = get_temperature_value();
    poise::say_reply(ctx, format!("the temperature is ", tv)).await?;
    Ok(())
}

/// sets the value in the s7 plc which sets the speed of a pump
#[poise::command(prefix_command, slash_command)]
async fn set_pump_value(
    ctx: Context<'_>,
    #[description = "value to set in PLC"] val: f32,
) -> Result<(), AppError> {
	set_pump_value(val);
    poise::say_reply(ctx, format!("you set the pump value :- {}", val)).await?;
    Ok(())
}

async fn on_error(error: poise::FrameworkError<'_, (), AppError>) {
    error!("{:?}", error);
}

// remember DISCORD_TOKEN="your token"; export DISCORD_TOKEN
//
#[tokio::main]
async fn main() {
    dotenv::dotenv().ok();
    env_logger::init();
    let token = env::var("DISCORD_TOKEN").expect("DISCORD_TOKEN not set");
   
    let options = poise::FrameworkOptions {
        commands: vec![register(), toggle_s7_lights(), get_temperature_value(), set_pump_value(), ],
        prefix_options: poise::PrefixFrameworkOptions {
            prefix: Some("::".to_string()),
            ..Default::default()
        },
        on_error: |err| Box::pin(on_error(err)),
        ..Default::default()
    };

    poise::Framework::build()
        .token(token)
        .options(options)
        .user_data_setup(|_, _, _| Box::pin(async { Ok(()) }))
        .run()
        .await
        .unwrap();
}


// toggle the light switch which is connected to S7 relay DB 878 OFFSET 6.4
fn toggle_s7_lights() -> std::io::Result<String> {
    println!(" --- > Relay write");

    // set communication to the siemens S7 PLC
    let addr = Ipv4Addr::new(10, 0, 0, 1);
    let mut opts = tcp::Options::new(IpAddr::from(addr), 5, 5, Connection::PG);

    opts.read_timeout = Duration::from_secs(2);
    opts.write_timeout = Duration::from_secs(2);

    let t = tcp::Transport::connect(opts).unwrap();
    let mut cl = Client::new(t).unwrap();

    // set the db and the offset
    let db = 878;

    // the offset in the PLC is represented by a float
    // the difit on the left is the index within the block
    // the digit after the decimal point is only important for the `Bool` to be able to change the relevant bit
    // we don't need after
    let mut offset = 6.4;
		
    // Since this is a boolean field, we are going to get back 1 byte
    cl.ag_read(db, offset as i32, Bool::size(), buffer).unwrap();

    // field mod provides types to handle the data from the PLC
    // create a bool field from the byte we got
    let mut lights = Bool::new(db, offset, buffer.to_vec()).unwrap();

    // the bit in the byte is set without changing any of the other bits
    lights.set_value(!lights.value()); // toggle the light switch

    let fields: Fields = vec![Box::new(lights)];

    // save back the changed values
    for field in fields.iter() {
       cl.ag_write(
           field.data_block(),
           field.offset(),
           field.to_bytes().len() as i32,
           field.to_bytes().as_mut(),
       )
       .unwrap();
    }
    //
    Ok(format!("{ value = {} }", lights.value()).to_string())	
}

// write a value for cooling to the s7 plc db 768 offset 10.0
fn set_pump_value(val: f32) {
    println!(" --- > Value write");

    // set communication to the siemens S7 PLC
    let addr = Ipv4Addr::new(10, 0, 0, 1);
    let mut opts = tcp::Options::new(IpAddr::from(addr), 5, 5, Connection::PG);

    opts.read_timeout = Duration::from_secs(2);
    opts.write_timeout = Duration::from_secs(2);

    let t = tcp::Transport::connect(opts).unwrap();
    let mut cl = Client::new(t).unwrap();
	
    let db = 768;
    let offset = 10.0;
	
    let mut pump_buffer = vec![0u8; Float::size() as usize];
    cl.ag_read(db, offset as i32, Float::size(), pump_buffer.as_mut())
        .unwrap();
    let mut pump = Float::new(db, offset, pump_buffer).unwrap();
    pump.set_value(val);

    let fields: Fields = vec![Box::new(pump)];

    // save back the changed values
    for field in fields.iter() {
        cl.ag_write(
           field.data_block(),
           field.offset(),
           field.to_bytes().len() as i32,
           field.to_bytes().as_mut(),
        )
        .unwrap();
    }
    //
    println!(" ---  > Value passed was / {:?}", val)
}

// get an analog value DB 678 OFFSET 1,1
fn get_temperature_value() -> std::io::Result<String> {
    println!(" --- > Value read");

    // set communication to the siemens S7 PLC
    let addr = Ipv4Addr::new(10, 0, 0, 1);
    let mut opts = tcp::Options::new(IpAddr::from(addr), 5, 5, Connection::PG);

    opts.read_timeout = Duration::from_secs(2);
    opts.write_timeout = Duration::from_secs(2);

    let t = tcp::Transport::connect(opts).unwrap();
    let mut cl = Client::new(t).unwrap();
	
    let db = 678;
    let offset = 1.1;
	
    let mut cooling_buffer = vec![0u8; Float::size() as usize];
    cl.ag_read(db, offset as i32, Float::size(), cooling_buffer.as_mut())
        .unwrap();
    let mut temper = Float::new(db, offset, cooling_buffer).unwrap();
    let valu = temper.value();

    //
    println!(" ---  > Value read was / {:?}", valu)
    Ok(format!("{ value = {} }", temper.value()).to_string())	
}

