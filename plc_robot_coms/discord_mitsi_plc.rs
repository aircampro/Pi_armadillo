// =================================================================== main.rs ===========================================================================
//
// About :- Chatbot for a discord channel which drives outputs and control on a Mitsubishi FX5U sequencer.
//          This channel is effectively a mobile robots server.... and can command a robot or machine via discord
//          in this example the protocol is modbus tcp and the actions would be determined in the sequencer.
//          if you add the neccessary crate and build that command as these examples show, that might be the radius of a abb robot, you pass from discord.
//
// Cargo.toml
//
//[dependencies]
// dotenv = "0.15.0"
// env_logger = "0.9.0"
// log = "0.4.14"
// poise = { git = "https://github.com/kangalioo/poise", rev = "9ebc7b5" }
// thiserror = "1.0.30"
// tokio = { version = "1.16.1", features = ["rt-multi-thread"] }
// regex = "0.2.1"
// modbus = "*"

#[macro_use]
extern crate log;
extern crate regex;

use regex::Regex;
use std::fs::File;
use std::io::{BufReader, BufRead};

use std::env;
use std::collections::HashMap;

use modbus::tcp;
use modbus::{Client, Coil};
use std::thread;
use std::time::Duration;

const IPADDRPLC: &str = "192.168.5.41";

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

/// sets the state of output Y1
#[poise::command(prefix_command, slash_command)]
async fn set_Y1(
    ctx: Context<'_>,
    #[description = "state = on/off"] state: String,
) -> Result<(), AppError> {
    let reg = match Regex::new(r"(?i)on") {
        Ok(reg) => reg,
        Err(e) => {
            println!("invalid regex {}",e);
            return;
        }
    };
    if reg.is_match(&state) {
	    out_relayY1(Coil::On, IPADDRPLC);	
	} else {
	    out_relayY1(Coil::Off, IPADDRPLC);
	}
	let Yblock = read_relayY0_7(IPADDRPLC);
    poise::say_reply(ctx, format!("set relay Y1 :- {}", Yblock)).await?;
    Ok(())
}

/// sets the state of outputs Y2 and Y3
#[poise::command(prefix_command, slash_command)]
async fn set_Y2_3(
    ctx: Context<'_>,
    #[description = "state = on/off"] state: String,
) -> Result<(), AppError> {
    let reg = match Regex::new(r"(?i)on") {
        Ok(reg) => reg,
        Err(e) => {
            println!("invalid regex {}",e);
            return;
        }
    };
    if reg.is_match(&state) {
	    out_relayY2_3(Coil::On, IPADDRPLC);	
	} else {
	    out_relayY2_3(Coil::Off, IPADDRPLC);
	}
	let Yblock = read_relayY0_7(IPADDRPLC);
    poise::say_reply(ctx, format!("set relay Y2 & Y3 :- {}", Yblock)).await?;
    Ok(())
}

/// sets the state of outputs e.g. for Y4==on then /set_coil 0x04 on
#[poise::command(prefix_command, slash_command)]
async fn set_coil(
    ctx: Context<'_>,
    #[description = "register address"] reg: u16,
    #[description = "state = on/off"] state: String,
) -> Result<(), AppError> {
    let reg = match Regex::new(r"(?i)on") {
        Ok(reg) => reg,
        Err(e) => {
            println!("invalid regex {}",e);
            return;
        }
    };
    if reg.is_match(&state) {
	    out_relayY(reg, Coil::On, IPADDRPLC);	
	} else {
	    out_relayY(reg, Coil::Off, IPADDRPLC);
	}
	let Yblock = read_relayY0_7(IPADDRPLC);
    poise::say_reply(ctx, format!("set relay Y2 & Y3 :- {}", Yblock)).await?;
    Ok(())
}

/// gets the state of inputs 0-7
#[poise::command(prefix_command, slash_command)]
async fn get_input(
    ctx: Context<'_>,
    #[description = "input register start address"] reg: u16,
    #[description = "number of inputs"] length: u16,
) -> Result<(), AppError> {

	let Xblock = read_inputX(reg, length, IPADDRPLC);
    poise::say_reply(ctx, format!("{}", Xblock)).await?;
    Ok(())
}

/// gets the state of memory
#[poise::command(prefix_command, slash_command)]
async fn get_memory(
    ctx: Context<'_>,
    #[description = "memory M start address"] reg: u16,
    #[description = "number of inputs"] length: u16,
) -> Result<(), AppError> {

	let Mblock = read_memoryM(reg, length, IPADDRPLC);
    poise::say_reply(ctx, format!("{}", Mblock)).await?;
    Ok(())
}

/// sets the state of registers D1 and D2
#[poise::command(prefix_command, slash_command)]
async fn set_D1_D2(
    ctx: Context<'_>,
    #[description = "D1 sets to this value"] state1: u16,
    #[description = "D2 sets to this value"] state2: u16,
) -> Result<(), AppError> {
	write_registerD1_D2(state1, state2, IPADDRPLC);
	let Dblock = read_registerD0_D7(IPADDRPLC);
    poise::say_reply(ctx, format!("set values D1 & D2 :- {}", Dblock)).await?;
    Ok(())
}

/// sets the state of regsiter /set_reg_val reg: 0x02, state: 1000 ==> D<reg> = state
#[poise::command(prefix_command, slash_command)]
async fn set_reg_val(
    ctx: Context<'_>,
    #[description = "register address"] reg: u16,
    #[description = "value for register"] state: u16,
) -> Result<(), AppError> {
	write_reg_val(reg, state, IPADDRPLC);
	let Dblock = read_registerD(reg, IPADDRPLC);
    poise::say_reply(ctx, format!("set value D{} :- {}", reg.to_string(), Dblock)).await?;
    Ok(())
}

/// sets the state of regsiter /set_word_val reg: 0x02, state: 1000 ==> W<reg> = state
#[poise::command(prefix_command, slash_command)]
async fn set_word_val(
    ctx: Context<'_>,
    #[description = "word address"] reg: u16,
    #[description = "value for register"] state: u16,
) -> Result<(), AppError> {
	write_reg_val((reg+30720), state, IPADDRPLC);
	let Wblock = read_registerW(reg, IPADDRPLC);
    poise::say_reply(ctx, format!("set value W{} :- {}", reg.to_string(), Wblock)).await?;
    Ok(())
}

/// sets the state of regsiter /set_reg_val reg: 0x02, state: 1000 ==> D<reg> = state
#[poise::command(prefix_command, slash_command)]
async fn read_reg_val(
    ctx: Context<'_>,
    #[description = "register address"] reg: u16,
) -> Result<(), AppError> {
	let Dblock = read_registerD(reg, IPADDRPLC);
    poise::say_reply(ctx, format!("set value D{} :- {}", reg.to_string(), Dblock)).await?;
    Ok(())
}

/// sets the state of regsiter /set_reg_val reg: 0x02, state: 1000 ==> W<reg> = state
#[poise::command(prefix_command, slash_command)]
async fn read_word_val(
    ctx: Context<'_>,
    #[description = "word address"] reg: u16,
) -> Result<(), AppError> {
	let Wblock = read_registerW(reg, IPADDRPLC);
    poise::say_reply(ctx, format!("set value W{} :- {}", reg.to_string(), Wblock)).await?;
    Ok(())
}

/// search key word in file e.g. /show_io_desc key: = "Y1"
#[poise::command(prefix_command, slash_command)]
async fn show_io_desc(
    ctx: Context<'_>,
    #[description = "key to look for... e.g. title or artist"] key: String,
) -> Result<(), AppError> {
    let filename = "/home/mark/io_list.txt"                             
    let file = match File::open(&filename) {
        Ok(file) => file,
        Err(e) => {
            println!("An error occurred while opening file {}:{}",filename,e);
            return;
        }
    };
    let reg = match Regex::new(&key) {
        Ok(reg) => reg,
        Err(e) => {
            println!("invalid regex {}",e);
            return;
        }
    };
    let input = BufReader::new(file);
    for line in input.lines() {
      let line = match line {
          Ok(line) => line,
          Err(e) => {
              println!("An error occurred reading line {}",e);
              return;
          }
      };
      if reg.is_match(&line) {
          println!("{}", line);
          poise::say_reply(ctx, format!("search matched :- {}", line.to_string())).await?;
      }
    };
    Ok(())
}

/// prints the io and description text file on the channel output line
#[poise::command(prefix_command, slash_command)]
async fn show_all_io(
    ctx: Context<'_>,
    //#[description = "key to look for... e.g. title or artist"] key: String,
) -> Result<(), AppError> {
    let filename = "/home/mark/io_list.txt"                             
    let file = match File::open(&filename) {
        Ok(file) => file,
        Err(e) => {
            println!("An error occurred while opening file {}:{}",filename,e);
            return;
        }
    };
    let input = BufReader::new(file);
    for line in input.lines() {
      let line = match line {
          Ok(line) => line,
          Err(e) => {
              println!("An error occurred reading line {}",e);
              return;
          }
      };
      println!("{}", line);
      poise::say_reply(ctx, format!("{}", line.to_string())).await?;
    };
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
        commands: vec![register(), set_Y1(), set_Y2_3(), set_D1_D2(), set_reg_val(), set_word_val(), read_reg_val(), read_word_val(), set_coil(), get_input(), get_memory(), show_io_desc(), show_all_io(), ],
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

// Output Y	startRegister
// Y0	0x00
// Y1	0x01
// Y2	0x02
// Y3	0x03

// write a single output from the PLC Y1
fn out_relayY1(val: Coil, ipaddr: &str) {
    println!(" --- > Relay write");

    //PLC connect
    let mut client = tcp::Transport::new(ipaddr).unwrap();

    //write single → Y1
    match client.write_single_coil(0x01, val) {
        Ok(_) => {}
        Err(err) => {
            eprintln!("{}", err.to_string());
            return;
        }
    }
    thread::sleep(Duration::from_millis(10));
		
    //PLC dis-connection
    match client.close() {
        Ok(_) => {}
        Err(err) => {
            eprintln!("{}", err.to_string());
            return;
        }
    }

    //
    println!(" ---  > Relay Y / {:?}", val)
}

// write two outputs from the PLC Y2-3
fn out_relayY2_3(val: Coil, ipaddr: &str) {
    println!(" --- > Relay write");

    //PLC connection
    let mut client = tcp::Transport::new(ipaddr).unwrap();

    //write to Y2-3 so therefore start at 2
    match client.write_multiple_coils(0x02, &[val, val]) {
        Ok(_) => {}
        Err(err) => {
            eprintln!("{}", err.to_string());
            return;
        }
    }
    thread::sleep(Duration::from_millis(10));
	
    //PLC dis-connection
    match client.close() {
        Ok(_) => {}
        Err(err) => {
            eprintln!("{}", err.to_string());
            return;
        }
    }

    //
    println!(" ---  > Relay Y / {:?}", val)
}

// write a single output from the PLC Y<reg>
fn out_relayY(reg: u16, val: Coil, ipaddr: &str) {
    println!(" --- > Relay write");

    //PLC connect
    let mut client = tcp::Transport::new(ipaddr).unwrap();

    //write single → Y<reg>
    match client.write_single_coil(reg, val) {
        Ok(_) => {}
        Err(err) => {
            eprintln!("{}", err.to_string());
            return;
        }
    }
    thread::sleep(Duration::from_millis(10));
		
    //PLC dis-connection
    match client.close() {
        Ok(_) => {}
        Err(err) => {
            eprintln!("{}", err.to_string());
            return;
        }
    }

    //
    println!(" ---  > Relay Y / {:?}", val)
}

fn read_relayY0_7(ipaddr: &str) -> std::io::Result<String>  {
    println!(" --- > Relay Read");

    //PLC connection	
    let mut client = tcp::Transport::new(ipaddr).unwrap();
	
    //Y0～7
    match client.read_coils(0x00, 7) {
        Ok(data) => {
            println!(" ---  > Relay Y0 - Y7 / {:?}", data)
        }
        Err(err) => {
            eprintln!("{}", err.to_string())
        }
    }
	
    //PLC dis-connection
    match client.close() {
        Ok(_) => {}
        Err(err) => {
            eprintln!("{}", err.to_string());
            return;
        }
    }
	
    Ok(format!("Y0-Y7 {:8}", data).to_string())	
}

fn read_inputX( reg: u16, len: u16, ipaddr: &str) -> std::io::Result<String>  {
    println!(" --- > Relay Read");

    //PLC connection	
    let mut client = tcp::Transport::new(ipaddr).unwrap();
	
    //X0～7
    match client.read_discrete_inputs(reg, len) {
        Ok(data) => {
            println!(" ---  > Input X{} - X{} / {:?}", reg.to_string(), (reg+len).to_string(), data)
        }
        Err(err) => {
            eprintln!("{}", err.to_string())
        }
    }
	
    //PLC dis-connection
    match client.close() {
        Ok(_) => {}
        Err(err) => {
            eprintln!("{}", err.to_string());
            return;
        }
    }
	
    Ok(format!("X{}-X{} {:8}", reg.to_string(), (reg+len).to_string(), data).to_string())	
}

fn read_memoryM( reg: u16, len: u16, ipaddr: &str) -> std::io::Result<String>  {
    println!(" --- > Memory Read");

    //PLC connection	
    let mut client = tcp::Transport::new(ipaddr).unwrap();
	
    //M0～7
    match client.read_coils((reg+8192), len) {
        Ok(data) => {
            println!(" ---  > Memory M{} - M{} / {:?}", reg.to_string(), (reg+len).to_string(), data)
        }
        Err(err) => {
            eprintln!("{}", err.to_string())
        }
    }
	
    //PLC dis-connection
    match client.close() {
        Ok(_) => {}
        Err(err) => {
            eprintln!("{}", err.to_string());
            return;
        }
    }
	
    Ok(format!("M{}-M{} {:8}", reg.to_string(), (reg+len).to_string(), data).to_string())	
}

//Data Registers	startRegister
//D0	0x00
//D1	0x01
//D2	0x02
//D3	0x03
//
fn write_registerD1_D2(val: u16, val2: u16, ipaddr: &str) {
    println!(" --- > Register Write");

    //PLC connection	
    let mut client = tcp::Transport::new(ipaddr).unwrap();
	
    // write to D1-D2 value and value+10
    match client.write_multiple_registers(0x01, &[val, val2]) {
        Ok(_) => {
            println!(" ---  > Registers D1 - D3 / {:?} {:?}", val, val2)
        }
        Err(err) => {
            eprintln!("{}", err.to_string())
        }
    }
    thread::sleep(Duration::from_millis(10));
	
    //PLC dis-connection
    match client.close() {
        Ok(_) => {}
        Err(err) => {
            eprintln!("{}", err.to_string());
            return;
        }
    }
}

// write D<reg> = <val>
fn write_reg_val(reg: u16, val: u16, ipaddr: &str) {
    println!(" --- > Register Write D",reg);

    //PLC connection	
    let mut client = tcp::Transport::new(ipaddr).unwrap();
	
    // write to D1-D2 value and value+10
    match client.write_multiple_registers(reg, val) {
        Ok(_) => {
            println!(" ---  > Registers D / {:?}", val)
        }
        Err(err) => {
            eprintln!("{}", err.to_string())
        }
    }
    thread::sleep(Duration::from_millis(10));
	
    //PLC dis-connection
    match client.close() {
        Ok(_) => {}
        Err(err) => {
            eprintln!("{}", err.to_string());
            return;
        }
    }
}

// read registers D0 to D7
fn read_registerD0_D7(ipaddr: &str) -> std::io::Result<String> {
    println!(" --- > Register Read");
	
    //PLC connection		
    let mut client = tcp::Transport::new(ipaddr).unwrap();
    // D0～7
    match client.read_holding_registers(0x00, 8) {
        Ok(data) => {
            println!(" ---  > Registers D0 - D7 / {:?}", data)
        }
        Err(err) => {
            eprintln!("{}", err.to_string())
        }
    }
    //PLC dis-connection
    match client.close() {
        Ok(_) => {}
        Err(err) => {
            eprintln!("{}", err.to_string());
            return;
        }
    }
    Ok(format!("D0-D7 {}", data).to_string())		
}

// read registers D<reg>
fn read_registerD(reg: u16, ipaddr: &str) -> std::io::Result<String> {
    println!(" --- > Register Read");
	
    //PLC connection		
    let mut client = tcp::Transport::new(ipaddr).unwrap();
    // D0～7
    match client.read_holding_registers(reg, 1) {
        Ok(data) => {
            println!(" ---  > Registers D{} / {:?}", reg.to_string(), data)
        }
        Err(err) => {
            eprintln!("{}", err.to_string())
        }
    }
    //PLC dis-connection
    match client.close() {
        Ok(_) => {}
        Err(err) => {
            eprintln!("{}", err.to_string());
            return;
        }
    }
    Ok(format!("D0-D7 {}", data).to_string())		
}

// read words W<reg>
fn read_registerW(reg: u16, ipaddr: &str) -> std::io::Result<String> {
    println!(" --- > Word Read");
	
    //PLC connection		
    let mut client = tcp::Transport::new(ipaddr).unwrap();
    // W0～7
    match client.read_holding_registers((reg+30720), 1) {
        Ok(data) => {
            println!(" ---  > Word W{} / {:?}", reg.to_string(), data)
        }
        Err(err) => {
            eprintln!("{}", err.to_string())
        }
    }
    //PLC dis-connection
    match client.close() {
        Ok(_) => {}
        Err(err) => {
            eprintln!("{}", err.to_string());
            return;
        }
    }
    Ok(format!("D0-D7 {}", data).to_string())		
}