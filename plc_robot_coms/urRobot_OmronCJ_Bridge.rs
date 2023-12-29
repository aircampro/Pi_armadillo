// Communication with Omron PLC CJ series with FINS protocol and Universal Robot UR3/5/10 over TCP
//
// OMron Fins Commands are summarised below
// Command MRC/SRC	these values appply we use 01 01 in the example below
// 0101	Memory Reads	address, number of elements
// 0102	Memory Writes	Address, Number of Elements, Write Data
// 0103	Bulk Write	Address, Number of Elements, Write Data
// 0104	Compound Readout	Multiple Read Addresses
// 0401	Start of operation	Monitor or Driving
// 0402	Shutdown	
// 0501	CPU Unit Information	
// 0601	CPU Status Readout	
// 0620	Cycle time	Initialize or Read
// 0701	Time information retrieval	
// 0702	Write Time Information	
// 2101	Anomaly release	Fault Codes
// 2102	Anomaly history	Start RecordNo, Number of Records
// 2103	Clearing the anomaly history

//
// Where the FINS header needs to be changed
// ICF	When sending 0x80
// RSV	0x00 fixation
// GCT	0x02 fixation
// Offset party
// FINS address	DNA	Destination network address (0x00 if it is your own network)
// DA1	Destination node address
// DA2	Address of the other machine (to CPU = 0x00)
// Source
// FINS address	SNA	Source network address (0x00 if it's your own network)
// SA1	Source node address (it seems good if it does not overlap with the destination in 01-FE)
// SA2	Sender address (fixed 0x00)
// SID	Arbitrary service ID (usually a value counted up for each communication)
//
// the UR Robot command manual is listed here https://s3-eu-west-1.amazonaws.com/ur-support-site/32554/scriptManual-3.5.4.pdf
//
use std::net::{TcpStream};
use std::io::{BufReader,BufRead,Write};

//Specify address and port i'll show 2 different methods of specifying this 
const OMRON_SERVER_ADDRESS:&str = "127.0.0.1:8080";
let urobo_host = "localhost";
let urobo_port = 3000;

// we'll use these functions to speak to the ur robot 

// read the socket
fn ur_read_something(reader: &mut BufReader<&TcpStream>) {

  let mut msg = String::new();
  reader.read_line(&mut msg).expect("RECEIVE FAILURE!!!");
  println!("{}", msg);
}

// write to the socket
fn ur_write_something(writer: &mut BufWriter<&TcpStream>, msg: &str) {
  
  writer.write(msg.as_bytes()).expect("SEND FAILURE!!!");
  writer.flush().unwrap();
}

// get the data from the omron server and push it to the robot server
fn main() {

    // Connect to the Omron PLC
    let mut sock = TcpStream::connect(OMRON_SERVER_ADDRESS).expect("failed to connect server");
    // Set to Non-Blocking
    sock.set_nonblocking(false).expect("out of service");
    println!("============== connect omron server ================\n");
    let shakehand_protocol = [
       // TCP header
       0x46, 0x49, 0x4E, 0x53,  // FINS
       0x00, 0x00, 0x00, 0x0C,  // Length
       0x00, 0x00, 0x00, 0x00,  // Command
       0x00, 0x00, 0x00, 0x00,  // Error Code

       // FINS node address  0x00 = auto recieve
       0x00, 0x00, 0x00, 0x00,
    ];
    // send the initial shakehand message
    match sock.write_all(shakehand_protocol.as_bytes()) {
        Ok(()) => println!("send test message success"),
        Err(v) => println!("send test message failed:{}",v),
    }

    let mut recv_response = String::new();
    //Use BufReader to read data
    let mut reader = BufReader::new(sock);
    //Wait for a response from the server
    if let Ok(v) = reader.read_line(&mut recv_response) {
        if v > 0 {
            //View Response and set the addresses from the reply
            println!("response:{}",recv_response);
	    let data1 = String::from(format!("{}",recv_response)).into_bytes();
	    let plc_node = data1[19];                 
	    let pc_node = data1[23];                                  
        }
    }
	
    let standby_protocol = [
    // TCP header
        0x46, 0x49, 0x4E, 0x53,  // FINS
        0x00, 0x00, 0x00, 0x1A,  // Length
        0x00, 0x00, 0x00, 0x02,  // Command
        0x00, 0x00, 0x00, 0x00,  // Error Code
    ];
	let sia = 0x55;          // should be a counter that cycles each time we send
    let read_protocol = [
        // header
        0x80,                //(ICF) 
        0x00,                //(RSV) 
        0x02,                //(GCT) 
        0x00,                //(DNA) 
        plc_node,            //(DA1) 
        0x00,                //(DA2) 
        0x00,                //(SNA) 
        pc_node,             //(SA1) 
        0x00,                //(SA2) 
        sia,                 //(SID) 

        // command
        0x01,                //(MRC) memory read from above table.
        0x01,                //(SRC) 
        // data
        0x82,                // Read device specification D=0x82, W=0xB1 H=0xB2 E=0xA0 + bank
        0x00, 0x00, 0x96,    // Read area
        0x00,                // Number of upper read bytes
        0x07,                // Number of read bytes
    ];

    // Send handshake
    match sock.write_all(standby_protocol.as_bytes()) {
        Ok(()) => println!("send test message success"),
        Err(v) => println!("send test message failed:{}",v),
    }
    // Actually Send message to read data
    match sock.write_all(read_protocol.as_bytes()) {
        Ok(()) => println!("send test message success"),
        Err(v) => println!("send test message failed:{}",v),
    }
    let mut msg = String::new();

    // Wait for a response from the Omron CJ series PLC server
    if let Ok(v) = reader.read_line(&mut msg) {
        if v > 0 {
            // View Response
            println!("response:{}",msg);
			let response = String::from(format!("{}",msg)).into_bytes();
            let x1 = (response[4] + response[5]) & 0xFFFF;                  
            let x2 = response[7];                                     
            let x3 = (response[8] + response[9]);                           
            let x4 = (response[10] + response[11]);                         
            let x5 = response[13];                                      
            let x6 = (response[2] + response[3] + response[0] + response[1]) 0xFFFF;
        }
    }
	
	// Now connect to the ur robot with a different TCP connectiom and send the track circular command 
	// with values got from PLC
    let host_and_port = format!("{}:{}", urobo_host, urobo_port);
    let mut addrs = host_and_port.to_socket_addrs().unwrap();

    if let Some(addr) = addrs.find(|x| (*x).is_ipv4()) {
        println!("============== connect universal robot server ================\n");
        match TcpStream::connect(addr) {
            Err(_) => {
                println!("Connection NG.");
            }
            Ok(stream) => {
               println!("Connection Ok.");
	       stream.set_nonblocking(false).expect("out of service");
               let mut reader = BufReader::new(&stream);
               let mut writer = BufWriter::new(&stream);
			   
	       // send the chosen x y values from the PLC to the robot
	       let x_coord = x3 / 10.0;
	       let y_coord = x4 / 10.0;
	       let msg = format!("track_conveyor_circular(p[{},{},0,0,0,0],500.0, false) \n", x_coord, y_coord);
               ur_write_something(&mut writer, msg);
			   
		// get the pose from the robot and print it.
	       let msg_pos = "get_actual_tcp_pose() \n";
               ur_write_something(&mut writer, msg_pos);
               ur_read_something(&mut reader);
          }
        }
    } else {
    eprintln!("Invalid Host:Port Number");
  }
}
