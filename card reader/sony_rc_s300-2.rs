// pre-requisite install for various linux flavors
//
// "If you are using Debian (or a derivative e.g. ubuntu , try:
// $ sudo apt install pkgconf libpcsclite-dev
// "If you are using Arch (or a derivative e.g. manjaro, try:
// $ sudo pacman -S pkgconf pcsclite
// "If you are using Fedora (or a derivative e.g. red hat, try:
// $ sudo dnf install pkgconf pcsc-lite-devel
//	
// ubuntu
// sudo apt install gcc cmake make automake autoconf libtool pkg-confi
// sudo apt install libusb-1.0-0 libusb-1.0-0-dev
// sudo apt install libpcsclite-dev pcscd pcsc-tools
// sudo apt install http://archive.ubuntu.com/ubuntu/pool/universe/c/ccid/libccid_1.5.0-2_amd64.deb
// sudo systemctl restart pcscd く
//
// manjaro
// sudo pacman -S libusb pcsclite ccid opensc pcsc-tools
// systemctl enable pcscd
// systemctl start pcscd
// pcsc_scan

// you must install rust on your box
// this is based on an article here https://zenn.dev/compass/articles/39bb050bdaeaaa
// which to use the Sony RC-S300 on USB using this rust driver
// https://www.sony.co.jp/Products/felica/consumer/products/RC-S300.html
// 
// format of data is shown here https://tex2e.github.io/blog/protocol/jpki-mynumbercard-with-apdu
//
// This article explains the results of a survey on My number cards, which have almost no published specifications among IC cards

// Cargo.toml
// [dependencies]
// pcsc = "2.7.0"
// dialoguer = "0.10.2"
// hex-literal = "0.3.4"
//
// [patch.crates-io]
// pcsc = { git = "https://github.com/CompassSN/pcsc-rust", package = "pcsc"}
// pcsc-sys = { git = "https://github.com/CompassSN/pcsc-rust", package = "pcsc-sys" }
//
use std::{ffi::CString};
use dialoguer::Select;
use hex_literal::hex;
use pcsc::{Context, Scope, ShareMode, Protocols, MAX_BUFFER_SIZE};

// return ascii from byte (extended table)
fn to_ascii(i: &u32) -> String {
        match *i {
                    x@0..=255 => format!("{:?}", x as u8 as char),
                            _ => "".into(),
                 }
}

fn main() {
    // let context = Context::establish(Scope::User).unwrap();

    // Establish a PC/SC context.with error handler
    let ctx = match Context::establish(Scope::User) {
        Ok(ctx) => ctx,
        Err(err) => {
            println!("Failed to establish context: {}", err);
            std::process::exit(1);
        }
    };
    let context = ctx.unwrap();	
	
    let mut readers_buf = [0; 2048];
    let mut readers = context.list_readers(&mut readers_buf).unwrap();
    let readers_string:Vec<_> = readers.clone().map(|r| {let cs = CString::from(r); cs.to_str().unwrap().to_string()}).collect();

    let selection = Select::new()
        .default(0)
        .items(&readers_string)
        .interact();

    let Ok(index) = selection else {return};

    let reader = readers.nth(index).unwrap();
    println!("Card reader to be used: {reader:?}");

    // let card = context.connect(reader, ShareMode::Shared, Protocols::ANY).unwrap();
	
    // Connect to the card. with error handler
    let card1 = match context.connect(reader, ShareMode::Shared, Protocols::ANY) {
        Ok(card1) => card1,
        Err(Error::NoSmartcard) => {
            println!("A smartcard is not present in the reader.");
            return;
        }
        Err(err) => {
            println!("Failed to connect to card: {}", err);
            std::process::exit(1);
        }
    };
    let card = card1.unwrap();

    // read an ID from the card
    let apdu = hex!("FF CA 00 00 00");
    let mut rapdu_buf = [0; MAX_BUFFER_SIZE];

    let rapdu = card.transmit(&apdu, &mut rapdu_buf).unwrap();

    let len = rapdu.len();
    let result = &rapdu[len-2..len];
    if !(*result.get(0).unwrap() == 0x90 && *result.get(1).unwrap() == 0x00){           // sends 0x90 0x00 on success
        println!("Failed to read IDm.");
    } else {
        let idm = &rapdu[..len-2];
        println!("{idm:02X?}", );
    }
	
    // ------------------------- talk to my number card. ---------------------------------
	//            ref :- https://tex2e.github.io/blog/protocol/jpki-mynumbercard-with-apdu
	
	// obtaining a certificate
	// Select Public Personal Authentication APS (DF)
	// DF name：D3 92 F0 00 26 01 00 00 00 01
    let apdu1 = hex!("00 A4 04 0C 0A D3 92 F0 00 26 01 00 00 00 01");
    let mut rapdu_buf1 = [0; MAX_BUFFER_SIZE];

    let rapdu1 = card.transmit(&apdu1, &mut rapdu_buf1).unwrap();

    let len1 = rapdu1.len();
    let result1 = &rapdu1[len1-2..len1];                                                    // last 2 bytes
    if !(*result1.get(0).unwrap() == 0x90 && *result1.get(1).unwrap() == 0x00){             // sends 0x90 0x00 on success
        println!("Select Public Personal Authentication APS (DF) Failed");
    } 
	// Select a Certificate for authentication (EF)
	// File ID:00 0A
    let apdu2 = hex!("00 A4 02 0C 02 00 0A");
    let mut rapdu_buf2 = [0; MAX_BUFFER_SIZE];

    let rapdu2 = card.transmit(&apdu2, &mut rapdu_buf2).unwrap();

    let len2 = rapdu2.len();
    let result2 = &rapdu2[len2-2..len2];                                                    // last 2 bytes
    if !(*result2.get(0).unwrap() == 0x90 && *result2.get(1).unwrap() == 0x00){             // sends 0x90 0x00 on success
        println!("Select a Certificate for authentication (EF) Failed");
    } 
    // Read DER certificate part 1
    let apdu3 = hex!("00 B0 00 00 04");
    let mut rapdu_buf3 = [0; MAX_BUFFER_SIZE];

    let rapdu3 = card.transmit(&apdu3, &mut rapdu_buf3).unwrap();

    let len3 = rapdu3.len();
    let result3 = &rapdu3[len3-2..len3];                                                    // last 2 bytes 
    if !(*result3.get(0).unwrap() == 0x90 && *result3.get(1).unwrap() == 0x00){             // sends 0x90 0x00 on success
        println!("Failed to read DER certificate.pt1");
    } else {
        let tlv_der1 = &rapdu3[..len3-2];
        println!("{tlv_der1:02X?}", );
        let len_tlvder1 = tlv_der1.len();
        let head1 = &tlv_der1[..2];                                                         // first 2 bytes 
        if !(*head1.get(0).unwrap() == 0x30 && *head1.get(1).unwrap() == 0x82){             // tag sequence
            println!("Header not correct when reading DER certificate - step1");
        } else {
           let data1 = &tlv_der1[2..len_tlvder1];                                           // prints the data
           println!("{data1:02X?}", );
        }
    }
    // Read DER certificate part 2	
    let apdu4 = hex!("00 B0 00 04 00 06 1F");
    let mut rapdu_buf4 = [0; MAX_BUFFER_SIZE];

    let rapdu4 = card.transmit(&apdu4, &mut rapdu_buf4).unwrap();

    let len4 = rapdu4.len();
    let result4 = &rapdu4[len4-2..len4];                                                    // last 2 bytes 
    if !(*result4.get(0).unwrap() == 0x90 && *result4.get(1).unwrap() == 0x00){             // sends 0x90 0x00 on success
        println!("Failed to read DER certificate.pt2");
    } else {
        let tlv_der4 = &rapdu4[..len4-2];                                                   // message is minus the last 2 transaction chars
        println!("{tlv_der4:02X?}", );
        let len_tlvder4 = tlv_der4.len();
        let head4 = &tlv_der4[..2];                                                         // first 2 bytes 
        if !(*head4.get(0).unwrap() == 0x30 && *head4.get(1).unwrap() == 0x82){             // tag sequence
            println!("Header not correct when reading DER certificate - step2");
        } else {
           let len_data_tlv = &tlv_der4[2..4];                                              // data containg the length
           println!("data length {}", *len_data_tlv);		   
           let data4 = &tlv_der4[4..len_tlvder4];                                           // prints the data
           println!("{data4:02X?}", );
        }
    }
	
    // Signing with a private key for authentication
	// Select Public Personal Authentication APS (DF)
	// DF name：D3 92 F0 00 26 01 00 00 00 01
    let apdu1 = hex!("00 A4 04 0C 0A D3 92 F0 00 26 01 00 00 00 01");
    let mut rapdu_buf1 = [0; MAX_BUFFER_SIZE];

    let rapdu1 = card.transmit(&apdu1, &mut rapdu_buf1).unwrap();

    let len1 = rapdu1.len();
    let result1 = &rapdu1[len1-2..len1];                                                    // last 2 bytes
    if !(*result1.get(0).unwrap() == 0x90 && *result1.get(1).unwrap() == 0x00){             // sends 0x90 0x00 on success
        println!("Select Public Personal Authentication APS (DF) Failed");
    } 
	// Select a PIN for Authentication (EF)
	// File Identifier: 00 18
    let apdu2 = hex!("00 A4 02 0C 02 00 18");
    let mut rapdu_buf2 = [0; MAX_BUFFER_SIZE];

    let rapdu2 = card.transmit(&apdu2, &mut rapdu_buf2).unwrap();

    let len2 = rapdu2.len();
    let result2 = &rapdu2[len2-2..len2];                                                    // last 2 bytes
    if !(*result2.get(0).unwrap() == 0x90 && *result2.get(1).unwrap() == 0x00){             // sends 0x90 0x00 on success
        println!("Select a Certificate for authentication (EF) Failed");
    } 
    // Update the security status by entering an authentication password using the password 1235
    let apdu3 = hex!("00 20 00 80 04 31 32 33 35");
    let mut rapdu_buf3 = [0; MAX_BUFFER_SIZE];

    let rapdu3 = card.transmit(&apdu3, &mut rapdu_buf3).unwrap();

    let len3 = rapdu3.len();
    let result3 = &rapdu3[len3-2..len3];                                                    // last 2 bytes 
    if !(*result3.get(0).unwrap() == 0x90 && *result3.get(1).unwrap() == 0x00){             // sends 0x90 0x00 on success
        println!("Failed to send correct password PIN number");
    } 
	// Select a private key for authentication (EF
	// File Identifier: 00 17	
    let apdu4 = hex!("00 A4 02 0C 02 00 17");
    let mut rapdu_buf4 = [0; MAX_BUFFER_SIZE];

    let rapdu4 = card.transmit(&apdu4, &mut rapdu_buf4).unwrap();

    let len4 = rapdu4.len();
    let result4 = &rapdu4[len4-2..len4];                                                    // last 2 bytes 
    if !(*result4.get(0).unwrap() == 0x90 && *result4.get(1).unwrap() == 0x00){             // sends 0x90 0x00 on success
        println!("Failed to Select a private key for authentication (EF)");
    }
    // Send and sign the target data
    // format is let apdu5 = hex!("80 2A 00 80 33 ...your data... 00");
	let your_data = [ 0x00, 0x33, 0x34, ];
    let apdu5_arr = [
    // header
    0x80,                 
    0x2A,                 
    0x00,                 
    0x80,  
    0x33,
    // your data	
    your_data[0],             
    your_data[1],                 
    your_data[2],                 
    // end
    0x00,                                 /
    ];
	let apdu5 = apdu5_arr.as_bytes();
    let mut rapdu_buf5 = [0; MAX_BUFFER_SIZE];

    let rapdu5 = card.transmit(&apdu5, &mut rapdu_buf5).unwrap();
	
    let len5 = rapdu5.len();
    let result5 = &rapdu5[len5-2..len5];                                                    // last 2 bytes 
    if !(*result5.get(0).unwrap() == 0x90 && *result5.get(1).unwrap() == 0x00){             // sends 0x90 0x00 on success
        println!("Send and sign the target data");
    } else {
        let data_back = &rapdu5[..len5-2];                                                   // message is minus the last 2 transaction chars
        println!("{data_back:02X?}", );
    } 	
	
    // sign with the "Signature private key" is as follows
	// Select Public Personal Authentication APS (DF)
	// DF name：D3 92 F0 00 26 01 00 00 00 01
    let apdu1 = hex!("00 A4 04 0C 0A D3 92 F0 00 26 01 00 00 00 01");
    let mut rapdu_buf1 = [0; MAX_BUFFER_SIZE];

    let rapdu1 = card.transmit(&apdu1, &mut rapdu_buf1).unwrap();

    let len1 = rapdu1.len();
    let result1 = &rapdu1[len1-2..len1];                                                    // last 2 bytes
    if !(*result1.get(0).unwrap() == 0x90 && *result1.get(1).unwrap() == 0x00){             // sends 0x90 0x00 on success
        println!("Select Public Personal Authentication APS (DF) Failed");
    } 
	// Select a PIN for Authentication (EF)
	// File Identifier: 00 1B
    let apdu2 = hex!("00 A4 02 0C 02 00 1B");
    let mut rapdu_buf2 = [0; MAX_BUFFER_SIZE];

    let rapdu2 = card.transmit(&apdu2, &mut rapdu_buf2).unwrap();

    let len2 = rapdu2.len();
    let result2 = &rapdu2[len2-2..len2];                                                    // last 2 bytes
    if !(*result2.get(0).unwrap() == 0x90 && *result2.get(1).unwrap() == 0x00){             // sends 0x90 0x00 on success
        println!("Select a Certificate for authentication (EF) Failed");
    } 
    // Update the security status by entering an authentication password using the password 1234567
    let apdu3 = hex!("00 20 00 80 06 31 32 33 34 35 36 37");
    let mut rapdu_buf3 = [0; MAX_BUFFER_SIZE];

    let rapdu3 = card.transmit(&apdu3, &mut rapdu_buf3).unwrap();

    let len3 = rapdu3.len();
    let result3 = &rapdu3[len3-2..len3];                                                    // last 2 bytes 
    if !(*result3.get(0).unwrap() == 0x90 && *result3.get(1).unwrap() == 0x00){             // sends 0x90 0x00 on success
        println!("Failed to send correct password PIN number");
    } 
	// Select a private key for authentication (EF
	// File Identifier: 00 1A	
    let apdu4 = hex!("00 A4 02 0C 02 00 1A");
    let mut rapdu_buf4 = [0; MAX_BUFFER_SIZE];

    let rapdu4 = card.transmit(&apdu4, &mut rapdu_buf4).unwrap();

    let len4 = rapdu4.len();
    let result4 = &rapdu4[len4-2..len4];                                                    // last 2 bytes 
    if !(*result4.get(0).unwrap() == 0x90 && *result4.get(1).unwrap() == 0x00){             // sends 0x90 0x00 on success
        println!("Failed to Select a private key for authentication (EF)");
    }
    // Send and sign the target data
    // format is let apdu5 = hex!("80 2A 00 80 33 ...your data... 00");
	let your_data = [ 0x00, 0x33, 0x34, ];
    let apdu5_arr = [
    // header
    0x80,                 
    0x2A,                 
    0x00,                 
    0x80,  
    0x33,
    // your data	
    your_data[0],             
    your_data[1],                 
    your_data[2],                 
    // end
    0x00,                                 /
    ];
	let apdu5 = apdu5_arr.as_bytes();
    let mut rapdu_buf5 = [0; MAX_BUFFER_SIZE];

    let rapdu5 = card.transmit(&apdu5, &mut rapdu_buf5).unwrap();

    let len5 = rapdu5.len();
    let result5 = &rapdu5[len5-2..len5];                                                    // last 2 bytes 
    if !(*result5.get(0).unwrap() == 0x90 && *result5.get(1).unwrap() == 0x00){             // sends 0x90 0x00 on success
        println!("Send and sign the target data");
    } else {
        let data_back = &rapdu5[..len5-2];                                                   // message is minus the last 2 transaction chars
        println!("{data_back:02X?}", );
    } 

    // Acquisition of My number (individual number)
	// Select Public Personal Authentication APS (DF)
	// D3 92 10 00 31 00 01 01 04 08
    let apdu1 = hex!("00 A4 04 0C 0A D3 92 10 00 31 00 01 01 04 08");
    let mut rapdu_buf1 = [0; MAX_BUFFER_SIZE];

    let rapdu1 = card.transmit(&apdu1, &mut rapdu_buf1).unwrap();

    let len1 = rapdu1.len();
    let result1 = &rapdu1[len1-2..len1];                                                    // last 2 bytes
    if !(*result1.get(0).unwrap() == 0x90 && *result1.get(1).unwrap() == 0x00){             // sends 0x90 0x00 on success
        println!("Select Public Personal Authentication APS (DF) Failed");
    } 
	// Select a PIN for Authentication (EF)
	// File Identifier: 00 11
    let apdu2 = hex!("00 A4 02 0C 02 00 11");
    let mut rapdu_buf2 = [0; MAX_BUFFER_SIZE];

    let rapdu2 = card.transmit(&apdu2, &mut rapdu_buf2).unwrap();

    let len2 = rapdu2.len();
    let result2 = &rapdu2[len2-2..len2];                                                    // last 2 bytes
    if !(*result2.get(0).unwrap() == 0x90 && *result2.get(1).unwrap() == 0x00){             // sends 0x90 0x00 on success
        println!("Select a Certificate for authentication (EF) Failed");
    } 
    // Update the security status by entering an authentication password using the password 7634
    let apdu3 = hex!("00 20 00 80 04 37 36 33 34");
    let mut rapdu_buf3 = [0; MAX_BUFFER_SIZE];

    let rapdu3 = card.transmit(&apdu3, &mut rapdu_buf3).unwrap();

    let len3 = rapdu3.len();
    let result3 = &rapdu3[len3-2..len3];                                                    // last 2 bytes 
    if !(*result3.get(0).unwrap() == 0x90 && *result3.get(1).unwrap() == 0x00){             // sends 0x90 0x00 on success
        println!("Failed to send correct password PIN number");
    } 
	// Select a private key for authentication (EF)
	// File Identifier: 00 01	
    let apdu4 = hex!("00 A4 02 0C 02 00 01");
    let mut rapdu_buf4 = [0; MAX_BUFFER_SIZE];

    let rapdu4 = card.transmit(&apdu4, &mut rapdu_buf4).unwrap();

    let len4 = rapdu4.len();
    let result4 = &rapdu4[len4-2..len4];                                                    // last 2 bytes 
    if !(*result4.get(0).unwrap() == 0x90 && *result4.get(1).unwrap() == 0x00){             // sends 0x90 0x00 on success
        println!("Failed to Select a private key for authentication (EF)");
    }
    // Send the target data and sign it.	
    let apdu5 = hex!("00 B0 00 00 00");
    let mut rapdu_buf5 = [0; MAX_BUFFER_SIZE];

    let rapdu5 = card.transmit(&apdu5, &mut rapdu_buf5).unwrap();

    let len5 = rapdu5.len();
    let result5 = &rapdu5[len5-2..len5];                                                    // last 2 bytes 
    if !(*result5.get(0).unwrap() == 0x90 && *result5.get(1).unwrap() == 0x00){             // sends 0x90 0x00 on success
        println!("Send and sign the target data");
    } else {
        let data_back = &rapdu5[..len5-2];                                                   // message is minus the last 2 transaction chars
        println!("{data_back:02X?}", );
    } 	
		
    // The four basic pieces of information are name, address, date of birth, and gender. 
    // This information can also be obtained by using the Ticket Entry Assistance AP
	// Select Public Personal Authentication APS (DF)
	// D3 92 10 00 31 00 01 01 04 08
    let apdu1 = hex!("00 A4 04 0C 0A D3 92 10 00 31 00 01 01 04 08");
    let mut rapdu_buf1 = [0; MAX_BUFFER_SIZE];

    let rapdu1 = card.transmit(&apdu1, &mut rapdu_buf1).unwrap();

    let len1 = rapdu1.len();
    let result1 = &rapdu1[len1-2..len1];                                                    // last 2 bytes
    if !(*result1.get(0).unwrap() == 0x90 && *result1.get(1).unwrap() == 0x00){             // sends 0x90 0x00 on success
        println!("Select Public Personal Authentication APS (DF) Failed");
    } 
	// Select a PIN for Authentication (EF)
	// File Identifier: 00 11
    let apdu2 = hex!("00 A4 02 0C 02 00 11");
    let mut rapdu_buf2 = [0; MAX_BUFFER_SIZE];

    let rapdu2 = card.transmit(&apdu2, &mut rapdu_buf2).unwrap();

    let len2 = rapdu2.len();
    let result2 = &rapdu2[len2-2..len2];                                                    // last 2 bytes
    if !(*result2.get(0).unwrap() == 0x90 && *result2.get(1).unwrap() == 0x00){             // sends 0x90 0x00 on success
        println!("Select a Certificate for authentication (EF) Failed");
    } 
    // Update the security status by entering an authentication password using the password 7634
    let apdu3 = hex!("00 20 00 80 04 31 32 33 34");
    let mut rapdu_buf3 = [0; MAX_BUFFER_SIZE];

    let rapdu3 = card.transmit(&apdu3, &mut rapdu_buf3).unwrap();

    let len3 = rapdu3.len();
    let result3 = &rapdu3[len3-2..len3];                                                    // last 2 bytes 
    if !(*result3.get(0).unwrap() == 0x90 && *result3.get(1).unwrap() == 0x00){             // sends 0x90 0x00 on success
        println!("Failed to send correct password PIN number");
    } 
	// Select a private key for authentication (EF)
	// File Identifier: 00 02	
    let apdu4 = hex!("00 A4 02 0C 02 00 02");
    let mut rapdu_buf4 = [0; MAX_BUFFER_SIZE];

    let rapdu4 = card.transmit(&apdu4, &mut rapdu_buf4).unwrap();

    let len4 = rapdu4.len();
    let result4 = &rapdu4[len4-2..len4];                                                    // last 2 bytes 
    if !(*result4.get(0).unwrap() == 0x90 && *result4.get(1).unwrap() == 0x00){             // sends 0x90 0x00 on success
        println!("Failed to Select a private key for authentication (EF)")
    }
    // READ BINARY: Read Basic 4 information (3rd byte data length only)	
    let apdu5 = hex!("00 B0 00 02 01");
    let mut rapdu_buf5 = [0; MAX_BUFFER_SIZE];

    let rapdu5 = card.transmit(&apdu5, &mut rapdu_buf5).unwrap();

    let len5 = rapdu5.len();
    let result5 = &rapdu5[len5-2..len5];                                                    // last 2 bytes 
    if !(*result5.get(0).unwrap() == 0x90 && *result5.get(1).unwrap() == 0x00){             // sends 0x90 0x00 on success
        println!("Send and sign the target data");
    } else {
        let data_back = &rapdu5[..len5-2];                                                   // message is minus the last 2 transaction chars
        println!("{data_back:02X?}", );
    } 

    // Read Basic 4 information (3 + 0x68)	
    let apdu6 = hex!("00 B0 00 00 71");
    let mut rapdu_buf6 = [0; MAX_BUFFER_SIZE];

    let rapdu6 = card.transmit(&apdu6, &mut rapdu_buf6).unwrap();

    let len6 = rapdu6.len();
    let result6 = &rapdu6[len6-2..len6];                                                    // last 2 bytes 
    if !(*result6.get(0).unwrap() == 0x90 && *result6.get(1).unwrap() == 0x00){             // sends 0x90 0x00 on success
        println!("Failed to send request to read person data");
    } else {
        // ff 20 62 df 21 08 ... Header ... DF 22 0F ... Name ...
        // DF 23 39 ... Address ... DF 24 08 ... Date of birth ... DF 25 01 Gender 90 0
        let msg1 = &rapdu6[..len6-2];                                                   // message is minus the last 2 transaction chars
        println!("{msg1:02X?}", );
        let len_msg1 = msg1.len();
        let head6 = &msg1[..6];                                                         // first 6 bytes 
        if !(((*head6.get(0).unwrap() == 0xFF && *head6.get(1).unwrap() == 0x20) && (*head6.get(2).unwrap() == 0x62 && *head6.get(3).unwrap() == 0xDF)) && (*head6.get(4).unwrap() == 0x21 && *head6.get(5).unwrap() == 0x08)) {     // tag sequence
            println!("Header not correct when reading person data");
        } else {
            let rest_of_msg = &msg1[6..len6-2];                                              // data containg the length
            println!("{rest_of_msg:02X?}", );

            // ----------------- parse the message to string ------------------------	
            //			
            // set of variables representing steps in the parser
            // 0 means active step and counts up as we get to the delimeter
            let header1 = 0;
            let name1 = 99;
            let address1 = 99;
            let dob1 = 99;

            // make strings to hold the answers
            let hdr = "Header : ";
            let mut ss_hdr = String::from(hdr);
            let nm = "Name : ";
            let mut ss_nm = String::from(nm);
            let adr = "Address : ";
            let mut ss_adr = String::from(adr);
            let dob = "D.O.B : ";
            let mut ss_dob = String::from(dob);

            // parse the message received for the above fields using the given delimeters
            for j in 0..rest_of_msg.len() {
                // header is delimeted by DF 22 0F
	            if (*rest_of_msg.get(j).unwrap() == 0x0F) && (header1 == 2) {
                    let header1 = 3;
                    let name1 = 0;
                } else if (header1 == 2) {
                    //let hdr_buf[j-2] = 0xDF;
                    //let hdr_buf[j-1] = 0x22;
		            // put the 2 collections back to the header string	
                    let rcv_str = to_ascii(&0xDF);
                    let s2_collection: Vec<char> = rcv_str.chars().collect();
                    ss_hdr.push(s2_collection[1]);
                    let rcv_str = to_ascii(&0x22);
                    let s2_collection: Vec<char> = rcv_str.chars().collect();
                    ss_hdr.push(s2_collection[1]);
                    let rcv_u = *rest_of_msg.get(j).unwrap());	
                    let rcv_str = to_ascii(&rcv_u);
                    if (rcv_str.len() > 1) {
                       let s2_collection: Vec<char> = rcv_str.chars().collect();
                       ss_hdr.push(s2_collection[1]);
                    }
                    let header1 = 4;	
                }
	            if (*rest_of_msg.get(j).unwrap() == 0x22) && (header1 == 1) {
                    let header1 = header1 + 1;
                } else if (header1 == 1) {
                    //let hdr_buf[j-1] = 0xDF;	
                    let rcv_u = 0xDF;	
                    let rcv_str = to_ascii(&rcv_u);
                    let s2_collection: Vec<char> = rcv_str.chars().collect();
                    ss_hdr.push(s2_collection[1]);
                    let header1 = 4;
                }
				if (*rest_of_msg.get(j).unwrap() == 0xDF) && (header1 == 0) {
                    let header1 = header1 + 1;
	            } else if (header1 == 0) {
                    //set ss_hdr[j] = *rest_of_msg.get(j).unwrap();
                    let rcv_u = *rest_of_msg.get(j).unwrap());	
                    let rcv_str = to_ascii(&rcv_u);
                    if (rcv_str.len() > 1) {
                       let s2_collection: Vec<char> = rcv_str.chars().collect();
                       ss_hdr.push(s2_collection[1]);
                    }		
	            } else if (header1 == 4) {
                    let header1 = 0;
				}

                // name string parse delim = DF 23 39
	            if (*rest_of_msg.get(j).unwrap() == 0x39) && (name1 == 2) {
                    let name1 = 3;
                    let address1 = 0;
                } else if (name1 == 2) {
                    //name_buf[j-2] = 0xDF;
                    //name_buf[j-1] = 0x23;	
                    let rcv_str = to_ascii(&0xDF);
                    let s2_collection: Vec<char> = rcv_str.chars().collect();
                    ss_nm.push(s2_collection[1]);
                    let rcv_str = to_ascii(&0x23);
                    let s2_collection: Vec<char> = rcv_str.chars().collect();
                    ss_nm.push(s2_collection[1]);
                    //name_buf[j] = *rest_of_msg.get(j).unwrap();
                    let rcv_u = *rest_of_msg.get(j).unwrap());	
                    let rcv_str = to_ascii(&rcv_u);
                    if (rcv_str.len() > 1) {
                        let s2_collection: Vec<char> = rcv_str.chars().collect();
                        ss_nm.push(s2_collection[1]);
                    }
                    let name1 = 4;	
                }
	            if (*rest_of_msg.get(j).unwrap() == 0x23) && (name1 == 1) {
                    let name1 = name1 + 1;
                } else if (name1 == 1) {
                    //name_buf[j-1] = 0xDF;	
                    let rcv_str = to_ascii(&0xDF);
                    let s2_collection: Vec<char> = rcv_str.chars().collect();
                    ss_nm.push(s2_collection[1]);
                    //name_buf[j] = *rest_of_msg.get(j).unwrap();
                    let rcv_u = *rest_of_msg.get(j).unwrap());	
                    let rcv_str = to_ascii(&rcv_u);
                    if (rcv_str.len() > 1) {
                        let s2_collection: Vec<char> = rcv_str.chars().collect();
                        ss_nm.push(s2_collection[1]);
                    }
                    let name1 = 4;
                }
                if (*rest_of_msg.get(j).unwrap() == 0xDF) && (name1 == 0) {
                    let name1 = name1 + 1;
	            } else if (name1 == 0) {
                    //name_buf[j] = *rest_of_msg.get(j).unwrap();
                    let rcv_u = *rest_of_msg.get(j).unwrap());	
                    let rcv_str = to_ascii(&rcv_u);
                    if (rcv_str.len() > 1) {
                        let s2_collection: Vec<char> = rcv_str.chars().collect();
                        ss_nm.push(s2_collection[1]);
                    }
                } else if (name1 == 4) {
                    let name1 = 0;
				}

                // address string parse delim = DF 24 08
	            if (*rest_of_msg.get(j).unwrap() == 0x08) && (address1 == 2) {
                    let address1 = 3;
                    let dob1 = 0;
                } else if (address1 == 2) {
                    //address_buf[j-2] = 0xDF;
                    //address_buf[j-1] = 0x24;	
                    let rcv_str = to_ascii(&0xDF);
                    let s2_collection: Vec<char> = rcv_str.chars().collect();
                    ss_adr.push(s2_collection[1]);
                    let rcv_str = to_ascii(&0x24);
                    let s2_collection: Vec<char> = rcv_str.chars().collect();
                    ss_adr.push(s2_collection[1]);
                    let rcv_u = *rest_of_msg.get(j).unwrap());	
                    let rcv_str = to_ascii(&rcv_u);
                    if (rcv_str.len() > 1) {
                       let s2_collection: Vec<char> = rcv_str.chars().collect();
                       ss_adr.push(s2_collection[1]);
                    }
                    let address1 = 4;	
                }
	            if (*rest_of_msg.get(j).unwrap() == 0x24) && (address1 == 1) {
                    let address1 = address1 + 1;
                } else if (address1 == 1) {
                    //address_buf[j-1] = 0xDF;
                    let rcv_str = to_ascii(&0xDF);
                    let s2_collection: Vec<char> = rcv_str.chars().collect();
                    ss_adr.push(s2_collection[1]);
                    let rcv_u = *rest_of_msg.get(j).unwrap());	
                    let rcv_str = to_ascii(&rcv_u);
                    if (rcv_str.len() > 1) {
                       let s2_collection: Vec<char> = rcv_str.chars().collect();
                       ss_adr.push(s2_collection[1]);
                    }
                    let address1 = 4;
                }
                if (*rest_of_msg.get(j).unwrap() == 0xDF) && (address1 == 0) {
                    let address1 = address1 + 1;
	            } else if (address1 == 0) {
                    //address_buf[j] = *rest_of_msg.get(j).unwrap();
                    let rcv_u = *rest_of_msg.get(j).unwrap());	
                    let rcv_str = to_ascii(&rcv_u);
                    if (rcv_str.len() > 1) {
                       let s2_collection: Vec<char> = rcv_str.chars().collect();
                       ss_adr.push(s2_collection[1]);
                    }
                } else if (address1 == 4) {
                    let address1 = 0;
                }

	
                // Date of Birth delimeted by DF 25 01
	            if (*rest_of_msg.get(j).unwrap() == 0x01) && (dob1 == 2) {
                    dob1 = 3;
                } else if (dob1 == 2) {
                    //dob_buf[j-2] = 0xDF;
                    //dob_buf[j-1] = 0x25;
                    let rcv_str = to_ascii(&0xDF);
                    let s2_collection: Vec<char> = rcv_str.chars().collect();
                    ss_dob.push(s2_collection[1]);
                    let rcv_str = to_ascii(&0x25);
                    let s2_collection: Vec<char> = rcv_str.chars().collect();
                    ss_dob.push(s2_collection[1]);
                    let rcv_u = *rest_of_msg.get(j).unwrap());	
                    let rcv_str = to_ascii(&rcv_u);
                    if (rcv_str.len() > 1) {
                        let s2_collection: Vec<char> = rcv_str.chars().collect();
                        ss_dob.push(s2_collection[1]);
                    }
                    let dob1 = 4;	
                }
	            if (*rest_of_msg.get(j).unwrap() == 0x25) && (dob1 == 1) {
                    let dob1 = dob1 + 1;
                } else if (dob1 == 1) {
                    //dob_buf[j-1] = 0xDF;
                    let rcv_str = to_ascii(&0xDF);
                    let s2_collection: Vec<char> = rcv_str.chars().collect();
                    ss_dob.push(s2_collection[1]);
                    let rcv_u = *rest_of_msg.get(j).unwrap());	
                    let rcv_str = to_ascii(&rcv_u);
                    if (rcv_str.len() > 1) {
                        let s2_collection: Vec<char> = rcv_str.chars().collect();
                        ss_dob.push(s2_collection[1]);
                    }
                    let dob1 = 4;
                }
                if (*rest_of_msg.get(j).unwrap() == 0xDF) && (dob1 == 0) {
                    let dob1 = dob1 + 1;
	            } else if (dob1 == 0) {
                    //dob_buf[j] = *rest_of_msg.get(j).unwrap();	
                    let rcv_u = *rest_of_msg.get(j).unwrap());	
                    let rcv_str = to_ascii(&rcv_u);
                    if (rcv_str.len() > 1) {
                        let s2_collection: Vec<char> = rcv_str.chars().collect();
                        ss_dob.push(s2_collection[1]);
                    }
                } else if (dob1 == 4) {
                    let dob1 = 0;
				}


			}
        }
        // print the results read from the card
        println!("From Card {} {} {} {}",ss_hdr,ss_nm,ss_adr,ss_dob);
    }	

}