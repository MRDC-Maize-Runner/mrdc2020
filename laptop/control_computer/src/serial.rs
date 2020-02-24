use std::io::Write;
use std::sync::mpsc::{Receiver, Sender};
use std::time::Duration;
use std::{thread, time};

use bytes::Bytes;

use serialport::prelude::*;
extern crate serialport;

use prost::Message;

use crate::controller::state;

pub mod status {
    include!(concat!(env!("OUT_DIR"), "/status.rs"));
}

//set up the serial port
pub fn port_setup(
    port_name: &str,
    baud_rate: u32,
    _timeout: u32,
) -> (
    Box<dyn serialport::SerialPort>,
    Box<dyn serialport::SerialPort>,
) {
    //get a serial port from settings
    let mut settings: SerialPortSettings = Default::default();
    settings.timeout = Duration::from_millis(10);
    settings.baud_rate = baud_rate;

    //duplicate it for duplex transmissions
    let transmit_port =
        serialport::open_with_settings(&port_name, &settings).expect("Failed to serial open port");
    let receive_port = transmit_port.try_clone().expect("Failed to clone");

    return (transmit_port, receive_port);
}

//thread to send data over serial port
pub fn serial_send_data(
    _tui_log_tx: Sender<String>,
    mut port: Box<dyn serialport::SerialPort>,
    rx: Receiver<state::State>,
) {
    loop {
        //receive a controller state, encode it, add a header and send it over serial
        let message: state::State = rx.recv().unwrap();
        let mut buf = Vec::new();
        message.encode(&mut buf);

        //ASCII "Transmission" and the length. This makes the header 13 bytes
        let mut transmission: Vec<u8> = vec![
            84,
            114,
            97,
            110,
            115,
            109,
            105,
            115,
            115,
            105,
            111,
            110,
            message.encoded_len() as u8,
        ];
        transmission.extend(buf);

        port.write(&transmission).unwrap();
    }
}

//get data from the serial port
pub fn serial_get_data(
    tui_log_tx: Sender<String>,
    mut port: Box<dyn serialport::SerialPort>,
    _tx: Sender<Vec<u8>>,
) {
    let header: [u8; 12] = [84, 114, 97, 110, 115, 109, 105, 115, 115, 105, 111, 110];
    let loop_delay = time::Duration::from_millis(10);

    let mut buf: [u8; 12] = [0; 12];
    loop {
        //check if header length is available
        if port.bytes_to_read().unwrap() > 13 {
            //if so read the header and check it matches
            let _ = port.read_exact(&mut buf);
            if buf == header {
                //If it matches read message and decode protobuf
                let mut len: [u8; 1] = [0];
                let _ = port.read_exact(&mut len);
                let mut data = vec![0; len[0] as usize];
                let _ = port.read_exact(&mut data);

                let status = status::Status::decode(Bytes::from(data));
                match status {
                    Ok(val) => {
                        let _ = tui_log_tx.send(val.update);
                    }
                    Err(_e) => {
                        let _ = tui_log_tx.send(String::from("Decode Failed"));
                    }
                }
            }
        }
        thread::sleep(loop_delay);
    }
}
