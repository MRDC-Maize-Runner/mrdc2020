use std::io::Write;
use std::mem::transmute;
use std::sync::mpsc::{Receiver, Sender};
use std::time::Duration;

use serialport::prelude::*;
extern crate serialport;

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
    rx: Receiver<(f32, f32, Vec<(&str, bool)>)>,
) {
    loop {
        let message: (f32, f32, Vec<(&str, bool)>) = rx.recv().unwrap();
        let bytes: Vec<u8> = construct_message(message);
        port.write(&bytes).expect("Cant write");
    }
}
//self explanitory
fn bool_to_u8(b: bool) -> u8 {
    match b {
        true => 1,
        false => 0,
    }
}

//implements our protocol to make a serial transmission
pub fn construct_message(message: (f32, f32, Vec<(&str, bool)>)) -> Vec<u8> {
    let mut bytes: Vec<u8> = Vec::new();
    let mut bytes_from_float: [u8; 4] = unsafe { transmute(message.0.to_bits().to_be()) };
    bytes.extend(bytes_from_float.iter().cloned());
    bytes_from_float = unsafe { transmute(message.1.to_bits().to_be()) };
    bytes.extend(bytes_from_float.iter().cloned());
    let mut bit_num = 8;
    let mut byte: u8 = 0;
    for x in message.2 {
        //println!("{}, {:b}", x, byte);
        byte += bool_to_u8(x.1);
        bit_num += -1;
        if bit_num == 0 {
            bytes.push(byte);
            byte = 0;
            bit_num = 7;
        } else {
            byte = byte << 1;
        }
    }
    bytes.push(byte);
    return bytes;
}

//get data from the serial port
pub fn serial_get_data(
    _tui_log_tx: Sender<String>,
    _port: Box<dyn serialport::SerialPort>,
    _tx: Sender<Vec<u8>>,
) {
    /*loop {
        //let mut serial_buf = Vec::new();

        //port.read(serial_buf.as_mut_slice()).expect("Cant read serial data");
    }*/
}
