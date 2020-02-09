use gilrs::Axis;
use std::sync::mpsc::{channel, sync_channel};
use std::sync::mpsc::{Receiver, Sender, SyncSender};
use std::thread;

use serialport::SerialPort;

mod controller;
mod serial;

fn main() {
    //make a channel
    let (controller_tx, controller_rx): (SyncSender<(f32, f32)>, Receiver<(f32, f32)>) =
        sync_channel(0);

    //spawn a controller thread
    let _controller_thread = thread::spawn(move || {
        controller::controller_loop(Axis::LeftStickX, Axis::LeftStickY, controller_tx);
    });

    let (serial_transmit_port, serial_receive_port) =
        serial::port_setup("/dev/ttyUSB0", 115200, 10);

    let (serial_transmit_tx, serial_transmit_rx): (
        Sender<(f32, f32)>,
        Receiver<(f32, f32)>,
    ) = channel();

    let (serial_receive_tx, serial_receive_rx): (Sender<Vec<u8>>, Receiver<Vec<u8>>) = channel();

    let _serial_transmit_thread =
        thread::spawn(move || serial::serial_send_data(serial_transmit_port, serial_transmit_rx));
    let _serial_receive_thread =
        thread::spawn(move || serial::serial_get_data(serial_receive_port, serial_receive_tx));
    //main loop
    loop {
        //get data from the channel
        let received = controller_rx.recv().unwrap();
        println!("{:?}", received);
        serial_transmit_tx.send(received);

        thread::sleep_ms(100); // sleep for 100ms
    }

    //controller_thread.join().unwrap(); //this wont run
}
