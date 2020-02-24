use gilrs::Axis;
use std::sync::mpsc::{channel, sync_channel};
use std::sync::mpsc::{Receiver, Sender, SyncSender};
use std::{env, thread, time};

use crate::controller::state;

mod controller;
mod serial;
mod tui;

fn main() {
    let args: Vec<String> = env::args().collect();
    //set loop timing for the main thread
    let loop_delay = time::Duration::from_millis(10);

    //set up tui
    let (tui_controller_tx, tui_controller_rx): (
        Sender<controller::state::State>,
        Receiver<controller::state::State>,
    ) = channel();
    let (tui_log_tx, tui_log_rx): (Sender<String>, Receiver<String>) = channel();
    let _tui_receive_thread = thread::spawn(move || tui::tui_setup(tui_controller_rx, tui_log_rx));
    let _ = tui_log_tx.send(String::from("Log Starting"));

    //set up controller
    let (controller_tx, controller_rx): (
        SyncSender<controller::state::State>,
        Receiver<controller::state::State>,
    ) = sync_channel(0);
    let controller_thread_logger = tui_log_tx.clone();
    let _controller_thread = thread::spawn(move || {
        controller::controller_loop(
            controller_thread_logger,
            Axis::LeftStickY,
            Axis::LeftStickX,
            controller_tx,
        );
    });

    //set up serial port
    let (serial_transmit_port, serial_receive_port) = serial::port_setup(&args[1], 115200, 10);
    let (serial_transmit_tx, serial_transmit_rx): (
        SyncSender<state::State>,
        Receiver<state::State>,
    ) = sync_channel(0);
    let (serial_receive_tx, _serial_receive_rx): (Sender<Vec<u8>>, Receiver<Vec<u8>>) = channel();
    let serial_transmit_thread_logger = tui_log_tx.clone();
    let _serial_transmit_thread = thread::spawn(move || {
        serial::serial_send_data(
            serial_transmit_thread_logger,
            serial_transmit_port,
            serial_transmit_rx,
        )
    });
    let serial_receive_thread_logger = tui_log_tx.clone();
    let _serial_receive_thread = thread::spawn(move || {
        serial::serial_get_data(
            serial_receive_thread_logger,
            serial_receive_port,
            serial_receive_tx,
        )
    });

    //main loop
    loop {
        //get data from the controller channel
        let controller_received = controller_rx.recv().expect("Not Crashing");
        let tui_send_controller = controller_received.clone();
        //send it to serial and tui
        let serial_transmit_return = serial_transmit_tx.send(controller_received);
        let tui_transmit_controller_transmit_return = tui_controller_tx.send(tui_send_controller);
        //error checking
        if serial_transmit_return.is_err() || tui_transmit_controller_transmit_return.is_err() {
            let _ = tui_log_tx.send(String::from("Channel Messaging Error"));
        }
        //set program speed
        thread::sleep(loop_delay);
    }

    //controller_thread.join().unwrap(); //this wont run
}
