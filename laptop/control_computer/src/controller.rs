use gilrs::ev::{state::GamepadState, Button, Code};
use gilrs::{Axis, Event, Gilrs};

use std::sync::mpsc::{Sender, SyncSender};

pub mod state {
    include!(concat!(env!("OUT_DIR"), "/state.rs"));
}

//Function to run a game controller
pub fn controller_loop(
    tui_log_tx: Sender<String>,
    foward: Axis,
    turn: Axis,
    tx: SyncSender<state::State>,
) {
    //initialize the library
    let mut gilrs = Gilrs::new().unwrap();
    // Iterate over all connected gamepads and print out stuff about them
    for (_id, gamepad) in gilrs.gamepads() {
        tui_log_tx
            .send(format!("{} is {:?}", gamepad.name(), gamepad.power_info()))
            .unwrap();
    }

    //make a place to store the active gamepad
    let mut active_gamepad = None;
    let mut fwdaxis = None;
    let mut trnaxis = None;

    //set up button bindings
    let generic_buttons: [(u32, Button); 20] = [
        (0, Button::South),
        (1, Button::East),
        (2, Button::North),
        (3, Button::West),
        (4, Button::C),
        (5, Button::Z),
        (6, Button::LeftTrigger),
        (7, Button::LeftTrigger2),
        (8, Button::RightTrigger),
        (9, Button::RightTrigger2),
        (10, Button::Select),
        (11, Button::Start),
        (12, Button::Mode),
        (13, Button::LeftThumb),
        (14, Button::RightThumb),
        (15, Button::DPadUp),
        (16, Button::DPadDown),
        (17, Button::DPadLeft),
        (18, Button::DPadRight),
        (19, Button::Unknown),
    ];
    let mut buttons: Vec<(&u32, Code)> = Vec::with_capacity(25);

    //find the active gamepad by looking for an event and setup the axis and buttons
    while active_gamepad.is_none() || fwdaxis.is_none() || trnaxis.is_none() {
        //wait for an event and set up gamepad when one is found
        while let Some(Event { id, event, time }) = gilrs.next_event() {
            let _ = tui_log_tx.send(format!("{:?} New event from {}: {:?}", time, id, event));
            active_gamepad = Some(id);

            //gamepad found, set up axis
            let gamepad = gilrs.gamepad(active_gamepad.unwrap());
            fwdaxis = gamepad.axis_code(foward);
            trnaxis = gamepad.axis_code(turn);

            //set up buttons
            buttons.drain(0..buttons.len());
            for (n, b) in &generic_buttons {
                let code = gamepad.button_code(b.clone());
                if code.is_some() {
                    buttons.push((n, code.unwrap()));
                }
            }
        }
    }
    loop {
        /* Get the latest data from the gamepad. This is the only way i could figure out how to do this.
        It basically just goes through all the controller events until nothing more in the buffer.Receiver
        This is what we want because we want the current state of the joystick not a past state
        I though about making this event based but then i would have to store the last state of everything. */
        while let Some(_ev) = gilrs.next_event() {}

        //Send the current state of the gamepad over the channel.
        if let Some(gamepad) = active_gamepad.map(|id| gilrs.gamepad(id)) {
            let state: &GamepadState = gamepad.state();

            let mut current_state = state::State::default();

            current_state.forward = state.axis_data(fwdaxis.unwrap()).unwrap().value();
            current_state.turn = state.axis_data(trnaxis.unwrap()).unwrap().value();

            //get buttons data
            let mut buttons_pressed: Vec<u32> = Vec::with_capacity(buttons.len());
            for (n, b) in buttons.clone() {
                if state.is_pressed(b) {
                    buttons_pressed.push(n+0);
                }
            }
            current_state.buttons = buttons_pressed;
            //send it, this will hang until there is space open in the channel
            tx.send(current_state);
        }

        gilrs.inc();
    }
}
