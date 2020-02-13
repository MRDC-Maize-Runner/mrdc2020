use gilrs::ev::{state::GamepadState, Button, Code};
use gilrs::{Axis, Event, Gilrs};

use std::sync::mpsc::{Sender, SyncSender};

//Function to run a game controller
pub fn controller_loop(
    tui_log_tx: Sender<String>,
    foward: Axis,
    turn: Axis,
    tx: SyncSender<(f32, f32, Vec<(&str, bool)>)>,
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
    let generic_buttons: [(&str, Button); 20] = [
        ("South", Button::South),
        ("East", Button::East),
        ("North", Button::North),
        ("West", Button::West),
        ("C", Button::C),
        ("Z", Button::Z),
        ("SLeftTrigger", Button::LeftTrigger),
        ("LeftTrigger2", Button::LeftTrigger2),
        ("RightTrigger", Button::RightTrigger),
        ("RightTrigger2", Button::RightTrigger2),
        ("Select", Button::Select),
        ("Start", Button::Start),
        ("Mode", Button::Mode),
        ("LeftThumb", Button::LeftThumb),
        ("RightThumb", Button::RightThumb),
        ("DPadUp", Button::DPadUp),
        ("DPadDown", Button::DPadDown),
        ("DPadLeft", Button::DPadLeft),
        ("DPadRight", Button::DPadRight),
        ("Unknown", Button::Unknown),
    ];
    let mut buttons: Vec<(&str, Code)> = Vec::with_capacity(25);

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
            for (s, b) in &generic_buttons {
                let code = gamepad.button_code(b.clone());
                if code.is_some() {
                    buttons.push((s, code.unwrap()));
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

            //get axis data
            let analog = (
                state.axis_data(fwdaxis.unwrap()),
                state.axis_data(trnaxis.unwrap()),
            );
            //get buttons data
            let mut buttons_state: Vec<(&str, bool)> = Vec::with_capacity(buttons.len());
            for (s, b) in buttons.clone() {
                buttons_state.push((s.clone(), state.is_pressed(b)));
            }
            //send it, this will hang until there is space open in the channel
            if analog.0.is_some() && analog.1.is_some() {
                tx.send((
                    analog.0.unwrap().value(),
                    analog.1.unwrap().value(),
                    buttons_state,
                ))
                .unwrap();
            }
        }

        gilrs.inc();
    }
}
