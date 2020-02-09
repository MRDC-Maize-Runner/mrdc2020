use gilrs::{Axis, Event, Gilrs};
use std::sync::mpsc::SyncSender;

//Function to run a game controller
pub fn controller_loop(foward: Axis, turn: Axis, tx: SyncSender<(f32, f32)>) {
    //initialize the library
    let mut gilrs = Gilrs::new().unwrap();
    // Iterate over all connected gamepads and print out stuff about them
    for (_id, gamepad) in gilrs.gamepads() {
        println!("{} is {:?}", gamepad.name(), gamepad.power_info());
    }

    //make a place to store the active gamepad
    let mut active_gamepad = None;

    //find the active gamepad by looking for an event
    while active_gamepad.is_none() {
        while let Some(Event { id, event, time }) = gilrs.next_event() {
            println!("{:?} New event from {}: {:?}", time, id, event);
            active_gamepad = Some(id);
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
            let analog = (gamepad.value(foward), gamepad.value(turn));
            tx.send(analog).unwrap();
        }
        gilrs.inc();
    }
}
