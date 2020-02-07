use gilrs::{Gilrs, Axis, Event};

fn main() {

	let mut gilrs = Gilrs::new().unwrap();

	// Iterate over all connected gamepads
	for (_id, gamepad) in gilrs.gamepads() {
		println!("{} is {:?}", gamepad.name(), gamepad.power_info());
	}

	let mut active_gamepad = None;


    while let Some(Event { id, event, time }) = gilrs.next_event() {
        println!("{:?} New event from {}: {:?}", time, id, event);
        active_gamepad = Some(id);
    }


	loop {
		while let Some( _ev ) = gilrs.next_event() {
		}
	
	

		// You can also use cached gamepad state
		if let Some(gamepad) = active_gamepad.map(|id| gilrs.gamepad(id)) {
			let left_axis = (gamepad.value(Axis::LeftStickX), 
				gamepad.value(Axis::LeftStickY));
			println!("{:?}", left_axis)
		}
	}
}


