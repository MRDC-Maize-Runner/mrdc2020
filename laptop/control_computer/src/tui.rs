use std::sync::mpsc::Receiver;
use std::{io::stdout, thread, time::Duration};

use crossterm::terminal::enable_raw_mode;

use tui::layout::{Constraint, Direction, Layout};
use tui::style::{Color, Style};
use tui::widgets::{Axis, Block, Borders, Chart, Dataset, Marker, Paragraph, Text, Widget};
use tui::{backend::CrosstermBackend, Terminal};

use crate::controller::state;

//setup tui and run thread
pub fn tui_setup(rx_controller: Receiver<state::State>, rx_log: Receiver<String>) {
    //set tui loop update delay
    let loop_delay = Duration::from_millis(100);

    //set up tui
    if enable_raw_mode().is_err() {
        println!("Error, Cant Start TUI");
    }

    let stdout = stdout();

    let backend = CrosstermBackend::new(stdout);

    let mut terminal = Terminal::new(backend).expect("Cant make tui");
    let _ = terminal.clear();
    let mut data: [(f64, f64); 1];

    //stuff to help in main loop
    let mut log: Vec<String> = Vec::new();

    loop {
        let mut controller_received: state::State = state::State::default();
        //make a place to store controller received stuff
        let controller_received_raw = rx_controller.try_iter();
        //get controller state
        for msg in controller_received_raw {
            controller_received = msg.clone();
        }
        //formate that data for display (graph, table)
        data = [(
            controller_received.forward as f64,
            controller_received.turn as f64,
        )];

        let mut buttons_str = String::from("Buttons: \n");
        for btn in controller_received.buttons {
            buttons_str.push_str(format!("{} \n", button_name(btn)).as_str());
        }
        //get log and add it to the log
        let log_msgs = rx_log.try_iter();
        for l in log_msgs {
            log.push(l);
        }

        //actually make the tui. I recommend minimizing this code block because it is long
        terminal
            .draw(|mut f| {
                //get size and drain log to fit
                let size = f.size();
                let height: u16 = (size.bottom() - size.top() - 4) / 2;
                if log.len() > height as usize {
                    log.drain(0..(log.len() - (height as usize)));
                }
                //make the tui
                let chunks = Layout::default()
                    .direction(Direction::Horizontal)
                    .constraints(
                        [
                            Constraint::Percentage(30),
                            Constraint::Percentage(20),
                            Constraint::Percentage(50),
                        ]
                        .as_ref(),
                    )
                    .split(f.size());
                Chart::default()
                    .block(
                        Block::default()
                            .title("Controller Position")
                            .title_style(Style::default())
                            .borders(Borders::ALL),
                    )
                    .x_axis(
                        Axis::default()
                            .title("X Axis")
                            .style(Style::default())
                            .labels_style(Style::default())
                            .bounds([-1.1, 1.1])
                            .labels(&["-1", "0", "1"]),
                    )
                    .y_axis(
                        Axis::default()
                            .title("Y Axis")
                            .style(Style::default())
                            .labels_style(Style::default())
                            .bounds([-1.1, 1.1])
                            .labels(&["-1", "0", "1"]),
                    )
                    .datasets(&[Dataset::default()
                        .name("Stick Position")
                        .marker(Marker::Dot)
                        .style(Style::default().fg(Color::Cyan))
                        .data(&data)])
                    .render(&mut f, chunks[0]);
                Paragraph::new([Text::raw(buttons_str)].iter())
                    .block(Block::default().title("Buttons").borders(Borders::ALL))
                    .style(Style::default())
                    .wrap(true)
                    .render(&mut f, chunks[1]);
                Paragraph::new([Text::raw(log.join("\n"))].iter())
                    .block(Block::default().title("Log").borders(Borders::ALL))
                    .style(Style::default())
                    .wrap(true)
                    .render(&mut f, chunks[2]);
            })
            .expect("terminal fail");
        //wait for loop delay
        thread::sleep(loop_delay);
    }
}

fn button_name(num: u32) -> Box<String> {
    let generic_buttons = [
        "South",
        "East",
        "North",
        "West",
        "C",
        "Z",
        "LeftTrigger",
        "LeftTrigger2",
        "RightTrigger",
        "RightTrigger2",
        "Select",
        "Start",
        "Mode",
        "LeftThumb",
        "RightThumb",
        "DPadUp",
        "DPadDown",
        "DPadLeft",
        "DPadRight",
        "Unknown",
    ];
    return Box::new(String::from(generic_buttons[num as usize]));
}
