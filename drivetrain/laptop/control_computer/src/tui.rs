use std::sync::mpsc::Receiver;
use std::{io::stdout, thread, time::Duration};

use crossterm::terminal::enable_raw_mode;

use tui::layout::{Constraint, Direction, Layout};
use tui::style::{Color, Style};
use tui::widgets::{
    Axis, Block, Borders, Chart, Dataset, Marker, Paragraph, Row, Table, Text, Widget,
};
use tui::{backend::CrosstermBackend, Terminal};

//setup tui and run thread
pub fn tui_setup(rx_controller: Receiver<(f32, f32, Vec<(&str, bool)>)>, rx_log: Receiver<String>) {
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
    let log_block = Block::default().title("Log").borders(Borders::ALL);

    loop {
        //make a place to store controller received stuff
        let mut controller_received: (f32, f32, Vec<(&str, bool)>) =
            (0.0, 0.0, vec![("debug", true)]);
        let controller_received_raw = rx_controller.try_iter();
        //get controller state
        for msg in controller_received_raw {
            controller_received = msg;
        }
        //formate that data for display (graph, table)
        data = [(controller_received.0 as f64, controller_received.1 as f64)];
        let mut rows: Vec<(String, String)> = Vec::with_capacity(controller_received.2.len());
        let mut row: Vec<[String; 2]> = Vec::new();
        for r in controller_received.2 {
            rows.push((String::from(r.0), format!("{}", r.1)));
        }
        for (s, v) in rows {
            row.push([s, v]);
        }
        let rows_premade = row.iter().map(|r| Row::Data(r.into_iter()));

        //get log and add it to the log
        let log_msgs = rx_log.try_iter();
        for l in log_msgs {
            log.push(l);
        }

        //actually make the tui. I recommend minimizing this code block because it is long
        terminal.draw(|mut f| {
                //get size and drain log to fit
                let size = f.size();
                let height: u16 = (size.bottom() - size.top() - 4) / 2;
                if log.len() > height as usize {
                    log.drain(0..(log.len() - (height as usize)));
                }
                //make the tui
                let chunks = Layout::default()
                    .direction(Direction::Vertical)
                    .margin(1)
                    .constraints([Constraint::Percentage(50), Constraint::Percentage(50)].as_ref())
                    .split(f.size());
                {
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
                        .split(chunks[0]);
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
                    Table::new(["Button", "Pressed"].iter(), rows_premade.map(|r| r))
                        .block(Block::default().title("Table"))
                        .header_style(Style::default().fg(Color::Yellow))
                        .widths(&[Constraint::Length(15), Constraint::Length(15)])
                        .style(Style::default().fg(Color::White))
                        .column_spacing(1)
                        .render(&mut f, chunks[1]);
                    Paragraph::new([Text::raw(log.join("\n"))].iter())
                        .block(log_block)
                        .style(Style::default())
                        .wrap(true)
                        .render(&mut f, chunks[2]);
                }
                Block::default()
                    .title("main block")
                    .borders(Borders::ALL)
                    .render(&mut f, chunks[1]);
            })
            .expect("terminal fail");
        //wait for loop delay
        thread::sleep(loop_delay)
    }
}
