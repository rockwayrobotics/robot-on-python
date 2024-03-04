// Based on the ds crate's "simple" example.
#![allow(unused_imports)]
// use log::*;

use std::sync::Arc;
use std::io;
use std::thread;
use std::sync::{mpsc::{self, Sender, Receiver}};
// use std::time::Duration;

use crossterm::{
    event::{self, Event, KeyCode},
    terminal::{disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen},
    ExecutableCommand,
};
use ratatui::{prelude::*, widgets::*};

use ds::*;

mod joystick;
// mod model;
// mod ui;

use crate::{
    // model::Model,
    // ui::Ui,
    joystick::{JoystickData, Joystick},
};

const POLL_PERIOD: u64 = 20;

struct Model {
    team: u32,
    count: u32,
    enabled: bool,
    mode: Mode,
    battery: f32,
    started: bool,

    cmd_mode: bool,
    input: String,

    cycle: u32,
    stick: Option<JoystickData>,
}

impl Model {
    fn new(team: u32) -> Self {
        Self {
            team,
            count: 0,
            enabled: false,
            mode: Mode::Autonomous,
            battery: 0.0,
            started: false,

            cmd_mode: false,
            input: "".into(),

            cycle: 0,
            stick: None,
        }
    }
}

enum LogMsg {
    // Debug(String),
    Stdout(Stdout),
}

// enum Action {
//     Quit,
//     Enable,
//     Disable,
//     Drive(f32),
//     // Turn(f32),
// }

struct Controller<T: ratatui::backend::Backend> {
    terminal: Terminal<T>,
    ds: DriverStation,
    model: Model,
    rx: Receiver<LogMsg>,
    joystick: Arc<Joystick>,
}

impl<T: ratatui::backend::Backend> Controller<T> {
    fn new(team: u32, terminal: Terminal<T>) -> Self {
        let (tx, rx) = mpsc::channel();

        let joystick = Arc::new(Joystick::new());

        let ds = Self::make_ds(team, tx, Arc::clone(&joystick));

        Self {
            terminal,
            ds,
            model: Model::new(team),
            rx,
            joystick,
        }
    }

    fn make_ds(team: u32, tx: Sender<LogMsg>, joy: Arc<Joystick>) -> DriverStation {
        let alliance = Alliance::new_red(1); // position 1-3
        let mut ds = DriverStation::new_team(team, alliance);

        ds.set_tcp_consumer(move |pkt| {
            match pkt {
                TcpPacket::Stdout(s) => {
                    let _ = tx.send(LogMsg::Stdout(s));
                }

                _ => {}
            }
        });

        ds.set_joystick_supplier(move || joy.get_report());
        // ds.queue_tcp(TcpTag::JoystickDesc(JoystickDesc));

        ds
    }

    fn handle_stdout(&mut self) {
        while let Ok(msg) = self.rx.try_recv() {
            match msg {
                LogMsg::Stdout(stdout) => {
                    // timestamp: f32,
                    // message: String,
                    // seqnum: u16,
                    let _ = self.terminal.insert_before(1, |buf| {
                        Paragraph::new(Span::raw(stdout.message))
                            .render(buf.area, buf);
                    });
                }
            }
        }
    }

    fn handle_cmd(&mut self) {
        let input = self.model.input.to_owned();
        self.model.input.clear();
        self.model.cmd_mode = false;

        match input.as_str() {
            "enable" => self.ds.enable(),
            "disable" => self.ds.disable(),

            "auto" => {
                self.ds.set_mode(Mode::Autonomous);
                // self.log(format!("Request: auto mode"));
            }

            "tele" | "teleop" => {
                self.ds.set_mode(Mode::Teleoperated);
                // self.log(format!("Request: teleop mode"));
            }

            "test" => {
                self.ds.set_mode(Mode::Test);
                // self.log(format!("Request: test mode"));
            }

            "restart" => {
                self.ds.restart_code();
                // self.log(format!("Request: restart code"));
            }

            "reboot" => {
                self.ds.restart_roborio();
                // self.log(format!("Request: reboot RIO"));
            }

            "joy" => {
                self.ds.queue_tcp(TcpTag::JoystickDesc(JoystickDesc));
            }

            _ => {
                self.log(format!("unrecognized cmd {}", input));
                return;
            }
        }

        self.log(format!("cmd: {}", input));
    }

    fn log(&mut self, text: String) {
        let _ = self.terminal.insert_before(1, |buf| {
            Line::from(Span::raw(text))
                .render(buf.area, buf);
        });
    }

    fn handle_events(&mut self) -> io::Result<bool> {
        if event::poll(std::time::Duration::from_millis(POLL_PERIOD))? {
            if let Event::Key(key) = event::read()? {
                if key.kind == event::KeyEventKind::Press {
                    let model = &mut self.model;
                    if model.cmd_mode {
                        match key.code {
                            KeyCode::Char(c) => {
                                if model.input.len() < 50 {
                                    model.input.push(c);
                                }
                            }

                            KeyCode::Backspace => {
                                let _ = model.input.pop();
                            }

                            KeyCode::Esc => {
                                model.input.clear();
                                model.cmd_mode = false;
                            }

                            KeyCode::Enter => {
                                model.cmd_mode = false;
                            }

                            KeyCode::Home
                            | KeyCode::Left
                            | KeyCode::Right
                            | KeyCode::Up
                            | KeyCode::Down
                            | KeyCode::End
                            | KeyCode::PageUp
                            | KeyCode::PageDown
                            | KeyCode::Tab
                            | KeyCode::BackTab
                            | KeyCode::Delete
                            | KeyCode::Insert
                            | KeyCode::F(_)
                            | KeyCode::Null
                            | KeyCode::CapsLock
                            | KeyCode::ScrollLock
                            | KeyCode::NumLock
                            | KeyCode::PrintScreen
                            | KeyCode::Pause
                            | KeyCode::Menu
                            | KeyCode::KeypadBegin
                            | KeyCode::Media(_)
                            | KeyCode::Modifier(_)
                                => {}
                        }
                    }
                    else {
                        match key.code {
                            KeyCode::Char('q') => return Ok(true),

                            KeyCode::Char('!') => {
                                model.input.clear();
                                model.cmd_mode = true;
                            }

                            KeyCode::Char(' ') |
                            KeyCode::Char('-') => {
                                self.ds.disable();
                            }

                            KeyCode::Enter |
                            KeyCode::Char('+') => {
                                self.ds.enable();
                            }

                            _ => {}
                        }
                    }
                }
            }
        }

        Ok(false)
    }

    fn render(model: &Model, frame: &mut Frame) {
        let size = frame.size();
        let area = Layout::default()
            .direction(Direction::Horizontal)
            .constraints(vec![
                Constraint::Length(10),
                Constraint::Length(30),
                Constraint::Length(30),
                Constraint::Min(0),
                ])
            // .margin(1)  // does not eat into the regions
            .split(size);

        //-----------------------------
        // First area: compact live status

        let text = format!("{}", match model.cycle % 4 {
            0 => "/", 1 => "-", 2 => "\\", 3 => "|", _ => unreachable!(),
        });

        frame.render_widget(
            Paragraph::new(text)
                .block(Block::default()
                    .title(format!("{}", model.team))
                    .borders(Borders::ALL)
                ),
            area[0],
        );

        //-----------------------------
        // Second area, top: general

        let row = Layout::default()
            .constraints(vec![
                Constraint::Min(7),
                Constraint::Length(3),  // cmd input
            ])
            // .margin(1)  // does not eat into the regions
            .split(area[1]);

        let mode = match (model.enabled, model.mode) {
            (false, _) => "Disabled".white(),
            (true, Mode::Autonomous) => "Autonomous".magenta(),
            (true, Mode::Teleoperated) => "Teleop".green(),
            (true, Mode::Test) => "Test".red(),
        };
        let batt = format!("{:.3}V", model.battery);
        let state = if model.started { "started"} else { "not ready" };

        frame.render_widget(
            Paragraph::new(vec![
                Line::from(mode),
                Line::from(batt),
                Line::from(state),
                ])
                .block(Block::default()
                    .title(format!("Team {}", model.team))
                    .borders(Borders::ALL)),
            row[0],
        );

        //-----------------------------
        // Third area: joystick data
        if let Some(stick) = &model.stick {
            let buttons = stick.buttons.reverse_bits();
            frame.render_widget(
                Paragraph::new(vec![
                    Line::from(format!("#{}", stick.count)),
                    Line::from(format!("Buttons: {:04b} {:04b} {:04b}",
                        (buttons >> 12) & 0x0f,
                        (buttons >> 8) & 0x0f,
                        (buttons >> 4) & 0x0f,
                    )),
                    Line::from(format!("POV: {}", match stick.pov {
                        8 => format!("---"),
                        x => format!("{:3}", x * 45),
                    })),
                    Line::from(format!("0 X Axis: {:+5.3}", stick.axes[0] as f32 / 128.0)),
                    Line::from(format!("1 Y Axis: {:+5.3}", stick.axes[1] as f32 / 128.0)),
                    Line::from(format!("2 Z Axis: {:+5.3}", stick.axes[2] as f32 / 128.0)),
                    Line::from(format!("3 X Rot.: {:+5.3}", stick.axes[3] as f32 / 128.0)),
                    Line::from(format!("4 Y Rot.: {:+5.3}", stick.axes[4] as f32 / 128.0)),
                    Line::from(format!("5 Z Rot.: {:+5.3}", stick.axes[5] as f32 / 128.0)),
                    ])
                    .block(Block::default()
                        .title("Joystick")
                        .borders(Borders::ALL)),
                area[2],
            );
        }

        //-----------------------------
        // Command input area

        if model.cmd_mode {
            // let text = Paragraph::new(
            //     Span::raw(&model.input)
            // ).block(Block::default().borders(Borders::ALL).title("Input"));

            let item = Paragraph::new(Line::from(
                vec![
                    Span::from("> "),
                    model.input.clone().bold(),
                ]));

            frame.render_widget(item, row[1]);
        }
    }

    fn _run(&mut self) -> io::Result<()> {
        // info!("Starting: team {}", self.team);
        // thread::sleep(Duration::from_millis(1000));
        // ds.restart_code();

        // debug!("mode {:?}", self.model.mode);

        const MIN_BATT_DIFF: f32 = 0.050; // volts
        let mut count = 0;
        let mut last_joy = 0;
        loop {
            if count % (50*5) == 0 {
                let v = self.ds.trace().is_code_started();
                if self.model.started != v {
                    self.model.started = v;
                    // debug!("#{} started={}", count, self.model.started);
                    if self.model.enabled && !v {
                        self.ds.disable();
                    }
                    self.ds.queue_tcp(TcpTag::JoystickDesc(JoystickDesc));
                    self.log(format!("send joystick desc"));
                }

                let v = self.ds.battery_voltage();
                if (self.model.battery - v).abs() >= MIN_BATT_DIFF {
                    self.model.battery = v;
                    self.log(format!("batt {:.3}", v));
                }

                let mode = self.ds.mode();
                if self.model.mode != mode {
                    self.log(format!("mode {:?} -> {:?}", self.model.mode, mode));
                    self.model.mode = mode;
                }
            }

            if count % 50 == 1 {
                self.model.enabled = self.ds.enabled();
            }

            self.model.count = count;

            self.handle_stdout();

            let joy_count = self.joystick.count();
            if joy_count != last_joy {
                last_joy = joy_count;
                self.model.stick = Some(self.joystick.get_data());
            }

            // If we're not in command mode but have input, that means
            // we should execute the command.
            if !self.model.cmd_mode && self.model.input.len() > 0 {
                self.handle_cmd();
            }

            self.model.cycle += 1;
            self.terminal.draw(|frame| Self::render(&self.model, frame))?;
            if self.handle_events()? {
                break;
            }

            count += 1;
        }

        Ok(())
    }

    fn run(&mut self) {
        let stick = Arc::clone(&self.joystick);
        thread::scope(|s| {
            s.spawn(move || stick.run());

            let _ = self._run();

            (*self.joystick).stop();
        });
    }
}


pub fn initialize_panic_handler() {
    let original_hook = std::panic::take_hook();
    std::panic::set_hook(Box::new(move |panic_info| {
        crossterm::execute!(std::io::stderr(), crossterm::terminal::LeaveAlternateScreen).unwrap();
        crossterm::terminal::disable_raw_mode().unwrap();
        original_hook(panic_info);
    }));
}

fn main() -> io::Result<()> {
    env_logger::init();

    initialize_panic_handler();

    enable_raw_mode()?;

    let mut stdout = io::stdout();
    stdout.execute(EnterAlternateScreen)?;

    let terminal = Terminal::with_options(
        CrosstermBackend::new(stdout),
        TerminalOptions {
            viewport: Viewport::Inline(14),
        },
    )?;

    let mut ctrl = Controller::new(8089, terminal);
    ctrl.run();

    disable_raw_mode()?;
    io::stdout().execute(LeaveAlternateScreen)?;
    Ok(())
}

