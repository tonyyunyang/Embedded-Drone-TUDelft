use crate::{app::App, ui};
use protocol::format::{DeviceProtocol, HostProtocol};
use std::{error::Error, sync::mpsc::Receiver};
use tui::{backend::Backend, Terminal};

// pub fn run(
//     _tick_rate: Duration,
//     enhanced_graphics: bool,
//     user_input: Receiver<HostProtocol>,
//     device_data: Receiver<DeviceProtocol>,
// ) -> Result<(), Box<dyn Error>> {
//     // setup terminal
//     let stdout = io::stdout().into_raw_mode()?;
//     let stdout = MouseTerminal::from(stdout);
//     // let stdout = io::stdout().into_raw_mode()?.into_alternate_screen().unwrap();
//     // let stdout = AlternateScreen::from(stdout);
//     let backend = TermionBackend::new(stdout);
//     let mut terminal = Terminal::new(backend)?;

//     // create app and run it
//     let app = App::new(" Group 5 Drone Demo!!!", enhanced_graphics);
//     run_app(&mut terminal, app, user_input, device_data)?;

//     Ok(())
// }

pub(crate) fn run_app<B: Backend>(
    terminal: &mut Terminal<B>,
    mut app: App,
    // tick_rate: Duration,
    user_input: Receiver<HostProtocol>,
    device_data: Receiver<DeviceProtocol>,
) -> Result<(), Box<dyn Error>> {
    // let events = events(tick_rate);
    // terminal.draw(|f| ui::draw(f, &mut app))?;
    loop {
        terminal.draw(|f| ui::draw(f, &mut app))?;

        // match events.recv()? {
        //     Event::Input(key) => match key {
        //         Key::Char(' ')=> app.mode=1,
        //         Key::Char(c) => app.on_key(c),
        //         Key::Esc => app.mode = 1,
        //         Key::End => app.on_end(),
        //         _ => {}
        //     },
        // Event::Tick => app.on_tick(),
        // }
        // let x=user_input.recv();
        match user_input.try_recv() {
            Ok(a) => {
                app.p = a.get_p().into();
                app.p1 = a.get_p1().into();
                app.p2 = a.get_p2().into();
                app.yaw = a.get_yaw().into();
                app.pitch = a.get_pitch().into();
                app.roll = a.get_roll().into();
                app.lift = a.get_lift().into();
                app.mode = a.get_mode();
                terminal.clear().unwrap();
                app.on_tick();
            }
            Err(_e) => {
                // app.error = format!("Error: {}", e);
            }
        }
        match device_data.try_recv() {
            Ok(a) => {
                app.motor[0] = a.get_motor()[0];
                app.motor[1] = a.get_motor()[1];
                app.motor[2] = a.get_motor()[2];
                app.motor[3] = a.get_motor()[3];
                app.duration = a.get_duration();
                app.mode_sent = a.get_mode();
                app.ypr[0] = a.get_ypr()[0].to_num();
                app.ypr[1] = a.get_ypr()[1].to_num();
                app.ypr[2] = a.get_ypr()[2].to_num();
                app.ypr_filter[0] = a.get_ypr_filter()[0].to_num();
                app.ypr_filter[1] = a.get_ypr_filter()[1].to_num();
                app.ypr_filter[2] = a.get_ypr_filter()[2].to_num();
                app.acc = a.get_acc();
                app.batt = a.get_bat();
                app.pres = a.get_pres();
                app.crc = a.get_crc();
                app.ack = a.get_ack();
                app.on_tick();
                terminal.clear().unwrap();
            }
            Err(_e) => {
                // app.error= format!("Error: {}", e);
            }
        }
        if app.should_quit {
            return Ok(());
        }
    }
}

// enum Event {
//     // Input(Key),
//     Tick,
// }

// fn events(tick_rate: Duration) -> mpsc::Receiver<Event> {
//     let (tx, rx) = mpsc::channel();
//     // let keys_tx = tx.clone();
//     // thread::spawn(move || {
//     //     let stdin = io::stdin();
//     //     for key in stdin.keys().flatten() {
//     //         if let Err(err) = keys_tx.send(Event::Input(key)) {
//     //             eprintln!("{}", err);
//     //             return;
//     //         }
//     //     }
//     // });
//     thread::spawn(move || loop {
//         if let Err(err) = tx.send(Event::Tick) {
//             eprintln!("{}", err);
//             break;
//         }
//         thread::sleep(tick_rate);
//     });
//     rx
// }
