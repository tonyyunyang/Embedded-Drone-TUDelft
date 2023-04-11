use crate::app::App;
use tui::{
    backend::Backend,
    layout::{Constraint, Direction, Layout, Rect},
    style::{Color, Modifier, Style},
    symbols::{self},
    text::{Span, Spans},
    widgets::{
        canvas::{Canvas, Line, Rectangle},
        Axis, Chart, Dataset,
    },
    widgets::{Block, Borders, Gauge, Paragraph, Row, Table, Wrap},
    Frame,
};

pub fn draw<B: Backend>(f: &mut Frame<B>, app: &mut App) {
    draw_second_tab(f, app, f.size());
}

fn draw_serial<B>(f: &mut Frame<B>, app: &mut App, area: Rect)
where
    B: Backend,
{
    let block = Block::default().borders(Borders::ALL).title("Drone Data");
    let str_duration = app.duration.to_string();
    let text = vec![
        Spans::from(vec![
            Span::styled(
                "DTT:",
                Style::default().fg(Color::Red).add_modifier(Modifier::BOLD),
            ),
            Span::from("\r"),
            Span::from(str_duration),
            Span::styled(
                "ms\r",
                Style::default().fg(Color::Red).add_modifier(Modifier::BOLD),
            ),
        ]),
        Spans::from(vec![
            Span::styled(
                "MODE:",
                Style::default()
                    .fg(Color::LightBlue)
                    .add_modifier(Modifier::BOLD),
            ),
            Span::from(match_mode_to_string(app.mode_sent)),
        ]),
        Spans::from(vec![
            Span::styled(
                "MTR:",
                Style::default()
                    .fg(Color::LightGreen)
                    .add_modifier(Modifier::BOLD),
            ),
            Span::from("\r"),
            Span::from(app.motor[0].to_string()),
            Span::from(" "),
            Span::from(app.motor[1].to_string()),
            Span::from(" "),
            Span::from(app.motor[2].to_string()),
            Span::from(" "),
            Span::from(app.motor[3].to_string()),
        ]),
        Spans::from(vec![
            Span::styled(
                "YPR:",
                Style::default()
                    .fg(Color::LightMagenta)
                    .add_modifier(Modifier::BOLD),
            ),
            Span::from("\r"),
            Span::from(app.ypr[0].to_string()),
            Span::from(" "),
            Span::from(app.ypr[1].to_string()),
            Span::from(" "),
            Span::from(app.ypr[2].to_string()),
        ]),
        Spans::from(vec![
            Span::styled(
                "YPR_Filter:",
                Style::default()
                    .fg(Color::LightRed)
                    .add_modifier(Modifier::BOLD),
            ),
            Span::from("\r"),
            Span::from(app.ypr_filter[0].to_string()),
            Span::from(" "),
            Span::from(app.ypr_filter[1].to_string()),
            Span::from(" "),
            Span::from(app.ypr_filter[2].to_string()),
        ]),
        Spans::from(vec![
            Span::styled(
                "ACC:",
                Style::default()
                    .fg(Color::Green)
                    .add_modifier(Modifier::BOLD),
            ),
            Span::from("\r"),
            Span::from(app.acc[0].to_string()),
            Span::from(" "),
            Span::from(app.acc[1].to_string()),
            Span::from(" "),
            Span::from(app.acc[2].to_string()),
        ]),
        Spans::from(vec![
            Span::styled(
                "BAT",
                Style::default()
                    .fg(Color::Blue)
                    .add_modifier(Modifier::BOLD),
            ),
            Span::from("\r"),
            Span::from(app.batt.to_string()),
        ]),
        Spans::from(vec![
            Span::styled(
                "BAR:",
                Style::default()
                    .fg(Color::Yellow)
                    .add_modifier(Modifier::BOLD),
            ),
            Span::from("\r"),
            Span::from(app.pres.to_string()),
        ]),
        Spans::from(vec![Span::from("CRC: \r"), Span::from(app.crc.to_string())]),
        Spans::from(vec![
            Span::from("ACK: \r"),
            Span::from(match_corres_ack(app.ack)),
        ]),
    ];
    let paragraph = Paragraph::new(text).block(block).wrap(Wrap { trim: false });
    f.render_widget(paragraph, area);
}

fn match_corres_ack(ack: u8) -> String {
    match ack {
        0b0000_0000 => String::from("NACK"),
        0b1111_1111 => String::from("ACK"),
        0b1111_0000 => String::from("NOT NEUTRAL STATE"),
        0b0000_1111 => String::from("TRANSITION DECLINED"),
        0b0011_1100 => String::from("TRANSITION ALLOWED"),
        0b0000_0001 => String::from("REMAINING SAME MODE"),
        0b0000_0010 => String::from("TO SAFE FROM PANIC"),
        _ => String::from("   "),
    }
}

fn match_mode_to_string(mode: u8) -> String {
    match mode {
        0 => String::from("Safe"),
        1 => String::from("Panic"),
        2 => String::from("Manual"),
        3 => String::from("Calibration"),
        4 => String::from("Yaw Control"),
        5 => String::from("Full Control"),
        6 => String::from("Raw Control"),
        7 => String::from("Height Control"),
        8 => String::from("Wireless"),
        _ => String::from("Not Defined"),
    }
}
fn draw_gauges<B>(f: &mut Frame<B>, app: &mut App, area: Rect)
where
    B: Backend,
{
    let chunks = Layout::default()
        .constraints(
            [
                Constraint::Min(4),
                Constraint::Min(4),
                Constraint::Min(4),
                Constraint::Min(4),
                Constraint::Min(4),
            ]
            .as_ref(),
        )
        .margin(1)
        .split(area);
    let block = Block::default().borders(Borders::ALL).title("Motor Speeds");
    f.render_widget(block, area);
    let mut max_motor_speed = 10;
    if app.mode_sent == 2 {
        max_motor_speed = 4;
    } else if app.mode_sent == 4 {
        max_motor_speed = 6;
    } else if app.mode_sent == 5 || app.mode_sent == 6 || app.mode_sent == 7 {
        max_motor_speed = 10;
    }
    let gauge = Gauge::default()
        .block(Block::default().title("Motor1 Speed:"))
        .gauge_style(
            Style::default()
                .fg(Color::LightBlue)
                .bg(Color::Black)
                .add_modifier(Modifier::ITALIC | Modifier::BOLD),
        )
        .percent(app.motor[0].saturating_div(max_motor_speed));
    f.render_widget(gauge, chunks[0]);
    let gauge1 = Gauge::default()
        .block(Block::default().title("Motor2 Speed:"))
        .gauge_style(
            Style::default()
                .fg(Color::LightBlue)
                .bg(Color::Black)
                .add_modifier(Modifier::ITALIC | Modifier::BOLD),
        )
        .percent(app.motor[1].saturating_div(max_motor_speed));
    f.render_widget(gauge1, chunks[1]);
    let gauge2 = Gauge::default()
        .block(Block::default().title("Motor3 Speed:"))
        .gauge_style(
            Style::default()
                .fg(Color::LightBlue)
                .bg(Color::Black)
                .add_modifier(Modifier::ITALIC | Modifier::BOLD),
        )
        .percent(app.motor[2].saturating_div(max_motor_speed));
    f.render_widget(gauge2, chunks[2]);
    let gauge3 = Gauge::default()
        .block(Block::default().title("Motor4 Speed:"))
        .gauge_style(
            Style::default()
                .fg(Color::LightBlue)
                .bg(Color::Black)
                .add_modifier(Modifier::ITALIC | Modifier::BOLD),
        )
        .percent(app.motor[3].saturating_div(max_motor_speed));
    f.render_widget(gauge3, chunks[3]);
    let gauge3 = Gauge::default()
        .block(Block::default().title("Battery Level"))
        .gauge_style(
            Style::default()
                .fg(Color::LightRed)
                .bg(Color::White)
                .add_modifier(Modifier::BOLD),
        )
        // TODO: MIght need to change this value
        .percent(app.batt.saturating_div(10));
    f.render_widget(gauge3, chunks[4]);
}

fn draw_text<B>(f: &mut Frame<B>, area: Rect)
where
    B: Backend,
{
    let text = vec![
        Spans::from("Press"),
        Spans::from(vec![
            Span::styled(
                "ESC, SPACE BAR ",
                Style::default().fg(Color::Red).add_modifier(Modifier::BOLD),
            ),
            Span::from("go to safe mode through panic mode"),
        ]),
        Spans::from(vec![
            Span::styled(
                "0,1,2,3,4,5,6,7,8,9",
                Style::default().fg(Color::Red).add_modifier(Modifier::BOLD),
            ),
            Span::raw(" for different modes."),
        ]),
        Spans::from(vec![
            Span::styled(
                "a/z",
                Style::default().fg(Color::Red).add_modifier(Modifier::BOLD),
            ),
            Span::raw(" for lift up/down."),
        ]),
        Spans::from(vec![
            Span::styled(
                "left/right",
                Style::default().fg(Color::Red).add_modifier(Modifier::BOLD),
            ),
            Span::raw(" for roll up/down."),
        ]),
        Spans::from(vec![
            Span::styled(
                "up/down",
                Style::default().fg(Color::Red).add_modifier(Modifier::BOLD),
            ),
            Span::raw(" for pitch up/down."),
        ]),
        Spans::from(vec![
            Span::styled(
                "q/w",
                Style::default().fg(Color::Red).add_modifier(Modifier::BOLD),
            ),
            Span::raw(" for yaw up/down."),
        ]),
        Spans::from(vec![
            Span::styled(
                "u/j",
                Style::default().fg(Color::Red).add_modifier(Modifier::BOLD),
            ),
            Span::raw(" for P increase/decrease."),
        ]),
        Spans::from(vec![
            Span::styled(
                "i/k",
                Style::default().fg(Color::Red).add_modifier(Modifier::BOLD),
            ),
            Span::raw(" for P1 increase/decrease."),
        ]),
        Spans::from(vec![
            Span::styled(
                "o/l",
                Style::default().fg(Color::Red).add_modifier(Modifier::BOLD),
            ),
            Span::raw(" for P2 increase/decrease."),
        ]),
    ];
    let block = Block::default()
        .borders(Borders::RIGHT | Borders::LEFT | Borders::TOP)
        .title(Span::styled(
            "Controls",
            Style::default()
                .fg(Color::Magenta)
                .add_modifier(Modifier::BOLD),
        ));
    let paragraph = Paragraph::new(text).block(block).wrap(Wrap { trim: false });
    f.render_widget(paragraph, area);
}
fn draw_input_values<B>(f: &mut Frame<B>, app: &mut App, area: Rect)
where
    B: Backend,
{
    let up_style = Style::default().fg(Color::Green);
    let str_pitch;
    let str_roll;
    let str_yaw;
    let mut str_lift = app.lift.to_string();
    let str_p = app.p.to_string();
    let str_p1 = app.p1.to_string();
    let str_p2 = app.p2.to_string();
    let mut str_mode = app.mode_sent.to_string();
    match app.mode {
        0 => {
            str_mode = "Safe".to_string();
        }
        1 => {
            str_mode = "Safe through Panic".to_string();
        }
        2 => {
            str_mode = "Manual".to_string();
        }
        3 => {
            str_mode = "Calibration".to_string();
        }
        4 => {
            str_mode = "Yaw Control".to_string();
        }
        5 => {
            str_mode = "Roll/Pitch Control".to_string();
        }
        6 => {
            str_mode = "Raw Sensor Reading".to_string();
        }
        7 => {
            str_mode = "Height Control".to_string();
        }
        8 => {
            str_mode = "ᛒᛒᛒ Wireless Control..not implemented :(".to_string();
        }
        0b1111_1110 => {
            app.should_quit = true;
        }
        _ => {}
    }

    // if app.pitch <= 90.0 && app.pitch >= 85.0 {
    //     str_pitch = "1.0".to_string();
    //     //  app.pitch = 1.0;
    // } else if app.pitch <= 85.0 && app.pitch > 80.0 {
    //     str_pitch = "0.875".to_string();
    //     //  app.pitch =0.875;
    // } else if app.pitch <= 80.0 && app.pitch > 75.0 {
    //     str_pitch = "0.75".to_string();
    // //    app.pitch = 0.75;
    // } else if app.pitch <= 75.0 && app.pitch > 70.0 {
    //     str_pitch = "0.625".to_string();
    //     // app.pitch = 0.625;
    // } else if app.pitch <= 70.0 && app.pitch > 65.0 {
    //     str_pitch = "0.5".to_string();
    //     // app.pitch = 0.5;
    // } else if app.pitch <= 65.0 && app.pitch > 60.0 {
    //     str_pitch = "0.375".to_string();
    //     // app.pitch =0.375;
    // } else if app.pitch <= 60.0 && app.pitch > 55.0 {
    //     str_pitch = "0.25".to_string();
    //     // app.pitch =0.25;
    // } else if app.pitch <= 55.0 && app.pitch > 50.0 {
    //     str_pitch = "0.125".to_string();
    //     // app.pitch =0.125;
    // } else if app.pitch <= 50.0 && app.pitch > 45.0 {
    //     str_pitch = "0.0".to_string();
    //     // app.pitch =0.0;
    // } else if app.pitch <= 45.0 && app.pitch > 40.0 {
    //     str_pitch = "0.125".to_string();
    //     // app.pitch =-0.125;
    // } else if app.pitch <= 40.0 && app.pitch > 35.0 {
    //     str_pitch = "-0.25".to_string();
    //     // app.pitch = -0.25;
    // } else if app.pitch <= 35.0 && app.pitch > 30.0 {
    //     str_pitch = "-0.375".to_string();
    //     // app.pitch =-0.375;
    // } else if app.pitch <= 30.0 && app.pitch > 25.0 {
    //     str_pitch = "-0.5".to_string();
    //     // app.pitch = -0.5;
    // } else if app.pitch <= 25.0 && app.pitch > 20.0 {
    //     str_pitch = "-0.625".to_string();
    //     // app.pitch =-0.625;
    // } else if app.pitch <= 20.0 && app.pitch > 15.0 {
    //     str_pitch = "-0.75".to_string();
    //     // app.pitch =-0.75;
    // } else if app.pitch <= 15.0 && app.pitch > 10.0 {
    //     str_pitch = "-0.875".to_string();
    //     // app.pitch =-0.875;
    // } else if app.pitch <= 10.0 {
    //     str_pitch = "-1.0".to_string();
    //     // app.pitch =-1.0
    // }

    // if app.yaw <= 90.0 && app.yaw >= 85.0 {
    //     str_yaw = "1.0".to_string();
    //     //  app.pitch = 1.0;
    // } else if app.yaw <= 85.0 && app.yaw > 80.0 {
    //     str_yaw = "0.875".to_string();
    //     //  app.yaw =0.875;
    // } else if app.yaw <= 80.0 && app.yaw > 75.0 {
    //     str_yaw = "0.75".to_string();
    // //    app.yaw = 0.75;
    // } else if app.yaw <= 75.0 && app.yaw > 70.0 {
    //     str_yaw = "0.625".to_string();
    //     // app.yaw = 0.625;
    // } else if app.yaw <= 70.0 && app.yaw > 65.0 {
    //     str_yaw = "0.5".to_string();
    //     // app.yaw = 0.5;
    // } else if app.yaw <= 65.0 && app.yaw > 60.0 {
    //     str_yaw = "0.375".to_string();
    //     // app.yaw =0.375;
    // } else if app.yaw <= 60.0 && app.yaw > 55.0 {
    //     str_yaw = "0.25".to_string();
    //     // app.yaw =0.25;
    // } else if app.yaw <= 55.0 && app.yaw > 50.0 {
    //     str_yaw = "0.125".to_string();
    //     // app.yaw =0.125;
    // } else if app.yaw <= 50.0 && app.yaw > 45.0 {
    //     str_yaw = "0.0".to_string();
    //     // app.yaw =0.0;
    // } else if app.yaw <= 45.0 && app.yaw > 40.0 {
    //     str_yaw = "0.125".to_string();
    //     // app.yaw =-0.125;
    // } else if app.yaw <= 40.0 && app.yaw > 35.0 {
    //     str_yaw = "-0.25".to_string();
    //     // app.yaw = -0.25;
    // } else if app.yaw <= 35.0 && app.yaw > 30.0 {
    //     str_yaw = "-0.375".to_string();
    //     // app.yaw =-0.375;
    // } else if app.yaw <= 30.0 && app.yaw > 25.0 {
    //     str_yaw = "-0.5".to_string();
    //     // app.yaw = -0.5;
    // } else if app.yaw <= 25.0 && app.yaw > 20.0 {
    //     str_yaw = "-0.625".to_string();
    //     // app.yaw =-0.625;
    // } else if app.yaw <= 20.0 && app.yaw > 15.0 {
    //     str_yaw = "-0.75".to_string();
    //     // app.yaw =-0.75;
    // } else if app.yaw <= 15.0 && app.yaw > 10.0 {
    //     str_yaw = "-0.875".to_string();
    //     // app.yaw =-0.875;
    // } else if app.yaw <= 10.0 {
    //     str_yaw = "-1.0".to_string();
    //     // app.yaw =-1.0
    // }
    if app.yaw == 90.0 {
        str_yaw = "80".to_string();
    } else if app.yaw == 85.0 {
        str_yaw = "70".to_string();
    } else if app.yaw == 80.0 {
        str_yaw = "60".to_string();
    } else if app.yaw == 75.0 {
        str_yaw = "50".to_string();
    } else if app.yaw == 70.0 {
        str_yaw = "40".to_string();
    } else if app.yaw == 65.0 {
        str_yaw = "30".to_string();
    } else if app.yaw == 60.0 {
        str_yaw = "20".to_string();
    } else if app.yaw == 55.0 {
        str_yaw = "10".to_string();
    } else if app.yaw == 50.0 {
        str_yaw = "0".to_string();
    } else if app.yaw == 45.0 {
        str_yaw = "-10".to_string();
    } else if app.yaw == 40.0 {
        str_yaw = "-20".to_string();
    } else if app.yaw == 35.0 {
        str_yaw = "-30".to_string();
    } else if app.yaw == 30.0 {
        str_yaw = "-40".to_string();
    } else if app.yaw == 25.0 {
        str_yaw = "-50.0".to_string();
    } else if app.yaw == 20.0 {
        str_yaw = "-60".to_string();
    } else if app.yaw == 15.0 {
        str_yaw = "-70".to_string();
    } else if app.yaw == 10.0 {
        str_yaw = "-80".to_string();
    } else {
        // not a valid command, we set motor to 0
        str_yaw = "0".to_string();
    }
    if app.roll == 90.0 {
        str_roll = "24".to_string();
    } else if app.roll == 85.0 {
        str_roll = "21".to_string();
    } else if app.roll == 80.0 {
        str_roll = "18".to_string();
    } else if app.roll == 75.0 {
        str_roll = "15".to_string();
    } else if app.roll == 70.0 {
        str_roll = "12".to_string();
    } else if app.roll == 65.0 {
        str_roll = "9".to_string();
    } else if app.roll == 60.0 {
        str_roll = "6".to_string();
    } else if app.roll == 55.0 {
        str_roll = "3".to_string();
    } else if app.roll == 50.0 {
        str_roll = "0".to_string();
    } else if app.roll == 45.0 {
        str_roll = "-3".to_string();
    } else if app.roll == 40.0 {
        str_roll = "-6".to_string();
    } else if app.roll == 35.0 {
        str_roll = "-9".to_string();
    } else if app.roll == 30.0 {
        str_roll = "-12".to_string();
    } else if app.roll == 25.0 {
        str_roll = "-15.0".to_string();
    } else if app.roll == 20.0 {
        str_roll = "-18".to_string();
    } else if app.roll == 15.0 {
        str_roll = "-21".to_string();
    } else if app.roll == 10.0 {
        str_roll = "-24".to_string();
    } else {
        // not a valid command, we set motor to 0
        str_roll = "0".to_string();
    }
    if app.pitch == 90.0 {
        str_pitch = "24".to_string();
    } else if app.pitch == 85.0 {
        str_pitch = "21".to_string();
    } else if app.pitch == 80.0 {
        str_pitch = "18".to_string();
    } else if app.pitch == 75.0 {
        str_pitch = "15".to_string();
    } else if app.pitch == 70.0 {
        str_pitch = "12".to_string();
    } else if app.pitch == 65.0 {
        str_pitch = "9".to_string();
    } else if app.pitch == 60.0 {
        str_pitch = "6".to_string();
    } else if app.pitch == 55.0 {
        str_pitch = "3".to_string();
    } else if app.pitch == 50.0 {
        str_pitch = "0".to_string();
    } else if app.pitch == 45.0 {
        str_pitch = "-3".to_string();
    } else if app.pitch == 40.0 {
        str_pitch = "-6".to_string();
    } else if app.pitch == 35.0 {
        str_pitch = "-9".to_string();
    } else if app.pitch == 30.0 {
        str_pitch = "-12".to_string();
    } else if app.pitch == 25.0 {
        str_pitch = "-15.0".to_string();
    } else if app.pitch == 20.0 {
        str_pitch = "-18".to_string();
    } else if app.pitch == 15.0 {
        str_pitch = "-21".to_string();
    } else if app.pitch == 10.0 {
        str_pitch = "-24".to_string();
    } else {
        // not a valid command, we set motor to 0
        str_pitch = "0".to_string();
    }
    //lift manual
    if app.mode_sent == 2 || app.mode_sent == 0 || app.mode_sent == 3 {
        if app.lift == 90 {
            str_lift = "0".to_string();
        } else if app.lift == 85 {
            str_lift = "205".to_string();
        } else if app.lift == 80 {
            str_lift = "210".to_string();
        } else if app.lift == 75 {
            str_lift = "220".to_string();
        } else if app.lift == 70 {
            str_lift = "230".to_string();
        } else if app.lift == 65 {
            str_lift = "240".to_string();
        } else if app.lift == 60 {
            str_lift = "250".to_string();
        } else if app.lift == 55 {
            str_lift = "260".to_string();
        } else if app.lift == 50 {
            str_lift = "270".to_string();
        } else if app.lift == 45 {
            str_lift = "280".to_string();
        } else if app.lift == 40 {
            str_lift = "290".to_string();
        } else if app.lift == 35 {
            str_lift = "300".to_string();
        } else if app.lift == 30 {
            str_lift = "310".to_string();
        } else if app.lift == 25 {
            str_lift = "320".to_string();
        } else if app.lift == 20 {
            str_lift = "330".to_string();
        } else if app.lift == 15 {
            str_lift = "340".to_string();
        } else {
            // not a valid command, we set motor to 0
            str_lift = "350".to_string();
        }
    }
    //lift control
    if app.mode_sent == 5 || app.mode_sent == 6 || app.mode_sent == 4 {
        if app.lift == 90 {
            str_lift = "0".to_string();
        } else if app.lift == 85 {
            str_lift = "230".to_string();
        } else if app.lift == 80 {
            str_lift = "260".to_string();
        } else if app.lift == 75 {
            str_lift = "290".to_string();
        } else if app.lift == 70 {
            str_lift = "320".to_string();
        } else if app.lift == 65 {
            str_lift = "350".to_string();
        } else if app.lift == 60 {
            str_lift = "380".to_string();
        } else if app.lift == 55 {
            str_lift = "410".to_string();
        } else if app.lift == 50 {
            str_lift = "440".to_string();
        } else if app.lift == 45 {
            str_lift = "470".to_string();
        } else if app.lift == 40 {
            str_lift = "500".to_string();
        } else if app.lift == 35 {
            str_lift = "520".to_string();
        } else if app.lift == 30 {
            str_lift = "540".to_string();
        } else if app.lift == 25 {
            str_lift = "560".to_string();
        } else if app.lift == 20 {
            str_lift = "580".to_string();
        } else if app.lift == 15 {
            str_lift = "590".to_string();
        } else {
            // not a valid command, we set motor to 0
            str_lift = "600".to_string();
        }
    }
    //lift height
    if app.mode_sent == 7 {
        if app.lift == 90 {
            str_lift = "0".to_string();
        } else if app.lift == 85 {
            str_lift = "240".to_string();
        } else if app.lift == 80 {
            str_lift = "280".to_string();
        } else if app.lift == 75 {
            str_lift = "320".to_string();
        } else if app.lift == 70 {
            str_lift = "360".to_string();
        } else if app.lift == 65 {
            str_lift = "400".to_string();
        } else if app.lift == 60 {
            str_lift = "440".to_string();
        } else if app.lift == 55 {
            str_lift = "480".to_string();
        } else if app.lift == 50 {
            str_lift = "520".to_string();
        } else if app.lift == 45 {
            str_lift = "560".to_string();
        } else if app.lift == 40 {
            str_lift = "600".to_string();
        } else if app.lift == 35 {
            str_lift = "640".to_string();
        } else if app.lift == 30 {
            str_lift = "680".to_string();
        } else if app.lift == 25 {
            str_lift = "720".to_string();
        } else if app.lift == 20 {
            str_lift = "760".to_string();
        } else if app.lift == 15 {
            str_lift = "800".to_string();
        } else {
            // not a valid command, we set motor to 0
            str_lift = "840".to_string();
        }
    }

    // if app.roll <= 90.0 && app.roll >= 85.0 {
    //     str_roll = "1.0".to_string();
    //     //  app.roll = 1.0;
    // } else if app.roll <= 85.0 && app.roll > 80.0 {
    //     str_roll = "0.875".to_string();
    //     //  app.roll =0.875;
    // } else if app.roll <= 80.0 && app.roll > 75.0 {
    //     str_roll = "0.75".to_string();
    // //    app.roll = 0.75;
    // } else if app.roll <= 75.0 && app.roll > 70.0 {
    //     str_roll = "0.625".to_string();
    //     // app.roll = 0.625;
    // } else if app.roll <= 70.0 && app.roll > 65.0 {
    //     str_roll = "0.5".to_string();
    //     // app.roll = 0.5;
    // } else if app.roll <= 65.0 && app.roll > 60.0 {
    //     str_roll = "0.375".to_string();
    //     // app.roll =0.375;
    // } else if app.roll <= 60.0 && app.roll > 55.0 {
    //     str_roll = "0.25".to_string();
    //     // app.roll =0.25;
    // } else if app.roll <= 55.0 && app.roll > 50.0 {
    //     str_roll = "0.125".to_string();
    //     // app.roll =0.125;
    // } else if app.roll <= 50.0 && app.roll > 45.0 {
    //     str_roll = "0.0".to_string();
    //     // app.roll =0.0;
    // } else if app.roll <= 45.0 && app.roll > 40.0 {
    //     str_roll = "0.125".to_string();
    //     // app.roll =-0.125;
    // } else if app.roll <= 40.0 && app.roll > 35.0 {
    //     str_roll = "-0.25".to_string();
    //     // app.roll = -0.25;
    // } else if app.roll <= 35.0 && app.roll > 30.0 {
    //     str_roll = "-0.375".to_string();
    //     // app.roll =-0.375;
    // } else if app.roll <= 30.0 && app.roll > 25.0 {
    //     str_roll = "-0.5".to_string();
    //     // app.roll = -0.5;
    // } else if app.roll <= 25.0 && app.roll > 20.0 {
    //     str_roll = "-0.625".to_string();
    //     // app.roll =-0.625;
    // } else if app.roll <= 20.0 && app.roll > 15.0 {
    //     str_roll = "-0.75".to_string();
    //     // app.roll =-0.75;
    // } else if app.roll <= 15.0 && app.roll > 10.0 {
    //     str_roll = "-0.875".to_string();
    //     // app.roll =-0.875;
    // } else if app.roll <= 10.0 {
    //     str_roll = "-1.0".to_string();
    //     // app.roll =-1.0
    // }
    let mut pitch_style = Style::default().fg(Color::White);
    let mut roll_style = Style::default().fg(Color::White);
    let mut yaw_style = Style::default().fg(Color::White);
    let mut lift_style = Style::default().fg(Color::White);
    let mut p_style = Style::default().fg(Color::White);
    let mut p1_style = Style::default().fg(Color::White);
    let mut p2_style = Style::default().fg(Color::White);
    if app.pitch != 50.0 {
        pitch_style = Style::default().fg(Color::Green);
    }
    if app.roll != 50.0 {
        roll_style = Style::default().fg(Color::Green);
    }
    if app.yaw != 50.0 {
        yaw_style = Style::default().fg(Color::Green);
    }
    if app.lift != 90 {
        lift_style = Style::default().fg(Color::Green);
    }
    if app.p != 50.0 {
        p_style = Style::default().fg(Color::Green);
    }
    if app.p1 != 50.0 {
        p1_style = Style::default().fg(Color::Green);
    }
    if app.p2 != 50.0 {
        p2_style = Style::default().fg(Color::Green);
    }
    // let mut ae1: u16 = (lift_temp - pitch - yaw) as u16;
    //     let mut ae2: u16 = (lift_temp - roll + yaw) as u16;
    //     let mut ae3: u16 = (lift_temp + pitch - yaw) as u16;
    //     let mut ae4: u16 = (lift_temp + roll + yaw) as u16;
    let m1_str = (str_lift.parse::<f32>().unwrap()
        - str_pitch.parse::<f32>().unwrap()
        - str_yaw.parse::<f32>().unwrap())
    .to_string();
    let m2_str = (str_lift.parse::<f32>().unwrap() - str_roll.parse::<f32>().unwrap()
        + str_yaw.parse::<f32>().unwrap())
    .to_string();
    let m3_str = (str_lift.parse::<f32>().unwrap() + str_pitch.parse::<f32>().unwrap()
        - str_yaw.parse::<f32>().unwrap())
    .to_string();
    let m4_str = (str_lift.parse::<f32>().unwrap()
        + str_roll.parse::<f32>().unwrap()
        + str_yaw.parse::<f32>().unwrap())
    .to_string();
    let rows = vec![
        Row::new(vec!["pitch", &str_pitch]).style(pitch_style),
        Row::new(vec!["roll", &str_roll]).style(roll_style),
        Row::new(vec!["yaw", &str_yaw]).style(yaw_style),
        Row::new(vec!["lift", &str_lift]).style(lift_style),
        Row::new(vec!["motor 1", &m1_str]).style(up_style),
        Row::new(vec!["motor 2", &m2_str]).style(up_style),
        Row::new(vec!["motor 3", &m3_str]).style(up_style),
        Row::new(vec!["motor 4", &m4_str]).style(up_style),
        Row::new(vec!["P", &str_p]).style(p_style),
        Row::new(vec!["P1", &str_p1]).style(p1_style),
        Row::new(vec!["P2", &str_p2]).style(p2_style),
        Row::new(vec!["mode", &str_mode]).style(up_style),
    ];
    let table = Table::new(rows)
        .header(
            Row::new(vec!["Signal", "Value"])
                .style(Style::default().fg(Color::Yellow))
                .bottom_margin(1),
        )
        .block(
            Block::default()
                .title("Control Signals")
                .borders(Borders::LEFT | Borders::RIGHT | Borders::BOTTOM),
        )
        .widths(&[Constraint::Length(15), Constraint::Length(15)]);
    f.render_widget(table, area);
}
fn draw_drone<B>(f: &mut Frame<B>, app: &mut App, area: Rect)
where
    B: Backend,
{
    let map = Canvas::default()
        .block(Block::default().title("Drone").borders(Borders::ALL))
        .paint(|ctx| {
            ctx.draw(&Rectangle {
                color: Color::Yellow,
                x: 0.0,
                y: -10.0,
                width: 20.0,
                height: 10.0,
            });
            ctx.layer();
            ctx.draw(&Rectangle {
                x: 0.0,
                y: 30.0,
                width: 20.0,
                height: 10.0,
                color: Color::Yellow,
            });
            ctx.draw(&Rectangle {
                x: 30.0,
                y: 10.0,
                width: 20.0,
                height: 10.0,
                color: Color::Yellow,
            });
            ctx.draw(&Rectangle {
                x: -30.0,
                y: 10.0,
                width: 20.0,
                height: 10.0,
                color: Color::Yellow,
            });
            ctx.draw(&Line {
                x1: 10.0,
                y1: 28.0,
                x2: 10.0,
                y2: 1.0,
                color: Color::Red,
            });
            ctx.draw(&Line {
                x1: -5.0,
                y1: 15.0,
                x2: 26.0,
                y2: 15.0,
                color: Color::Red,
            });
            //draw 4 arrows
            if app.yaw > 50.0 && (app.mode_sent != 0 || app.mode_sent != 1) {
                let span1 = vec![Span::styled(
                    "❯❯❯❯❯",
                    Style::default()
                        .add_modifier(Modifier::RAPID_BLINK)
                        .fg(Color::Red),
                )];
                ctx.print(60.0, 0.0, span1);
            } else if app.yaw < 50.0 && (app.mode_sent != 0 || app.mode_sent != 1) {
                let span1 = vec![Span::styled(
                    "❮❮❮❮❮",
                    Style::default()
                        .add_modifier(Modifier::RAPID_BLINK)
                        .fg(Color::Red),
                )];
                ctx.print(60.0, 0.0, span1);
            }
            if app.pitch > 50.0 && (app.mode_sent != 0 || app.mode_sent != 1) {
                let span1 = vec![Span::styled(
                    "▲▲▲▲▲",
                    Style::default()
                        .add_modifier(Modifier::RAPID_BLINK)
                        .fg(Color::Red),
                )];
                ctx.print(60.0, 10.0, span1);
            } else if app.pitch < 50.0 && (app.mode_sent != 0 || app.mode_sent != 1) {
                let span1 = vec![Span::styled(
                    "▼▼▼▼▼",
                    Style::default()
                        .add_modifier(Modifier::RAPID_BLINK)
                        .fg(Color::Red),
                )];
                ctx.print(60.0, 10.0, span1);
            }
            if app.roll > 50.0 && (app.mode_sent != 0 || app.mode_sent != 1) {
                let span1 = vec![Span::styled(
                    "⟳⟳⟳⟳",
                    Style::default()
                        .add_modifier(Modifier::RAPID_BLINK)
                        .fg(Color::Red),
                )];
                ctx.print(60.0, 20.0, span1);
            } else if app.roll < 50.0 && (app.mode_sent != 0 || app.mode_sent != 1) {
                let span1 = vec![Span::styled(
                    "⟲⟲⟲⟲",
                    Style::default()
                        .add_modifier(Modifier::RAPID_BLINK)
                        .fg(Color::Red),
                )];
                ctx.print(60.0, 20.0, span1);
            }

            ctx.print(
                -30.0,
                5.0,
                vec![Span::styled(
                    "Motor4",
                    Style::default().add_modifier(Modifier::RAPID_BLINK),
                )],
            );
            ctx.print(
                30.0,
                5.0,
                vec![Span::styled(
                    "Motor2",
                    Style::default().add_modifier(Modifier::RAPID_BLINK),
                )],
            );
            ctx.print(
                -0.0,
                45.0,
                vec![Span::styled(
                    "Motor1",
                    Style::default().add_modifier(Modifier::RAPID_BLINK),
                )],
            );
            ctx.print(
                0.0,
                -15.0,
                vec![Span::styled(
                    "Motor3",
                    Style::default().add_modifier(Modifier::RAPID_BLINK),
                )],
            );
        })
        .marker(if app.enhanced_graphics {
            symbols::Marker::Braille
        } else {
            symbols::Marker::Dot
        })
        .x_bounds([-180.0, 180.0])
        .y_bounds([-90.0, 90.0]);
    f.render_widget(map, area);
}
// fn draw_bar<B>(f: &mut Frame<B>,app: &mut App, area: Rect)
// where
// B: Backend,
// {
//     let a=[("y_r",(app.ypr[0]*100.0) as u64),("y_f",(app.ypr_filter[0]*100.0) as u64),("p_r",(app.ypr[1]*100.0) as u64),("p_f",(app.ypr_filter[1]*100.0) as u64),("r_r",(app.ypr[2]*100.0) as u64),("r_f",(app.ypr_filter[2]*100.0) as u64)];
//     let barchart = BarChart::default()
//             .block(Block::default().borders(Borders::ALL).title("Bar chart"))
//             .data(&a)
//             .bar_width(3)
//             .bar_gap(2)
//             .bar_set(if app.enhanced_graphics {
//                 symbols::bar::NINE_LEVELS
//             } else {
//                 symbols::bar::THREE_LEVELS
//             })
//             .value_style(
//                 Style::default()
//                     .fg(Color::Black)
//                     .bg(Color::Green)
//                     .add_modifier(Modifier::ITALIC),
//             )
//             .label_style(Style::default().fg(Color::Yellow))
//             .bar_style(Style::default().fg(Color::Green));
//         f.render_widget(barchart, area);
// }

fn draw_charts<B>(f: &mut Frame<B>, app: &mut App, area: Rect)
where
    B: Backend,
{
    let x_labels = vec![
        Span::styled(
            format!("3.14"),
            Style::default().add_modifier(Modifier::BOLD),
        ),
        Span::raw(format!("0")),
        Span::styled(
            format!("-3.14"),
            Style::default().add_modifier(Modifier::BOLD),
        ),
    ];
    // let time= [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14];
    let a = [(0.0, app.ypr[1].into()), (app.ypr[2].into(), 0.0)];
    let b = [
        (0.0, app.ypr_filter[1].into()),
        (app.ypr_filter[2].into(), 0.0),
    ];
    let datasets = vec![
        Dataset::default()
            .name("dmp")
            .marker(symbols::Marker::Block)
            .style(Style::default().fg(Color::Cyan))
            .data(&a),
        Dataset::default()
            .name("kal")
            .marker(symbols::Marker::Block)
            .style(Style::default().fg(Color::Yellow))
            .data(&b),
    ];
    let chart = Chart::new(datasets)
        .block(
            Block::default()
                .title(Span::styled(
                    "Filter Accuracy",
                    Style::default()
                        .fg(Color::Cyan)
                        .add_modifier(Modifier::BOLD),
                ))
                .borders(Borders::ALL),
        )
        .x_axis(
            Axis::default()
                .title("Roll")
                .style(Style::default().fg(Color::Gray))
                // .bounds(app.)
                .bounds([-3.14, 3.14])
                .labels(x_labels),
        )
        .y_axis(
            Axis::default()
                .title("Pitch")
                .style(Style::default().fg(Color::Gray))
                .bounds([-3.14, 3.14])
                .labels(vec![
                    Span::styled("-3.14", Style::default().add_modifier(Modifier::BOLD)),
                    Span::raw("0"),
                    Span::styled("3.14", Style::default().add_modifier(Modifier::BOLD)),
                ]),
        );
    f.render_widget(chart, area);
}
fn draw_two<B>(f: &mut Frame<B>, app: &mut App, area: Rect)
where
    B: Backend,
{
    let chunks = Layout::default()
        .constraints([Constraint::Ratio(40, 100), Constraint::Ratio(60, 100)])
        .direction(Direction::Vertical)
        .margin(1)
        .split(area);
    // draw_input_values(f, app, chunks[0]);
    // draw_serial(f, app, chunks[0]);
    draw_text(f, chunks[0]);
    draw_input_values(f, app, chunks[1]);
    // draw_drone(f, app, chunks[1]);
}
fn draw_legend<B>(f: &mut Frame<B>, app: &mut App, area: Rect)
where
    B: Backend,
{
    let chunks = Layout::default()
        .constraints([
            Constraint::Min(20),
            Constraint::Min(40),
            Constraint::Min(20),
        ])
        .direction(Direction::Vertical)
        .margin(1)
        .split(area);
    // draw_input_values(f, app, chunks[0]);
    draw_serial(f, app, chunks[0]);
    draw_two(f, app, chunks[1]);
    // draw_input_values(f, app, chunks[2]);
    // draw_drone(f, app, chunks[1]);
}

fn draw_block3<B>(f: &mut Frame<B>, app: &mut App, area: Rect)
where
    B: Backend,
{
    let chunks = Layout::default()
        .constraints([Constraint::Percentage(50), Constraint::Percentage(50)].as_ref())
        .direction(Direction::Vertical)
        .margin(1)
        .split(area);
    draw_gauges(f, app, chunks[0]);
    // draw_bar(f,app,chunks[1]);
    draw_charts(f, app, chunks[1]);
    // draw_serial(f, app, chunks[1]);
}

fn draw_second_tab<B>(f: &mut Frame<B>, app: &mut App, area: Rect)
where
    B: Backend,
{
    let chunks = Layout::default()
        .constraints(
            [
                Constraint::Percentage(30),
                Constraint::Percentage(40),
                Constraint::Percentage(30),
            ]
            .as_ref(),
        )
        .direction(Direction::Horizontal)
        .split(area);
    draw_legend(f, app, chunks[0]);
    // draw_serial(f, app, chunks[0]);
    draw_drone(f, app, chunks[1]);
    draw_block3(f, app, chunks[2]);
}
