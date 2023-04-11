// use rand::{
//     distributions::{Distribution, Uniform},
//     rngs::ThreadRng,
// };
// use tui::widgets::ListState;
// use tui::terminal;

#[derive(Clone)]
// pub struct RandomSignal {
//     distribution: Uniform<u64>,
//     rng: ThreadRng,
// }

// impl RandomSignal {
//     pub fn new(lower: u64, upper: u64) -> RandomSignal {
//         RandomSignal {
//             distribution: Uniform::new(lower, upper),
//             rng: rand::thread_rng(),
//         }
//     }
// }

// impl Iterator for RandomSignal {
//     type Item = u64;
//     fn next(&mut self) -> Option<u64> {
//         Some(self.distribution.sample(&mut self.rng))
//     }
// }

// #[derive(Clone)]

pub struct TabsState<'a> {
    pub titles: Vec<&'a str>,
    pub index: usize,
}

impl<'a> TabsState<'a> {
    pub fn new(titles: Vec<&'a str>) -> TabsState {
        TabsState { titles, index: 0 }
    }
    // pub fn control_tab(&mut self) {
    //     self.index = 0;
    // }
    // pub fn welcome_tab(&mut self) {
    //     self.index = 1;
    // }
}

// pub struct Signal<S: Iterator> {
//     source: S,
//     pub points: Vec<S::Item>,
//     tick_rate: usize,
// }

// impl<S> Signal<S>
// where
//     S: Iterator,
// {
//     fn on_tick(&mut self) {
//         for _ in 0..self.tick_rate {
//             self.points.remove(0);
//         }
//         self.points
//             .extend(self.source.by_ref().take(self.tick_rate));
//     }
// }

pub struct App<'a> {
    pub title: &'a str,
    pub should_quit: bool,
    pub tabs: TabsState<'a>,
    pub show_chart: bool,
    pub progress: f64,
    // pub sparkline: Signal<RandomSignal>,
    pub enhanced_graphics: bool,
    pub lift: u16,
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
    pub p: f32,
    pub p1: f32,
    pub p2: f32,
    pub mode_sent: u8,
    pub mode: u8,
    pub error: String,
    pub motor: [u16; 4],
    pub duration: u16,
    pub ypr: [f32; 3],
    pub ypr_filter: [f32; 3],
    pub acc: [i16; 3],
    pub batt: u16,
    pub pres: i32,
    pub crc: u16,
    pub ack: u8,
}

impl<'a> App<'a> {
    pub fn new(title: &'a str, _enhanced_graphics: bool) -> App<'a> {
        // let mut rand_signal = RandomSignal::new(0, 100);
        // let sparkline_points = rand_signal.by_ref().take(300).collect();
        App {
            title,
            should_quit: false,
            tabs: TabsState::new(vec!["Control", "Data", "Tab2"]),
            show_chart: true,
            progress: 0.0,
            // sparkline: Signal {
            //     source: rand_signal,
            //     points: sparkline_points,
            //     tick_rate: 1,
            // },
            enhanced_graphics: true,
            lift: 0,
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.0,
            p: 0.0,
            p1: 0.0,
            p2: 0.0,
            mode: 0,
            error: String::new(),
            motor: [0, 0, 0, 0],
            duration: 0,
            ypr: [0.0, 0.0, 0.0],
            ypr_filter: [0.0, 0.0, 0.0],
            acc: [0, 0, 0],
            batt: 0,
            pres: 0,
            mode_sent: 0,
            crc: 0,
            ack: 0b1000_0001, // this is a redundant ack byte
        }
    }

    // pub fn on_up(&mut self) {
    //     // if self.pitch < 1.0 {
    //     //     self.pitch += 0.1;
    //     // }
    // }

    // pub fn on_down(&mut self) {
    //     // if self.pitch > -1.0 {
    //     //     self.pitch -= 0.1;
    //     // }
    // }

    // pub fn on_right(&mut self) {
    //     // if self.roll < 1.0 {
    //     //     self.roll += 0.1;
    //     // }
    // }

    // pub fn on_left(&mut self) {
    //     // if self.roll > -1.0 {
    //     //     self.roll -= 0.1;
    //     // }
    // }
    // pub fn on_end(&mut self) {
    //     self.should_quit = true;
    // }
    // pub fn mode(&mut self, mode: u8){
    //     self.mode = mode;
    // }

    // pub fn on_key(&mut self, c: char) {
    //     match c {
    //         't' => {
    //             self.tabs.index = 2;
    //         }
    //         'c' => {
    //             self.tabs.control_tab();
    //         }
    //         'd' => {
    //             self.tabs.welcome_tab();
    //         }

    //         _ => {}
    //     }
    // }

    pub fn on_tick(&mut self) {
        // Update progress
        self.progress += 0.001;
        if self.progress > 1.0 {
            self.progress = 0.0;
        }
    }
}
