use crate::control::state_machine::StateMachine;
use crate::yaw_pitch_roll::YawPitchRoll;
use alloc::format;

use crate::control::state_machine::State::Safety;
use tudelft_quadrupel::barometer::read_pressure;
use tudelft_quadrupel::battery::read_battery;
use tudelft_quadrupel::block;
use tudelft_quadrupel::led::Led::Blue;
use tudelft_quadrupel::led::Yellow;
use tudelft_quadrupel::motor::get_motors;
use tudelft_quadrupel::mpu::{read_dmp_bytes, read_raw};
use tudelft_quadrupel::time::{set_tick_frequency, wait_for_next_tick, Instant};
use tudelft_quadrupel::uart::send_bytes;

mod state_machine;

pub fn control_loop() -> ! {
    set_tick_frequency(100);
    let mut last = Instant::now();

    let state_machine = StateMachine::new();

    // Quick test to see if state_machine is initialized, can be removed later.
    if state_machine.state() == Safety {
        Yellow.toggle();
    }

    for i in 0.. {
        let _ = Blue.toggle();
        let now = Instant::now();
        let dt = now.duration_since(last);
        last = now;

        // In communication protocol parse the bit string for mode to one of states in state_machine
        // Store this parsed mode as next_state then call function transition()
        // state_machine.transition(next_state)

        // if state_machine.permissions.calibration {
        // Do calibration.
        // }

        // if state_machine.permissions.controller {
        // process controller commands
        // }

        //if (state_machine.permissions.yaw_control) {
        // yaw control loop in here
        //}

        let motors = get_motors();
        let quaternion = block!(read_dmp_bytes()).unwrap();
        let ypr = YawPitchRoll::from(quaternion);
        let (accel, _) = read_raw().unwrap();
        let bat = read_battery();
        let pres = read_pressure();

        if i % 100 == 0 {
            send_bytes(format!("DTT: {:?}ms\n", dt.as_millis()).as_bytes());
            send_bytes(
                format!(
                    "MTR: {} {} {} {}\n",
                    motors[0], motors[1], motors[2], motors[3]
                )
                .as_bytes(),
            );
            send_bytes(format!("YPR {} {} {}\n", ypr.yaw, ypr.pitch, ypr.roll).as_bytes());
            send_bytes(format!("ACC {} {} {}\n", accel.x, accel.y, accel.z).as_bytes());
            send_bytes(format!("BAT {bat}\n").as_bytes());
            send_bytes(format!("BAR {pres} \n").as_bytes());
            send_bytes("\n".as_bytes());
        }

        // wait until the timer interrupt goes off again
        // based on the frequency set above
        wait_for_next_tick();
    }
    unreachable!();
}
