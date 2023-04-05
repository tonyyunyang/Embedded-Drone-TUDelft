use core::u8;

use tudelft_quadrupel::fixed::types::I16F16;

use super::SensorData;

pub struct GeneralController {
    pub yaw_control: YawController,
    pub pitch_control: PitchController,
    pub roll_control: RollController,
}

impl GeneralController {
    pub fn new(
        yaw_control: YawController,
        pitch_control: PitchController,
        roll_control: RollController,
    ) -> GeneralController {
        GeneralController {
            yaw_control,
            pitch_control,
            roll_control,
        }
    }
}
pub struct PIDController {
    pub kp: I16F16,  // yaw control P
    pub kp1: I16F16, // cascaded roll/pitch control P1
    pub kp2: I16F16, // cascaded roll/pitch control P2
    pub ki: I16F16,
    pub kd: I16F16,
}

#[allow(dead_code)]
impl PIDController {
    pub fn new(kp: I16F16, kp1: I16F16, kp2: I16F16, ki: I16F16, kd: I16F16) -> PIDController {
        PIDController {
            kp,  // yaw control P
            kp1, // cascaded roll/pitch control P1
            kp2, // cascaded roll/pitch control P2
            ki,
            kd,
        }
    }

    pub fn set_kp(&mut self, kp: I16F16) {
        self.kp = kp;
    }

    pub fn set_kp1(&mut self, kp1: I16F16) {
        self.kp1 = kp1;
    }

    pub fn set_kp2(&mut self, kp2: I16F16) {
        self.kp2 = kp2;
    }

    pub fn set_ki(&mut self, ki: I16F16) {
        self.ki = ki;
    }

    pub fn set_kd(&mut self, kd: I16F16) {
        self.kd = kd;
    }

    pub fn get_kp(&self) -> I16F16 {
        self.kp
    }

    pub fn get_kp1(&self) -> I16F16 {
        self.kp1
    }

    pub fn get_kp2(&self) -> I16F16 {
        self.kp2
    }

    pub fn get_ki(&self) -> I16F16 {
        self.ki
    }

    pub fn get_kd(&self) -> I16F16 {
        self.kd
    }
}

pub struct YawController {
    pub pid: PIDController,
    pub proportional: I16F16,
    pub integral: I16F16,
    pub derivative: I16F16,
    pub yaw: I16F16,
    pub prev_phi: I16F16,
    pub phi: I16F16,
    pub dt: I16F16,
    pub current_yaw: I16F16,
    pub prev_error: I16F16,
    pub error: I16F16,
    pub new_yaw: I16F16,
}

impl YawController {
    pub fn new(pid: PIDController) -> YawController {
        YawController {
            pid,
            proportional: I16F16::from_num(0),
            integral: I16F16::from_num(0),
            derivative: I16F16::from_num(0),
            yaw: I16F16::from_num(0),
            prev_phi: I16F16::from_num(0),
            phi: I16F16::from_num(0),
            dt: I16F16::from_num(0),
            current_yaw: I16F16::from_num(0),
            prev_error: I16F16::from_num(0),
            error: I16F16::from_num(0),
            new_yaw: I16F16::from_num(0),
        }
    }

    pub fn reset_values(&mut self) {
        self.proportional = I16F16::from_num(0);
        self.integral = I16F16::from_num(0);
        self.derivative = I16F16::from_num(0);
        self.yaw = I16F16::from_num(0);
        self.prev_phi = I16F16::from_num(0);
        self.phi = I16F16::from_num(0);
        self.dt = I16F16::from_num(0);
        self.current_yaw = I16F16::from_num(0);
        self.prev_error = I16F16::from_num(0);
        self.error = I16F16::from_num(0);
        self.new_yaw = I16F16::from_num(0);
    }

    pub fn set_kp(&mut self, kp: I16F16) {
        self.pid.set_kp(kp);
    }

    pub fn update_yaw(&mut self, command: I16F16) {
        self.yaw = command;
    }

    pub fn update_phi(&mut self, sensor_data: &SensorData) {
        self.phi = sensor_data.ypr.yaw;
    }

    pub fn update_prev_phi(&mut self) {
        self.prev_phi = self.phi;
    }

    pub fn update_dt(&mut self, sensor_data: &SensorData) {
        self.dt = I16F16::from_num(sensor_data.dt.as_secs_f32());
    }

    pub fn update_current_yaw(&mut self) {
        self.current_yaw = (self.phi - self.prev_phi) / self.dt;
    }

    pub fn error(&mut self) {
        self.error = self.yaw - self.current_yaw;
    }

    pub fn update_prev_error(&mut self) {
        self.prev_error = self.error;
    }

    pub fn update_proportional(&mut self) {
        self.proportional = self.pid.get_kp() * self.error;
    }

    /// This function is not used in the current implementation
    pub fn update_integral(&mut self) {
        self.integral += self.pid.get_ki() * self.error * self.dt;
    }

    /// This function is not used in the current implementation
    pub fn update_derivative(&mut self) {
        self.derivative = self.pid.get_kd() * (self.error - self.prev_error) / self.dt;
    }

    pub fn update_new_yaw(&mut self) {
        self.new_yaw = self.proportional + self.integral + self.derivative;
    }

    pub fn go_through_process(&mut self, command: I16F16, sensor_data: &SensorData) {
        self.update_yaw(command);
        self.update_phi(sensor_data);
        self.update_dt(sensor_data);
        self.update_current_yaw();
        self.error();
        self.update_prev_error();
        self.update_proportional();
        self.update_integral();
        self.update_derivative();
        self.update_prev_phi();
        self.update_new_yaw();
    }
}

pub struct PitchController {
    pub pid: PIDController,
    pub proportional1: I16F16,
    pub proportional2: I16F16,
    pub integral: I16F16,
    pub derivative: I16F16,
    pub pitch: I16F16,
    pub prev_theta: I16F16,
    pub theta: I16F16,
    pub dt: I16F16,
    pub prev_error: I16F16,
    pub error: I16F16,
    pub new_pitch: I16F16,
}

#[allow(dead_code)]
impl PitchController {
    pub fn new(pid: PIDController) -> PitchController {
        PitchController {
            pid,
            proportional1: I16F16::from_num(0),
            proportional2: I16F16::from_num(0),
            integral: I16F16::from_num(0),
            derivative: I16F16::from_num(0),
            pitch: I16F16::from_num(0),
            prev_theta: I16F16::from_num(0),
            theta: I16F16::from_num(0),
            dt: I16F16::from_num(0),
            prev_error: I16F16::from_num(0),
            error: I16F16::from_num(0),
            new_pitch: I16F16::from_num(0),
        }
    }

    pub fn reset_values(&mut self) {
        self.proportional1 = I16F16::from_num(0);
        self.proportional2 = I16F16::from_num(0);
        self.integral = I16F16::from_num(0);
        self.derivative = I16F16::from_num(0);
        self.pitch = I16F16::from_num(0);
        self.prev_theta = I16F16::from_num(0);
        self.theta = I16F16::from_num(0);
        self.dt = I16F16::from_num(0);
        self.prev_error = I16F16::from_num(0);
        self.error = I16F16::from_num(0);
        self.new_pitch = I16F16::from_num(0);
    }

    pub fn set_kp1(&mut self, kp1: I16F16) {
        self.pid.set_kp1(kp1);
    }

    pub fn set_kp2(&mut self, kp2: I16F16) {
        self.pid.set_kp2(kp2);
    }

    pub fn update_pitch(&mut self, command: I16F16) {
        self.pitch = command;
    }

    pub fn update_theta(&mut self, sensor_data: &SensorData) {
        self.theta = sensor_data.ypr.pitch;
    }

    pub fn update_prev_theta(&mut self) {
        self.prev_theta = self.theta;
    }

    pub fn update_dt(&mut self, sensor_data: &SensorData) {
        self.dt = I16F16::from_num(sensor_data.dt.as_secs_f32());
    }

    pub fn error(&mut self) {
        self.error = self.pitch - self.theta;
    }

    pub fn update_prev_error(&mut self) {
        self.prev_error = self.error;
    }

    pub fn update_proportional1(&mut self) {
        self.proportional1 = self.pid.get_kp1() * self.error;
    }

    pub fn update_proportional2(&mut self) {
        self.proportional2 = self.pid.get_kp2() * self.prev_error;
    }

    /// This function is not used in the current implementation
    pub fn update_integral(&mut self) {
        self.integral += self.pid.get_ki() * self.error * self.dt;
    }

    /// This function is not used in the current implementation
    pub fn update_derivative(&mut self) {
        self.derivative = self.pid.get_kd() * (self.error - self.prev_error) / self.dt;
    }

    pub fn update_new_pitch(&mut self) {
        self.new_pitch = self.proportional1 + self.proportional2 + self.integral + self.derivative;
    }

    pub fn go_through_process(&mut self, command: I16F16, sensor_data: &SensorData) {
        self.update_pitch(command);
        self.update_theta(sensor_data);
        self.update_dt(sensor_data);
        self.error();
        self.update_proportional1();
        self.update_proportional2();
        self.update_new_pitch();
        self.update_prev_theta();
        self.update_prev_error();
    }
}

pub struct RollController {
    pub pid: PIDController,
    pub proportional1: I16F16,
    pub proportional2: I16F16,
    pub integral: I16F16,
    pub derivative: I16F16,
    pub roll: I16F16,
    pub prev_psi: I16F16,
    pub psi: I16F16,
    pub dt: I16F16,
    pub prev_error: I16F16,
    pub error: I16F16,
    pub new_roll: I16F16,
}

#[allow(dead_code)]
impl RollController {
    pub fn new(pid: PIDController) -> RollController {
        RollController {
            pid,
            proportional1: I16F16::from_num(0),
            proportional2: I16F16::from_num(0),
            integral: I16F16::from_num(0),
            derivative: I16F16::from_num(0),
            roll: I16F16::from_num(0),
            prev_psi: I16F16::from_num(0),
            psi: I16F16::from_num(0),
            dt: I16F16::from_num(0),
            prev_error: I16F16::from_num(0),
            error: I16F16::from_num(0),
            new_roll: I16F16::from_num(0),
        }
    }

    pub fn reset_values(&mut self) {
        self.proportional1 = I16F16::from_num(0);
        self.proportional2 = I16F16::from_num(0);
        self.integral = I16F16::from_num(0);
        self.derivative = I16F16::from_num(0);
        self.roll = I16F16::from_num(0);
        self.prev_psi = I16F16::from_num(0);
        self.psi = I16F16::from_num(0);
        self.dt = I16F16::from_num(0);
        self.prev_error = I16F16::from_num(0);
        self.error = I16F16::from_num(0);
        self.new_roll = I16F16::from_num(0);
    }

    pub fn set_kp1(&mut self, kp1: I16F16) {
        self.pid.set_kp1(kp1);
    }

    pub fn set_kp2(&mut self, kp2: I16F16) {
        self.pid.set_kp2(kp2);
    }

    pub fn update_roll(&mut self, command: I16F16) {
        self.roll = command;
    }

    pub fn update_psi(&mut self, sensor_data: &SensorData) {
        self.psi = sensor_data.ypr.roll;
    }

    pub fn update_prev_psi(&mut self) {
        self.prev_psi = self.psi;
    }

    pub fn update_dt(&mut self, sensor_data: &SensorData) {
        self.dt = I16F16::from_num(sensor_data.dt.as_secs_f32());
    }

    pub fn error(&mut self) {
        self.error = self.roll - self.psi;
    }

    pub fn update_prev_error(&mut self) {
        self.prev_error = self.error;
    }

    pub fn update_proportional1(&mut self) {
        self.proportional1 = self.pid.get_kp1() * self.error;
    }

    pub fn update_proportional2(&mut self) {
        self.proportional2 = self.pid.get_kp2() * self.prev_error;
    }

    /// This function is not used in the current implementation
    pub fn update_integral(&mut self) {
        self.integral += self.pid.get_ki() * self.error * self.dt;
    }

    /// This function is not used in the current implementation
    pub fn update_derivative(&mut self) {
        self.derivative = self.pid.get_kd() * (self.error - self.prev_error) / self.dt;
    }

    pub fn update_new_roll(&mut self) {
        self.new_roll = self.proportional1 + self.proportional2 + self.integral + self.derivative;
    }

    pub fn go_through_process(&mut self, command: I16F16, sensor_data: &SensorData) {
        self.update_roll(command);
        self.update_psi(sensor_data);
        self.update_dt(sensor_data);
        self.error();
        self.update_proportional1();
        self.update_prev_error();
        self.update_proportional2();
        self.update_new_roll();
        self.update_prev_psi();
    }
}

pub fn map_p_to_fixed(p: u8) -> I16F16 {
    let max_new: I16F16 = I16F16::from_num(10);
    let min_new: I16F16 = I16F16::from_num(5);
    let max_old: I16F16 = I16F16::from_num(90);
    let min_old: I16F16 = I16F16::from_num(10);
    let p_old: I16F16 = I16F16::from_num(p);

    (max_new - min_new) / (max_old - min_old) * (p_old - min_old) + min_new
}

pub fn map_p1_to_fixed(p: u8) -> I16F16 {
    let max_new: I16F16 = I16F16::from_num(10);
    let min_new: I16F16 = I16F16::from_num(5);
    let max_old: I16F16 = I16F16::from_num(90);
    let min_old: I16F16 = I16F16::from_num(10);
    let p_old: I16F16 = I16F16::from_num(p);

    (max_new - min_new) / (max_old - min_old) * (p_old - min_old) + min_new
}

pub fn map_p2_to_fixed(p: u8) -> I16F16 {
    let max_new: I16F16 = I16F16::from_num(60);
    let min_new: I16F16 = I16F16::from_num(1);
    let max_old: I16F16 = I16F16::from_num(90);
    let min_old: I16F16 = I16F16::from_num(10);
    let p_old: I16F16 = I16F16::from_num(p);

    (max_new - min_new) / (max_old - min_old) * (p_old - min_old) + min_new
}
