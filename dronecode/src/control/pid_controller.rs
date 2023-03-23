use tudelft_quadrupel::fixed::types::I16F16;

use super::SensorData;


pub struct GeneralController {
    pub yaw_control: YawController,
    pub pitch_control: PitchController,
    pub roll_control: RollController,
}

impl GeneralController {
    pub fn new(yaw_control: YawController, pitch_control: PitchController, roll_control: RollController) -> GeneralController {
        GeneralController {
            yaw_control,
            pitch_control,
            roll_control,
        }
    }
}
pub struct PIDController {
    pub kp: I16F16,
    pub ki: I16F16,
    pub kd: I16F16,
}

impl PIDController {
    pub fn new(kp: I16F16, ki: I16F16, kd: I16F16) -> PIDController {
        PIDController {
            kp: kp,
            ki: ki,
            kd: kd,
        }
    }

    pub fn increment_kp(&mut self, increment: I16F16) {
        self.kp += increment;
    }

    pub fn increment_ki(&mut self, increment: I16F16) {
        self.ki += increment;
    }

    pub fn increment_kd(&mut self, increment: I16F16) {
        self.kd += increment;
    }

    pub fn decrement_kp(&mut self, decrement: I16F16) {
        self.kp -= decrement;
    }

    pub fn decrement_ki(&mut self, decrement: I16F16) {
        self.ki -= decrement;
    }

    pub fn decrement_kd(&mut self, decrement: I16F16) {
        self.kd -= decrement;
    }
}

pub struct YawController {
    pub pid: PIDController,
    pub proportioanl: I16F16,
    pub integral: I16F16,
    pub derivative: I16F16,
    pub yaw: I16F16,
    pub prev_x: I16F16,
    pub x: I16F16,
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
            proportioanl: I16F16::from_num(0),
            integral: I16F16::from_num(0),
            derivative: I16F16::from_num(0),
            yaw: I16F16::from_num(0),
            prev_x: I16F16::from_num(0),
            x: I16F16::from_num(0),
            dt: I16F16::from_num(0),
            current_yaw: I16F16::from_num(0),
            prev_error: I16F16::from_num(0),
            error: I16F16::from_num(0),
            new_yaw: I16F16::from_num(0),
        }
    }

    pub fn reset_values(&mut self) {
        self.proportioanl = I16F16::from_num(0);
        self.integral = I16F16::from_num(0);
        self.derivative = I16F16::from_num(0);
        self.yaw = I16F16::from_num(0);
        self.prev_x = I16F16::from_num(0);
        self.x = I16F16::from_num(0);
        self.dt = I16F16::from_num(0);
        self.current_yaw = I16F16::from_num(0);
        self.prev_error = I16F16::from_num(0);
        self.error = I16F16::from_num(0);
        self.new_yaw = I16F16::from_num(0);
    }

    pub fn update_yaw(&mut self, command: I16F16) {
        self.yaw = command;
    }

    pub fn update_x(&mut self, sensor_data: SensorData) {
        self.x = sensor_data.ypr.yaw;
    }

    pub fn update_prev_x(&mut self) {
        self.prev_x = self.x;
    }

    pub fn update_dt(&mut self, sensor_data: SensorData) {
        self.dt = I16F16::from_num(sensor_data.dt.as_secs_f32());
    }

    pub fn update_current_yaw(&mut self) {
        self.current_yaw = (self.x - self.prev_x) / self.dt;
    }

    pub fn error(&mut self) {
        self.error = self.yaw - self.current_yaw;
    }

    pub fn update_prev_error(&mut self) {
        self.prev_error = self.error;
    }

    pub fn update_proportional(&mut self) {
        self.proportioanl = self.pid.kp * self.error;
    }

    pub fn update_integral(&mut self) {
        self.integral = self.integral + self.pid.ki * self.error * self.dt;
    }

    pub fn update_derivative(&mut self) {
        self.derivative = self.pid.kd * (self.error - self.prev_error) / self.dt;
    }

    pub fn update_new_yaw(&mut self) {
        self.new_yaw = self.proportioanl + self.integral + self.derivative;
    }
}

pub struct PitchController {
    pub pid: PIDController,
    pub proportioanl: I16F16,
    pub integral: I16F16,
    pub derivative: I16F16,
    pub pitch: I16F16,
    pub prev_y: I16F16,
    pub y: I16F16,
    pub dt: I16F16,
    pub prev_error: I16F16,
    pub error: I16F16,
    pub new_pitch: I16F16,
}

impl PitchController {
    pub fn new(pid: PIDController) -> PitchController {
        PitchController {
            pid,
            proportioanl: I16F16::from_num(0),
            integral: I16F16::from_num(0),
            derivative: I16F16::from_num(0),
            pitch: I16F16::from_num(0),
            prev_y: I16F16::from_num(0),
            y: I16F16::from_num(0),
            dt: I16F16::from_num(0),
            prev_error: I16F16::from_num(0),
            error: I16F16::from_num(0),
            new_pitch: I16F16::from_num(0),
        }
    }

    pub fn reset_values(&mut self) {
        self.proportioanl = I16F16::from_num(0);
        self.integral = I16F16::from_num(0);
        self.derivative = I16F16::from_num(0);
        self.pitch = I16F16::from_num(0);
        self.prev_y = I16F16::from_num(0);
        self.y = I16F16::from_num(0);
        self.dt = I16F16::from_num(0);
        self.prev_error = I16F16::from_num(0);
        self.error = I16F16::from_num(0);
        self.new_pitch = I16F16::from_num(0);
    }

    pub fn update_pitch(&mut self, command: I16F16) {
        self.pitch = command;
    }

    pub fn update_y(&mut self, sensor_data: SensorData) {
        self.y = sensor_data.ypr.pitch;
    }

    pub fn update_prev_y(&mut self) {
        self.prev_y = self.y;
    }

    pub fn update_dt(&mut self, sensor_data: SensorData) {
        self.dt = I16F16::from_num(sensor_data.dt.as_secs_f32());
    }

    pub fn error(&mut self) {
        self.error = self.pitch - self.y;
    }

    pub fn update_prev_error(&mut self) {
        self.prev_error = self.error;
    }

    pub fn update_proportional(&mut self) {
        self.proportioanl = self.pid.kp * self.error;
    }

    pub fn update_integral(&mut self) {
        self.integral = self.integral + self.pid.ki * self.error * self.dt;
    }

    pub fn update_derivative(&mut self) {
        self.derivative = self.pid.kd * (self.error - self.prev_error) / self.dt;
    }

    pub fn update_new_pitch(&mut self) {
        self.new_pitch = self.proportioanl + self.integral + self.derivative;
    }
}

pub struct RollController {
    pub pid: PIDController,
    pub proportioanl: I16F16,
    pub integral: I16F16,
    pub derivative: I16F16,
    pub roll: I16F16,
    pub prev_z: I16F16,
    pub z: I16F16,
    pub dt: I16F16,
    pub prev_error: I16F16,
    pub error: I16F16,
    pub new_roll: I16F16,
}

impl RollController {
    pub fn new(pid: PIDController) -> RollController {
        RollController {
            pid,
            proportioanl: I16F16::from_num(0),
            integral: I16F16::from_num(0),
            derivative: I16F16::from_num(0),
            roll: I16F16::from_num(0),
            prev_z: I16F16::from_num(0),
            z: I16F16::from_num(0),
            dt: I16F16::from_num(0),
            prev_error: I16F16::from_num(0),
            error: I16F16::from_num(0),
            new_roll: I16F16::from_num(0),
        }
    }

    pub fn reset_values(&mut self) {
        self.proportioanl = I16F16::from_num(0);
        self.integral = I16F16::from_num(0);
        self.derivative = I16F16::from_num(0);
        self.roll = I16F16::from_num(0);
        self.prev_z = I16F16::from_num(0);
        self.z = I16F16::from_num(0);
        self.dt = I16F16::from_num(0);
        self.prev_error = I16F16::from_num(0);
        self.error = I16F16::from_num(0);
        self.new_roll = I16F16::from_num(0);
    }

    pub fn update_roll(&mut self, command: I16F16) {
        self.roll = command;
    }

    pub fn update_z(&mut self, sensor_data: SensorData) {
        self.z = sensor_data.ypr.roll;
    }

    pub fn update_prev_z(&mut self) {
        self.prev_z = self.z;
    }

    pub fn update_dt(&mut self, sensor_data: SensorData) {
        self.dt = I16F16::from_num(sensor_data.dt.as_secs_f32());
    }

    pub fn error(&mut self) {
        self.error = self.roll - self.z;
    }

    pub fn update_prev_error(&mut self) {
        self.prev_error = self.error;
    }

    pub fn update_proportional(&mut self) {
        self.proportioanl = self.pid.kp * self.error;
    }

    pub fn update_integral(&mut self) {
        self.integral = self.integral + self.pid.ki * self.error * self.dt;
    }

    pub fn update_derivative(&mut self) {
        self.derivative = self.pid.kd * (self.error - self.prev_error) / self.dt;
    }

    pub fn update_new_roll(&mut self) {
        self.new_roll = self.proportioanl + self.integral + self.derivative;
    }
}