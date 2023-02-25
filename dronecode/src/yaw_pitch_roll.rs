use tudelft_quadrupel::{
    fixed::{types, FixedI32},
    mpu::structs::Quaternion,
};

use cordic;

// A struct to hold yaw, pitch, and roll values
#[derive(Debug, Copy, Clone)]
pub struct YawPitchRoll {
    pub yaw: FixedI32<types::extra::U29>,
    pub pitch: FixedI32<types::extra::U29>,
    pub roll: FixedI32<types::extra::U29>,
}

// An implementation of the `From` trait for converting a `Quaternion` into a `YawPitchRoll`
impl From<Quaternion> for YawPitchRoll {
    fn from(q: Quaternion) -> Self {
        // Extract the components of the quaternion
        let Quaternion { w, x, y, z } = q;

        // Calculate the values for yaw, pitch, and roll using the quaternion components
        let two = 2i32;

        let (gx, gy, gz) = (
            two * x * z - w * y,
            two * w * x + y * z,
            w * w - x * x - y * y + z * z,
        );

        // Convert the gx, gy, and gz values to FixedI32<types::extra::U29> format
        let gx = FixedI32::<types::extra::U29>::from_num(gx);
        let gy = FixedI32::<types::extra::U29>::from_num(gy);
        let gz = FixedI32::<types::extra::U29>::from_num(gz);

        // Calculate the values for yaw, pitch, and roll using the Cordic library
        let yaw = cordic::atan2::<FixedI32<types::extra::U29>>(gx, gz);
        let pitch = cordic::atan2::<FixedI32<types::extra::U29>>(gy, gz);
        let roll = cordic::atan2::<FixedI32<types::extra::U29>>(gx, gy);

        // Create a new `YawPitchRoll` struct with the calculated yaw, pitch, and roll values
        Self { yaw, pitch, roll }
    }
}
