use crate::yaw_pitch_roll::YawPitchRoll;
use cordic::atan;
use fixed_trigonometry::sqrt;
use tudelft_quadrupel::fixed::consts::PI;
use tudelft_quadrupel::fixed::types::I16F16;

use super::SensorOffset;

// #[derive(Debug, Clone, Copy)]
// pub(crate) struct LowPass {
//     pub accel_x_in :[I16F16;3],
//     pub accel_y_in :[I16F16;3],
//     pub accel_z_in:[I16F16;3],
//     pub gyro_x_in :[I16F16;3],
//     pub gyro_y_in :[I16F16;3],
//     pub gyro_z_in:[I16F16;3],
//     pub accel_x_out :[I16F16;3],
//     pub accel_y_out :[I16F16;3],
//     pub accel_z_out:[I16F16;3],
//     pub gyro_x_out :[I16F16;3],
//     pub gyro_y_out :[I16F16;3],
//     pub gyro_z_out:[I16F16;3],
//     pub b:[I16F16;3],
//     pub a:[I16F16;2],

// }
// impl LowPass
// {
//     pub fn new()-> LowPass{
//         let i16f16_zero= I16F16::from_num(0.0);
//         LowPass {
//             accel_x_in:[i16f16_zero, i16f16_zero, i16f16_zero],
//             accel_y_in:[i16f16_zero, i16f16_zero, i16f16_zero],
//             accel_z_in:[i16f16_zero, i16f16_zero, i16f16_zero],
//             gyro_x_in: [i16f16_zero, i16f16_zero, i16f16_zero],
//             gyro_y_in: [i16f16_zero, i16f16_zero, i16f16_zero],
//             gyro_z_in: [i16f16_zero, i16f16_zero, i16f16_zero],
//             accel_x_out:[i16f16_zero, i16f16_zero, i16f16_zero],
//             accel_y_out: [i16f16_zero, i16f16_zero, i16f16_zero],
//             accel_z_out: [i16f16_zero, i16f16_zero, i16f16_zero],
//             gyro_x_out: [i16f16_zero, i16f16_zero, i16f16_zero],
//             gyro_y_out: [i16f16_zero, i16f16_zero, i16f16_zero],
//             gyro_z_out: [i16f16_zero, i16f16_zero, i16f16_zero],
//             // b: [I16F16::from_num(0.3625),I16F16::from_num(0.725),I16F16::from_num(0.3625)],
//             // a: [I16F16::from_num(-0.2659),I16F16::from_num(-0.1841)], //40 hz
//             b: [I16F16::from_num(0.2066),I16F16::from_num(0.4131),I16F16::from_num(0.2066)],
//             a: [I16F16::from_num(-0.3695),I16F16::from_num(0.1958)],
//          }

//     }
//     pub fn low_pass(&mut self, gyro: [I16F16;3], acc: [I16F16;3])-> ([I16F16;3], [I16F16;3])    {
//         // self.k =self.k +1;
//         self.accel_x_in[0]= acc[0];
//         self.accel_y_in[0]= acc[1];
//         self.accel_z_in[0]= acc[2];
//         self.gyro_x_in[0]= gyro[0];
//         self.gyro_y_in[0]= gyro[1];
//         self.gyro_z_in[0]= gyro[2];

//         self.accel_x_out[0]= self.a[0]*self.accel_x_out[1] +self.a[1]*self.accel_x_out[2]+ self.b[0]*self.accel_x_in[0] + self.b[1]* self.accel_x_in[1]+ self.b[2]*self.accel_x_in[2];
//         self.accel_y_out[0]= self.a[0]*self.accel_y_out[1] +self.a[1]*self.accel_y_out[2]+ self.b[0]*self.accel_y_in[0] + self.b[1]* self.accel_y_in[1]+ self.b[2]*self.accel_y_in[2];
//         self.accel_z_out[0]= self.a[0]*self.accel_z_out[1] +self.a[1]*self.accel_z_out[2]+ self.b[0]*self.accel_z_in[0] + self.b[1]* self.accel_z_in[1]+ self.b[2]*self.accel_z_in[2];

//         self.gyro_x_out[0]= self.a[0]*self.gyro_x_out[1] +self.a[1]*self.gyro_x_out[2]+ self.b[0]*self.gyro_x_in[0] + self.b[1]* self.gyro_x_in[1]+ self.b[2]*self.gyro_x_in[2];
//         self.gyro_y_out[0]= self.a[0]*self.gyro_y_out[1] +self.a[1]*self.gyro_y_out[2]+ self.b[0]*self.gyro_y_in[0] + self.b[1]* self.gyro_y_in[1]+ self.b[2]*self.gyro_y_in[2];
//         self.gyro_z_out[0]= self.a[0]*self.gyro_z_out[1] +self.a[1]*self.gyro_z_out[2]+ self.b[0]*self.gyro_z_in[0] + self.b[1]* self.gyro_z_in[1]+ self.b[2]*self.gyro_z_in[2];

//         self.gyro_x_in[1]=self.gyro_x_in[0];
//         self.gyro_x_in[2]=self.gyro_x_in[1];
//         self.gyro_y_in[1]=self.gyro_x_in[0];
//         self.gyro_y_in[2]=self.gyro_x_in[1];
//         self.gyro_z_in[1]=self.gyro_x_in[0];
//         self.gyro_z_in[2]=self.gyro_x_in[1];

//         self.accel_x_in[1]=self.accel_x_in[0];
//         self.accel_x_in[2]=self.accel_x_in[1];
//         self.accel_y_in[1]=self.accel_y_in[0];
//         self.accel_y_in[2]=self.accel_y_in[1];
//         self.accel_z_in[1]=self.accel_z_in[0];
//         self.accel_z_in[2]=self.accel_z_in[1];

//         self.gyro_x_out[1]=self.gyro_x_out[0];
//         self.gyro_x_out[2]=self.gyro_x_out[1];
//         self.gyro_y_out[1]=self.gyro_y_out[0];
//         self.gyro_y_out[2]=self.gyro_y_out[1];
//         self.gyro_z_out[1]=self.gyro_z_out[0];
//         self.gyro_z_out[2]=self.gyro_z_out[1];

//         self.accel_x_out[1]=self.accel_x_out[0];
//         self.accel_x_out[2]=self.accel_x_out[1];
//         self.accel_y_out[1]=self.accel_y_out[0];
//         self.accel_y_out[2]=self.accel_y_out[1];
//         self.accel_z_out[1]=self.accel_z_out[0];
//         self.accel_z_out[2]=self.accel_z_out[1];
//         // let mut accel_ypr :YawPitchRoll = YawPitchRoll { yaw: I16F16::from_num(0.0), pitch: I16F16::from_num(0.0), roll:I16F16::from_num(0.0) };
//         // let mut  gyro_ypr :YawPitchRoll = YawPitchRoll { yaw: I16F16::from_num(0.0), pitch: I16F16::from_num(0.0), roll:I16F16::from_num(0.0) };

//         // let roll_acc = atan(self.accel_y_out[0]/ (sqrt::niirf(self.accel_x_out[0]* self.accel_x_out[0] + self.accel_z_out[0] * self.accel_z_out[0], 2)));
//         // let pitch_acc = atan(I16F16::from_num(-1) * self.accel_x_out[0] / sqrt::niirf(self.accel_y_out[0] * self.accel_y_out[0]  + self.accel_z_out[0]  * self.accel_z_out[0], 2));

//         // let roll_gyro = atan(self.gyro_y_out[0]/ (sqrt::niirf(self.gyro_x_out[0]* self.gyro_x_out[0] + self.gyro_z_out[0] * self.gyro_z_out[0], 2)));
//         // let pitch_gyro = atan(I16F16::from_num(-1) * self.gyro_x_out[0] / sqrt::niirf(self.gyro_y_out[0] * self.gyro_y_out[0]  + self.gyro_z_out[0]  * self.gyro_z_out[0], 2));

//         // accel_ypr = YawPitchRoll { yaw: I16F16::from_num(0.0), pitch: pitch_acc, roll:roll_acc };
//         // gyro_ypr  = YawPitchRoll { yaw: I16F16::from_num(0.0), pitch: pitch_gyro, roll:roll_gyro };
//         return ([self.accel_x_out[0], self.accel_y_out[0], self.accel_z_out[0]], [self.gyro_x_out[0],self.gyro_y_out[0],self.gyro_z_out[0]]);

//         // return (accel_ypr, gyro_ypr);

//     }

// }

pub struct LowPassOne {
    pub accel_x_in: [I16F16; 2],
    pub accel_y_in: [I16F16; 2],
    pub accel_z_in: [I16F16; 2],
    pub gyro_x_in: [I16F16; 2],
    pub gyro_y_in: [I16F16; 2],
    pub gyro_z_in: [I16F16; 2],
    pub accel_x_out: [I16F16; 2],
    pub accel_y_out: [I16F16; 2],
    pub accel_z_out: [I16F16; 2],
    pub gyro_x_out: [I16F16; 2],
    pub gyro_y_out: [I16F16; 2],
    pub gyro_z_out: [I16F16; 2],
    pub b: [I16F16; 2],
    pub a: [I16F16; 1],
}
impl LowPassOne {
    pub fn new() -> LowPassOne {
        let i16f16_zero = I16F16::from_num(0.001);
        LowPassOne {
            accel_x_in: [i16f16_zero, i16f16_zero],
            accel_y_in: [i16f16_zero, i16f16_zero],
            accel_z_in: [i16f16_zero, i16f16_zero],
            gyro_y_in: [i16f16_zero, i16f16_zero],
            gyro_x_in: [i16f16_zero, i16f16_zero],
            gyro_z_in: [i16f16_zero, i16f16_zero],
            accel_x_out: [i16f16_zero, i16f16_zero],
            accel_y_out: [i16f16_zero, i16f16_zero],
            accel_z_out: [i16f16_zero, i16f16_zero],
            gyro_x_out: [i16f16_zero, i16f16_zero],
            gyro_y_out: [i16f16_zero, i16f16_zero],
            gyro_z_out: [i16f16_zero, i16f16_zero],
            // b: [I16F16::from_num(0.4527), I16F16::from_num(0.4527)],
            // a: [I16F16::from_num(0.0945)], //sampling=100, cutoff 22
            b: [I16F16::from_num(0.33179), I16F16::from_num(0.33179)],
            a: [I16F16::from_num(0.33643)], //sampling=150, cutoff=22
        }
    }
    pub fn low_pass_one(
        &mut self,
        gyro: [I16F16; 3],
        acc: [I16F16; 3],
    ) -> ([I16F16; 3], [I16F16; 3]) {
        // self.k =self.k +1;
        self.accel_x_in[0] = acc[0];
        self.accel_y_in[0] = acc[1];
        self.accel_z_in[0] = acc[2];
        self.gyro_x_in[0] = gyro[0];
        self.gyro_y_in[0] = gyro[1];
        self.gyro_z_in[0] = gyro[2];

        self.accel_x_out[0] = self.a[0] * self.accel_x_out[1]
            + self.b[0] * self.accel_x_in[0]
            + self.b[1] * self.accel_x_in[1];
        self.accel_y_out[0] = self.a[0] * self.accel_y_out[1]
            + self.b[0] * self.accel_y_in[0]
            + self.b[1] * self.accel_y_in[1];
        self.accel_z_out[0] = self.a[0] * self.accel_z_out[1]
            + self.b[0] * self.accel_z_in[0]
            + self.b[1] * self.accel_z_in[1];

        self.gyro_x_out[0] = self.a[0] * self.gyro_x_out[1]
            + self.b[0] * self.gyro_x_in[0]
            + self.b[1] * self.gyro_x_in[1];
        self.gyro_y_out[0] = self.a[0] * self.gyro_y_out[1]
            + self.b[0] * self.gyro_y_in[0]
            + self.b[1] * self.gyro_y_in[1];
        self.gyro_z_out[0] = self.a[0] * self.gyro_z_out[1]
            + self.b[0] * self.gyro_z_in[0]
            + self.b[1] * self.gyro_z_in[1];

        self.gyro_x_in[1] = self.gyro_x_in[0];

        self.gyro_y_in[1] = self.gyro_x_in[0];

        self.gyro_z_in[1] = self.gyro_x_in[0];

        self.accel_x_in[1] = self.accel_x_in[0];

        self.accel_y_in[1] = self.accel_y_in[0];

        self.accel_z_in[1] = self.accel_z_in[0];

        self.gyro_x_out[1] = self.gyro_x_out[0];

        self.gyro_y_out[1] = self.gyro_y_out[0];

        self.gyro_z_out[1] = self.gyro_z_out[0];

        self.accel_x_out[1] = self.accel_x_out[0];

        self.accel_y_out[1] = self.accel_y_out[0];

        self.accel_z_out[1] = self.accel_z_out[0];

        // let mut accel_ypr :YawPitchRoll = YawPitchRoll { yaw: I16F16::from_num(0.0), pitch: I16F16::from_num(0.0), roll:I16F16::from_num(0.0) };
        // let mut  gyro_ypr :YawPitchRoll = YawPitchRoll { yaw: I16F16::from_num(0.0), pitch: I16F16::from_num(0.0), roll:I16F16::from_num(0.0) };

        // let roll_acc = atan(self.accel_y_out[0]/ (sqrt::niirf(self.accel_x_out[0]* self.accel_x_out[0] + self.accel_z_out[0] * self.accel_z_out[0], 2)));
        // let pitch_acc = atan(I16F16::from_num(-1) * self.accel_x_out[0] / sqrt::niirf(self.accel_y_out[0] * self.accel_y_out[0]  + self.accel_z_out[0]  * self.accel_z_out[0], 2));

        // let roll_gyro = atan(self.gyro_y_out[0]/ (sqrt::niirf(self.gyro_x_out[0]* self.gyro_x_out[0] + self.gyro_z_out[0] * self.gyro_z_out[0], 2)));
        // let pitch_gyro = atan(I16F16::from_num(-1) * self.gyro_x_out[0] / sqrt::niirf(self.gyro_y_out[0] * self.gyro_y_out[0]  + self.gyro_z_out[0]  * self.gyro_z_out[0], 2));

        // accel_ypr = YawPitchRoll { yaw: I16F16::from_num(0.0), pitch: pitch_acc, roll:roll_acc };
        // gyro_ypr  = YawPitchRoll { yaw: I16F16::from_num(0.0), pitch: pitch_gyro, roll:roll_gyro };(
        (
            [
                self.accel_x_out[0],
                self.accel_y_out[0],
                self.accel_z_out[0],
            ],
            [self.gyro_x_out[0], self.gyro_y_out[0], self.gyro_z_out[0]],
        )

        // return (accel_ypr, gyro_ypr);
    }
}

#[derive(Debug, Copy, Clone)]
pub struct KalmanFilter {
    pub bias: [I16F16; 3],
    pub acc_sphi: [I16F16; 3],
    pub out_rate: [I16F16; 3],
    pub out_angle: YawPitchRoll,
    pub integration_constant: I16F16,
    pub c1: I16F16,
    pub c2: I16F16,
    pub new_ypr: YawPitchRoll,
}

impl KalmanFilter {
    pub fn new(c1: I16F16, c2: I16F16) -> KalmanFilter {
        KalmanFilter {
            bias: [I16F16::from_num(0.0); 3],
            acc_sphi: [I16F16::from_num(0.0); 3],
            out_rate: [I16F16::from_num(0.0); 3],
            out_angle: YawPitchRoll {
                yaw: I16F16::from_num(0.0),
                pitch: I16F16::from_num(0.0),
                roll: I16F16::from_num(0.0),
            },
            integration_constant: I16F16::from_num(1 / 150), //1/frequency is time, every tick we fetch the values from MPU6050
            c1,
            c2,
            new_ypr: YawPitchRoll {
                yaw: I16F16::from_num(0.0),
                pitch: I16F16::from_num(0.0),
                roll: I16F16::from_num(0.0),
            },
        }
    }

    pub fn get_kalman_data(
        &mut self,
        mut acc: [I16F16; 3],
        gyro: [I16F16; 3],
        sd: &SensorOffset,
    ) -> YawPitchRoll {
        //todo: pass calibration offsets or pass it after offsetting ?
        //set bias as the calibration offset ?
        // self.bias = 0,0,0?? //get gyro data from sensor --> initializing out rate to this value, no need to store it separately.
        //convert input values to roll and pitch
        acc[0] *= I16F16::from_num(PI / 180);
        acc[1] *= I16F16::from_num(PI / 180);
        acc[2] *= I16F16::from_num(PI / 180);

        let acc_roll = atan(acc[1] / sqrt::niirf(acc[0] * acc[0] + acc[2] * acc[2], 2));
        let acc_pitch = atan(acc[0] / sqrt::niirf(acc[1] * acc[1] + acc[2] * acc[2], 2));
        // acc[0]=I16F16::from_num(0);
        acc[1] = acc_pitch;
        acc[2] = acc_roll;

        self.acc_sphi[0] = acc[0] - self.bias[0]; //get acc data in Yaw,Pitch, Roll from Sensor
        self.acc_sphi[1] = acc[1] - self.bias[1];
        self.acc_sphi[2] = acc[2] - self.bias[2];

        self.out_rate[0] =
            (gyro[2] + I16F16::from_num(sd.gyro_offset[2] as i16)) * I16F16::from_num(PI / 180);
        // self.out_rate[0] = (gyro[2])*I16F16::from_num(PI/180);
        self.out_rate[1] = gyro[1] * I16F16::from_num(PI / 180); // pitch vel
        self.out_rate[2] = gyro[0] * I16F16::from_num(PI / 180); //roll vel

        self.out_angle.yaw -=
            (self.out_rate[0] / I16F16::from_num(16.4)) * I16F16::from_num(0.0017453);
        self.out_angle.pitch += self.out_rate[1] * self.integration_constant;
        self.out_angle.roll += self.out_rate[2] * self.integration_constant;

        // let pitch =  atan2(self.acc_sphi[0], sqrt::niirf(self.acc_sphi[1]*self.acc_sphi[1] + self.acc_sphi[2]*self.acc_sphi[2],2));
        // let roll =  atan2(self.acc_sphi[1], sqrt::niirf(self.acc_sphi[0]*self.acc_sphi[0] + self.acc_sphi[2]*self.acc_sphi[2],2));

        // let error_yaw= self.out_angle.yaw - self.acc_sphi[0];
        let error_pitch = self.out_angle.pitch - self.acc_sphi[1];
        let error_roll = self.out_angle.roll - self.acc_sphi[2];

        // self.out_angle.yaw = self.out_angle.yaw - error_yaw / self.c1;
        self.out_angle.pitch -= error_pitch / self.c1;
        self.out_angle.roll -= error_roll / self.c1;

        // self.bias[0] = self.bias[0] + ((error_yaw*100)/ self.c2);
        self.bias[1] += (error_pitch * 100) / self.c2;
        self.bias[2] += (error_roll * 100) / self.c2;

        //         float sgZ = accZ<0 ? -1 : 1; // allow one angle to go from -180 to +180 degrees
        //   angleAccX =   atan2(accY, sgZ*sqrt(accZ*accZ + accX*accX)) * RAD_2_DEG; // [-180,+180] deg
        //   angleAccY = - atan2(accX,     sqrt(accZ*accZ + accY*accY)) * RAD_2_DEG;
        // let mut sgz=I16F16::from_num(0.0);
        // if self.acc_sphi[2]<0
        // {
        //     sgz= I16F16::from_num(-1.0);
        // }
        // else {
        //     sgz= I16F16::from_num(1.0);
        // }
        // self.new_ypr.yaw=self.out_angle.yaw;
        // self.new_ypr.roll= atan2(self.out_angle.pitch, sgz*sqrt::niirf(self.out_angle.yaw*self.out_angle.yaw+self.out_angle.roll*self.out_angle.roll, 2));
        // self.new_ypr.pitch= atan2(self.out_angle.yaw, sqrt::niirf(self.out_angle.pitch*self.out_angle.pitch+self.out_angle.roll*self.out_angle.roll, 2));

        // Calculating Roll and Pitch from the accelerometer data
        //let roll = atan(self.out_angle.pitch / (sqrt::niirf(self.out_angle.yaw * self.out_angle.yaw + self.out_angle.roll * self.out_angle.roll, 2)));
        // let pitch = atan(I16F16::from_num(-1) * self.out_angle.yaw / sqrt::niirf(self.out_angle.pitch * self.out_angle.pitch + self.out_angle.roll * self.out_angle.roll, 2));
        //   Pitch = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI);
        self.new_ypr.yaw = self.out_angle.yaw - sd.yaw_offset;
        self.new_ypr.roll = self.out_angle.roll - sd.roll_offset;
        self.new_ypr.pitch = self.out_angle.pitch - sd.pitch_offset;
        // self.new_ypr.yaw=self.out_angle.yaw;
        // self.new_ypr.roll=self.out_angle.roll;
        // self.new_ypr.pitch=self.out_angle.pitch;
        // self.new_ypr.yaw=acc[0];
        // self.new_ypr.pitch=acc[1];
        // self.new_ypr.roll=acc[2];
        self.new_ypr
    }

    //     pub fn get_kalman_ypr(&mut self, gyro:[I16F16;3], mut acc:[I16F16;3], sd:&mut SensorData)-> YawPitchRoll
    //     {
    //         acc[0]=acc[0]*I16F16::from_num(PI/180);
    //         acc[1]=acc[1]*I16F16::from_num(PI/180);
    //         acc[2]=acc[2]*I16F16::from_num(PI/180);

    //         acc[0]=acc[0]+sd.calibrated_accel[0];
    //         acc[1]=acc[1]+sd.calibrated_accel[1];
    //         acc[2]=acc[2]+sd.calibrated_accel[2];

    //         let acc_roll = atan(acc[1]/sqrt::niirf((acc[0]*acc[0]+acc[2]*acc[2]), 2)        );
    //         let acc_pitch = atan(-acc[0]/sqrt::niirf((acc[1]*acc[1]+acc[2]*acc[2]), 2)        );
    //         acc[0]=I16F16::from_num(0);
    //         acc[1]=acc_pitch;
    //         acc[2]=acc_roll;

    //         self.acc_sphi[0] = acc[0]; //get acc data in Yaw,Pitch, Roll from Sensor
    //         self.acc_sphi[1] = acc[1];
    //         self.acc_sphi[2] = acc[2]*I16F16::from_num(PI/180);

    //         self.out_rate[0] = gyro[2];
    //         self.out_rate[1] = gyro[1];
    //         self.out_rate[2] = gyro[0]*I16F16::from_num(PI/180);

    //         self.out_angle.yaw = self.out_rate[0] * self.integration_constant ;
    //         self.out_angle.pitch = self.out_angle.pitch + self.out_rate[1] * self.integration_constant;
    //         self.out_angle.roll = self.out_angle.roll + self.out_rate[2] * self.integration_constant;

    //         // let error_yaw= self.out_angle.yaw - self.acc_sphi[0];
    //         let error_pitch= self.out_angle.pitch - self.acc_sphi[1];
    //         let error_roll= self.out_angle.roll - self.acc_sphi[2];

    //         // self.out_angle.yaw = self.out_angle.yaw - error_yaw / self.c1;
    //         self.out_angle.pitch = self.out_angle.pitch - error_pitch / self.c1;
    //         self.out_angle.roll = self.out_angle.roll - error_roll / self.c1;

    //         // sd.calibrated_accel[0]= sd.calibrated_accel[0]+(error_yaw*100)/self.c2;
    //         sd.calibrated_accel[1]= sd.calibrated_accel[1]+(error_pitch*100)/self.c2;
    //         sd.calibrated_accel[2]= sd.calibrated_accel[2]+(error_roll*100)/self.c2;

    //         // self.new_ypr.yaw=self.out_angle.yaw-sd.calibrated_dmp.yaw;
    //         // self.new_ypr.pitch=self.out_angle.pitch-sd.calibrated_dmp.pitch;
    //         // self.new_ypr.roll=self.out_angle.roll-sd.calibrated_dmp.roll;
    //         sd.filter_ypr.yaw=self.out_angle.yaw;
    //         sd.filter_ypr.pitch=self.out_angle.pitch-sd.calibrated_dmp.pitch;
    //         sd.filter_ypr.roll=self.out_angle.roll-sd.calibrated_dmp.roll;
    //         // let mut sgz=I16F16::from_num(0.0);
    //         // if self.acc_sphi[2]<0
    //         // {
    //         //     sgz= I16F16::from_num(-1.0);
    //         // }
    //         // else {
    //         //     sgz= I16F16::from_num(1.0);
    //         // }
    //         // self.new_ypr.yaw=self.out_angle.yaw;
    //         // self.new_ypr.roll= atan2(self.out_angle.pitch, sgz*sqrt::niirf(self.out_angle.yaw*self.out_angle.yaw+self.out_angle.roll*self.out_angle.roll, 2));
    //         // self.new_ypr.pitch= atan2(self.out_angle.yaw, sqrt::niirf(self.out_angle.pitch*self.out_angle.pitch+self.out_angle.roll*self.out_angle.roll, 2));

    // return sd.filter_ypr;
    //     }
    // pub fn fusion_algorithm(&mut self,sensor: &mut Sensor) {
    //     self.acc_sphi = yaw_pitch_roll_from_acc(sensor.data.acceleration+sensor.calibrate_offset.acceleration);
    //     self.out_rate = sensor.data.velocity;
    //     self.out_phi = self.out_phi + self.out_rate * self.integration_constant;
    //     let e = self.out_phi - self.acc_sphi;
    //    // sensor.data.radius = self.out_phi;
    //     self.out_phi = self.out_phi - e / self.c1;
    //     sensor.calibrate_offset.velocity =
    //         sensor.calibrate_offset.velocity + (e / self.integration_constant) / self.c2;
    //     sensor.data.radius = self.out_phi - sensor.calibrate_offset.radius;
    //     sensor.data.radius.yaw = Frac::from_num(0.);
    // }
}
