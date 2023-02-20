
# Quadrupel Project Template

Most of the code we provide you for the Embedded Systems Lab
is not actually contained in this template. Instead, it's provided 
as a library called [`tudelft_quadrupel`](https://docs.rs/tudelft_quadrupel)

In your `Cargo.toml`, its version is set to version `1`. This means, that running 
`cargo update` will automatically get you bugfixes if released by the course staff 
(any version `1.x.x`).

You can find the assignment text on the [course website](https://cese.ewi.tudelft.nl)

## Template layout

Your template consists of two main folders: `dronecode` and `runner`. The compilation target
of these two is different. The `dronecode` code is compiled for `thumbv6m-none-eabi` while the default
target for `runner` is `x86_64-unknown-linux-gnu`.

The `dronecode` will run on your drone, and contains an example on how to write some basic code for the 
drone. `runner` is responsible for uploading the program to your drone, and can then also start any
code that needs to run on the PC to communicate with the drone.

## Our Time Schedule & Checklist

### Lab 1, 21 Feb
#### Before
- [ ] Draft design protocol
- [ ] Farmiliarize with HW

#### After
- [ ] Test designed protocol
- [ ] Implement manual mode

### Lab 2, 28 Feb
#### Before
- [ ] Implement data logging
- [ ] Test data logging

#### After
- [ ] Demonstrate protocol
- [ ] Test manual mode

### Lab 3, 7 Mar
#### Before
- [ ] Implement calibration
- [ ] Implement yaw control (DMP)

#### After
- [ ] Demonstrate manual mode
- [ ] Test calibration
- [ ] Test yaw control (DMP)

### Lab 4, 14 Mar
#### Before
- [ ] Implement roll and pitch control (DMP)
- [ ] Start on filters (Butterworth 1st order, Kalman)
- [ ] Profile your code!

#### After
- [ ] Test and demonstrate yaw mode
- [ ] Test roll and pitch control (DMP)

### Lab 5, 21 Mar
#### Before
- [ ] Continue work on filters (Butterworth 1st order, Kalman)
- [ ] Implement height control
- [ ] Implement wireless

#### After
- [ ] Test and demonstrate full control using DMP
- [ ] Test Kalman filter
- [ ] Test wireless
- [ ] Test height control

### Lab 6, 28 Mar
#### Before
- [ ] Profile your code some more
- [ ] Finalize full control using Kalman

#### After
- [ ] Test and demonstrate full control using Kalman
- [ ] Preliminary demonstration + free advice
- [ ] Fine tuning

### Lab 7, 4 April
#### Before
- [ ] Finalize everything

#### After
- [ ] Formal demonstration and grading
