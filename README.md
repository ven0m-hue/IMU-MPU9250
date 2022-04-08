# IMU-MPU9250
Driver for MU9250 9-DoF Inertial Measurement Sensor.

MPU9250 consists of MEMS based 3 Dof Accelerometer, Magnetometer, Gyroscope.

Intended use - STM32 based microcontroller. Developed using STM32 CubeIDE.

Suports only I2C at the momment. 

uses, PB6 - SCL || PB7 - SDA


@Update: Currently I am switching from Embedded C to Cpp for a loads of reason. 
Transforming the MPU9250 driver file from C code to Cpp for STM32. 
Might also plan on adding features for SPI protocol. 
Might also plan on adding a MPU9250 Wrapper. 
