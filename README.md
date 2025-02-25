# MDP
## Group 6 (STM) Done by [Wei Hao](https://github.com/WeiHaoChin)  and [Bryan](https://github.com/c220142)
Utilizes a combination of the following components to perform required tasks and clear the checklist for Task 1 and Task 2:
- **Encoder**
- **Gyroscope**
- **Accelerometer**
- **Servomotor**
- **Motor**
- **IR Sensor**
- **Ultrasonic Sensor**
## Gyroscope and Accelerometer is combined using madgwick filter to stabilise reading for accurate turning <br />
## STM(slave) ← UART → RPI(master) <br />
### Commands and usage
`W`,`A`,`S`,`D` ,`RUSD`<br />
`W100` = Forward 100 cm <br />
`S100` = Backward 100 cm <br />
`D100` = Forward 90 degrees <br />
`D000` = Backward 90 degrees <br />
`A100` = Forward 90 degrees <br />
`A000` = Backward 90 degrees <br />
`RUSD` = Return distance from Ultrasonic Sensor <br />
`ACK` is sent when the execution is completed
## Motor and Servo → Robot movements <br />
## Encoder → Distance measurement and PID control <br />
**330 Encoder pulses per revolution**<br />
**Wheel Diameter 6.5cm**<br />
