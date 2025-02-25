# MDP
## Group 6 (STM Part)
### Done by [Wei Hao](https://github.com/WeiHaoChin)  and [Bryan](https://github.com/c220142)
 Uses Encoder,Gyroscope, Accelerometer,Servometer,Motor, IR sensor, Ultrasonic sensor to perform required tasks to clear checklist, Task 1 and Task 2 <br />
## Gyroscope and Accelerometer is combined using madgwick filter to stabilise reading for accurate turning <br />
## STM(slave) ← UART → RPI(master) <br />
### Commands and usage
`W`,`A`,`S`,`D` <br />
W100 = Forward 100 cm <br />
S100 = Backward 100 cm <br />
D100 = Forward 90 degrees <br />
D000 = Backward 90 degrees <br />
A100 = Forward 90 degrees <br />
A000= Backward 90 degrees <br />
`ACK` is sent when the execution is completed
## Motor and Servo → Robot movements <br />
## Encoder → Distance measurement and PID control <br />
