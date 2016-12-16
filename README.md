# servoiface

DC Servo motor interface

Board use to control a DC motor and act as a servomotor, based on STM32F042 MCU. 

Position feedback can be provided by potentiometer, limit switches or encoder. 

Reference position command can be provided from a PWM signal such as regular RC servomotors, or an halfduplex UART such as Dynamixel smart servomotors.

Motor current limitation can be driven, thus the device can be used as a torque controlled system (useful for robot gripper, for instance).

Debug RGB, tactile switch, and potentiometer can be used for system calibration and visual feedback.

Board:

![board](https://cloud.githubusercontent.com/assets/10212201/21272868/3b424e80-c3c1-11e6-8c1d-a49f0b522402.png)

Schematics:

![sch](https://cloud.githubusercontent.com/assets/10212201/21272883/492f0966-c3c1-11e6-9b20-b8466eae82ae.png)

