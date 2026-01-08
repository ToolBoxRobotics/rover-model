Hardware Changes & Wiring Map
I2C Bus (SDA=20, SCL=21): Connect the Arduino Mega to the PCA9685 and the TCA9548A.

Note on Address Conflict: The INA219 and PCA9685 often both default to address 0x40. To avoid this, we will connect the PCA9685 to the Main I2C Bus and hide the INA219 behind the TCA9548A (Port 1).

PCA9685 Connections:

Channels 0-5: Motor PWM Speed (M1, M2, M3, M4, M5, M6).

Channels 6-9: Steering Servos (FL, FR, RL, RR).

Arduino Mega Digital Pins:

Pins 22-27: Motor Direction (DIR) pins. (PCA handles speed, Mega handles Forward/Reverse logic).

Arduino Mega Analog/Interrupt Pins:

A0-A5 & A8-A13: Encoders (Remains unchanged for fast interrupt response).

Updated rover_base.ino
