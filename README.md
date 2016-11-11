# Tiva C Quadcopter Controller

Hey guys!
This is a project page of my quadcopter FC build. I'm using Texas Instruments Tiva C TM4C123G microcontroller. 
Quadcopter was built from various parts I found on Hobbyking. I think part names don't matter, 
so I'll just post the most important info about them.

In the end of this file I'll post some notes and tricks that I learned by trial and error. I hope they help you! 

|           |             |       |
| --------- | ----------- | ----- |
| Props:    | 10x4.5 |
| Motors:   | 950kV |
| Battery:  | 5200mAh |
| ESC:      | Afro ESC or any ESC that has 1000Hz refresh rate |
| Frame:    | 550X from Hobbyking |
| TX/RX:    | Turnigy TGY-i6 and iA6 6 channel reciever |
| Acc/Gyro: | MPU6050 |
| Sonar:    | HC-SR04 |

Quad flies in x-config.

## OUTPUT PINS:
|              |                                                    |
| ------------ | -------------------------------------------------- |
| I2C:         | PB2 - scl , PB3 - sda                              |
| UART:        | PA0 - rx , PA1 - tx                                |
| MPU6050:     | same as I2C (doooooh), INT pin from mpu6050 to PE2 |
| ESC(motors): | PB4, PB5, PB6, PB7                                 |
| RX:          | PA2, PA3, PA4, PA5, PA6, PA8                       |

This is a project that I started about one year ago and It took some really hard work to get it airborne. 
So I would appreciate any comments, suggestions or anything at all.

## NOTES:
- MPU connection fails sometimes. In practical terms that means that if you are not cautious, your quadcopter might do quad   backflip and kiss your head. If you see that it has some strange behaviour, just unplug power wire and reset MPU. It should work fine then. Note: I'm planning to add somesort of failsafe so that this does not happen.

- Vibrations matter! They can mess up your motor's nuts and bolts (that means your motor might fall of mid-flight) and stronger vibrations mean less accurate sensor readings. So always make sure your props are balanced.

- Keep quadcopter center of gravity as close to center of quadcopter. This is important becuse it can mess with PID controller, which in term makes PID tuning impossible.

- PID. The Uh and Oh and tears. Sorry to tell you guys, but without a mathematical model, you'll have a bad time tuning the PID. Remember, you start from scratch. You won't have any idea at which numbers to start. Also, because I use cascaded PID, there are 5 parameters to tune. (rate PD and stabilize PID). And if that is not enough, your quadcopter will probably have different weight distribution on pitch and roll axis. That makes 10 parameters to tune. Have fun!
