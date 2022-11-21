# rs_pololu

```
Code Repository of My MSc Course Project

EMATM0054 Robotic Systems

University of Bristol, UK
```

Version description:
* V1.0: Achieved very simple line following
    * motor driver
    * line sensors driver
    * Bang-bang controller
    * Straight line
* V2.0: Achieved improved line following
    * 90 degree cornor
    * 135 degree cornor
    * Finite-state-machine (FSM)
    * utility driver: beep and led
* V3.0: Achieved line following challenge, eared 95% marks for assessment 1 [[video]](https://www.youtube.com/watch?v=ppUrGDie5EU&ab_channel=RoboDDai)
    * encoder driver
    * join the line at 90 degree
    * simple odometry (by count)
    * simple return-to-home (RTH)
* v4.0: Achieved comprehensive line following, baseline for group report
    * wheel speed estimation using encoder
    * low pass filter
    * task scheduler: multi-tasking programming for Arduino
    * cascaded control architecture
        * Heading Controller (PID)->Motor Speed Controller (PID)
* v5.0: LQR-based heading controller