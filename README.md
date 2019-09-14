# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program


## Overview

This project implements a PID controller for the car and visulizes it in the Udacity's simulator, the link to which can be found below in the dependencies section. The simulator sends the error in steer angle and the speed to the PID program using WebSocket and receives the steering angle normalized to [-1 1] and throttle normalized to [-1 1]. The PID script uses uWebSockets WebSocket implementation. Udacity provides a seed project to start from on this project [(here)](https://github.com/udacity/CarND-Controls-PID).

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

Fellow students have put together a guide to Windows set-up for the project [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/Kidnapped_Vehicle_Windows_Setup.pdf) if the environment you have set up for the Sensor Fusion projects does not work for this project. There's also an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3).

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

## Implementation

### The PID algorithm flows as what was taught in the lessons
The PID implementation was done in the following steps.
1. Implement PID controller for Steering 
2. Implement PID for Throttle
3. Parameter tuning for Steering and Throttle.

###  PID Controller Implementation

For the controller for Steering controller, I implemented a PID controller but for the Throttle, I implemented a PI controller. The actual implementation of code for a basic PID/PI controller is fairly straightforward, but the performance of the controller mainly depends on how well the controller parameters are tuned, for which knowledge of each of the P,I and D components is required.Lets look into each of this component seperately.

Proportional component: As the name suggest it is just a proportional gian to the error, error between the set point and process variable. That means if we have a system with higher proportional gain the response to the error will be faster. But the drawback is that there will be a very high overshoot, which also leads to higher oscillations in the system response, which can result in unstability and in cases can also lead the sytem to go out of control. For the car the example can be seen here


Integral component: As the name suggests it integrates all the errors from the start. The integral gain is multiplied by the integral error. Logically speaking since it is integral error the integral gain is very low in magnitude compared to the proportional and the derivative component. The result is that even a small error term will cause the integral component to increase slowly. The integral response will continually increase over time unless the error is zero, so the effect is to drive the Steady-State error to zero. Steady-State error is the final difference between the process variable and set point. The integral componenet in effective while the car is taking turns. For the car eample can be found here. 

Derivative component:A derivative term as the term suggest responds to the rate of change of error, trying to bring this rate to zero. It aims at flattening the error trajectory into a horizontal line, damping the force applied, and so reduces overshoot. havind a high derivative gain when the error is small will lead to overshoot. After overshooting, the controller will apply a large correction in the opposite direction and repeatedly overshoot the desired position, the output would oscillate around the setpoint in either a constant, growing, or decaying sinusoid. For the car exmaple can be found here.

### Parameter Tuning

To tune the parameters i went with the manual tuning approach,Increase the Kp until the output of the loop oscillates, then the Kp should be set to approximately half of that value for a "quarter amplitude decay" type response. Then increase Ki until any offset is corrected in sufficient time for the process. However, too much Ki will cause instability. Finally, increase Kd, if required, until the loop is acceptably quick to reach its reference after a load disturbance. However, too much Kd will cause excessive response and overshoot. The following table summerizes the effect of each parameter on the system.


| Parameter  | Rise Time   | Overshoot  | Settling Time   | Steadystate error  |
|---|---|---|---|---|
| Kp  | Decrease  | Increase  | Small change  | Decrease  |
| Ki  | Decrease  | Increase  | Increase  | Decrease  |
| Kd  | Small change  | Decrease  | Decrease  | No change  |


The final parameters for my controllers are: 

|   | Steering  | Speed  |
|---|---|---|
| Kp  |  0.1 |  0.1 |
| Ki  | 0.0045  |  0.002 |
| Kd  | 1.5  |  0.0 |

