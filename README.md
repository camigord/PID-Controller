# PID Controller Project
Self-Driving Car Engineer Nanodegree Program


<img src="./assets/video1.gif?raw=true" width="320"><img src="./assets/video2.gif?raw=true" width="320">


## Overview

Implementing a PID controller to "control" both the steering angle and the speed of a simulated vehicle.

## Tuning PID parameters

I implemented two PID controllers:

  - _Position controller_: steering the vehicle in order to keep it in the middle of the road. The controller tries to minimize the lateral distance to the center of the road.
  - _Speed controller_: accelerating or breaking in order to keep a predefined velocity. The controller will therefore try to minimize the difference between the current speed and the desired velocity (set to 25 MPH after some experimental evaluation).

Controlling the speed is very important given that as the speed increases, it also becomes harder to control the steering of the vehicle (specially in curves). Moreover, without a speed controller is not really possible to tune the _position controller_ parameters.

I started by defining a very simple PID as a _speed controller_ by setting _K<sub>p</sub> = 2, K<sub>d</sub> = 5, K<sub>i</sub> = 0_. This was enough to maintain a velocity very similar to the desired one.

Having this _speed controller_ I moved on to tune the _position controller_ parameters  using the _Twiddle Algorithm_. After tuning, I got the following parameters:

  - Kp_position = 1.0
  - Kd_position = 17.4119
  - Ki_position = 0.017179

Now it is time to tune the _speed controller_. To do this, I set the _position controller_ parameters to the values provided before and I use _Twiddle_ once more, but this time on the _speed controller_ parameters.

In the case of the _speed controller_ and when evaluating the performance of a set of parameters, I defined the _error_ of a trial as:

`
error = 0.7*avg_pos_controller_error + 0.3*avg_speed_controller_error
`

The intuition behind including the _position controller_ error was to relax the constraints on the _speed controller_ performance as long as the car was kept as close to the middle of the road as possible. It is better for the velocity to go down if that allows the _position controller_ to do its job.

The final parameters for the _speed controller_ are:

- Kp_speed = 2.0
- Kd_speed = 13.8514
- Ki_speed = 0.00891601

## Running the Code
The project employs the simulation environment provided by UDACITY which can be downloaded from [here](https://github.com/udacity/self-driving-car-sim/releases).

This repository includes two files that can be used to set up and intall [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. ./clean.sh
2. ./build.sh
3. ./run.sh
