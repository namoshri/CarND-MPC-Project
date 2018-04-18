# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

# Model Predictive Controller

## Dependencies and prerequisites

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
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Introduction
Implement Model Predictive Control to drive the car around the track.

## Rubik points
### Code and Compilation
Code is available at [github](https://github.com/namoshri/CarND-MPC-Project) .
Code is ready to execute without any errors.

### Model

**State**
Below are state parameters model considers
- x , y positions, 
- ψ - orientation of car
- v- velocity of car
- psi Yaw angle
- CTE - the cross track error
- epsi - orientation error.

First four parameters are given as input from simulator. As position of car given in map co-ordiantes converted it into vehicle co-ordinates.


**Actuators**
Below are two actuators considered
- δ - steering angle 
- a acceleration 

Lf (constant) distance of center of mass and front wheel of car


**Update equations**
Below are equations expressed by model:
x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
psi_[t+1] = psi[t] + v[t] * delta[t] / Lf * dt
v_[t+1] = v[t] + a[t] * dt
cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt


These kinematic model equations helps to get new state from previous state.

### Timestep Length and Elapsed Duration (N & dt)
There is always trade-off for choosing tunned values timestamps here.
A good approach for setting N, dt is to first determine a reasonable range prediction horizon T. T is product of N and dt. Based on T dt and N tuned appropriately. 
Chosen 1 secons as prediction horizon and after tunning 
N set to 10,
dt to 0.1

### Polynomial Fitting and MPC Preprocessing

coeffs = polyfit(ptsx, ptsy, 1);

As mentioned in lectures, 3rd degree polynomial fitted for vehicle co-ordinates positions.

Further, these input and trajecory reference given to MPC solver to get result solution back to feed to simulator.

As mentioned in lesson, lower and upperbound constraints set for each vars. 
FG_eval used to get next possible state.

### Model Predictive Control with Latency
Actuation command won't execute instantly.  A realistic delay might be 100 ms added . Same replicated to state before giving input to MPC solver. Latency here added to all states as seen implemented in main.cpp state vector formation. Latency addition contributed to get better speed. 

### Simulation
 Click here to view output : [video](https://www.youtube.com/watch?v=9fwxipZF-io)
 
 Click here to view car track with latency [video](https://youtu.be/FWmcnvKR-Qg)




