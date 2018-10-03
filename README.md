# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

[//]: # (Image References)
[equations]: ./images/model_equations.png "Model equations"

## Overview
This is my implementation of project 5 of term 2 of the Udacity - Self-Driving Car NanoDegree. You can find the original repo under [CarND-MPC-Project](https://github.com/udacity/CarND-MPC-Project).

## Implementation
This part will cover the criteria defined in the rubric points.

### The Model
This MPC uses a kinematic bicycle model and can be found in `FG_eval` in the file `MPC.cpp`. The model includes the vehicles x and y coordinates, orientation angle (psi), velocity (v), the cross-track error (cte) as well as the psi error (epsi). The calculated output are the actuators acceleration and steering angle (delta). The model uses the state and actuations from a timestep to predict the next one. This calculation is based on the following equations:

![Kinematic model equations][equations]

`Lf` is is the distance between the center of mass of the vehicle and the front wheels. The value of 2.67 was provided by Udacity for this project.

The model is non-linear as it takes changes of heading direction into account. With this in mind we can use [Ipopt](https://projects.coin-or.org/Ipopt) as non-linear solver to find a local optima given the constraints and cost function also defined in `FG_eval`.

### Timestep Length and Elapsed Duration (N & dt)
The values for N and dt are 7 and 0.15. Together they define the prediction horizon as `T = N * dt`, which is 1.05 seconds in my solution. In this timespan the solver tries to find a trajectory to get as near to the waypoints as possible under the given cost function. Other values tried where 10/0.15, 20/0.05, 15/0.12 and 10/0.12 and more. But all of them did lead to worse results. 

### Polynomial Fitting and MPC Preprocessing
With the provided function `polyfit` a polynomial is fitted to the provided waypoints (main.cpp, line 104). Before that, the waypoints are converted/transformed to the vehicle perspective. This simplifies the fitting of a polynomial as psi and the vehicles x and y coordinates are all zero.
Additionally, the latency is given to the model as a preprocessing step. This is covered in the next section.

### Model Predictive Control with Latency
As explained in the [Udacity forum](https://discussions.udacity.com/t/how-to-incorporate-latency-into-the-model/257391/2), the latency can be predicted before feeding them into the solver. I used the equations provided in the [Udacity forum](https://discussions.udacity.com/t/how-to-incorporate-latency-into-the-model/257391/4) to take a latency of 100 ms into account. You can find the equations in `main.cpp`, line 85 to 93, right before converting the waypoints into the vehicle coordinate system.

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
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
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
       +  Some Mac users have experienced the following error:
       ```
       Listening to port 4567
       Connected!!!
       mpc(4561,0x7ffff1eed3c0) malloc: *** error for object 0x7f911e007600: incorrect checksum for freed object
       - object was probably modified after being freed.
       *** set a breakpoint in malloc_error_break to debug
       ```
       This error has been resolved by updrading ipopt with
       ```brew upgrade ipopt --with-openblas```
       per this [forum post](https://discussions.udacity.com/t/incorrect-checksum-for-freed-object/313433/19).
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/).
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `sudo bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.
