# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

### **Vehicle Model**

The model I used in this MPC project is a kinematic bicycle model which neglecting the complex interations between the tires and road. The equations of the model shows below

```
x[t] = x[t-1] + v[t-1] * cos(psi[t-1]) * dt
y[t] = y[t-1] + v[t-1] * sin(psi[t-1]) * dt
psi[t] = psi[t-1] + v[t-1] / Lf * delta[t-1] * dt
v[t] = v[t-1] + a[t-1] * dt
cte[t] = f(x[t-1]) - y[t-1] + v[t-1] * sin(epsi[t-1]) * dt
epsi[t] = psi[t] - psides[t-1] + v[t-1] * delta[t-1] / Lf * dt
```

Here x and y is the car's position, psi is the car's heading direction, v is the car's velocity, cte is the cross-track error, epsi is the orientation error, those values are considered the state of the model. In addition to that, Lf is the distance between the center of mass of the car and the front wheels.

The model input (actuators) is the car's acceleration(throttle) a, and the steering angle delta.

The goal of MPC method is to find a and delta in the way it will minimize the cost function which is the combination of several factors, taking care of the main objective - to minimize our cross track, heading, and velocity errors. A further enhancement is to constrain erratic control inputs, for example, if we're making a turn, we'd like the turn to be smooth, not sharp. Additionally, the vehicle velocity should not change too radically. The goal of this final loop is to make control decisions more consistent, or smoother. The next control input should be similar to the current one

### Timestep Length and Elapsed Duration (N & dt)

The number of points  N and the time interval dt define the prediction horizon T = N * dt. The number of points impacts the performance of controller, short prediction horizon lead to more reponsive controllers, but are less accurate and lead to more instable when chosen too short. Long prediction horizon generally lead to smoother controls. For a given prediction horizon shorter time step dt although lead to more accurate controls, it will slow down the computation bring latency.

In this project, I start to try N =25 and dt =0.05, but it weny wild easily. After try many times, finally I chose N =10 and dt = 0.1(100 milliseconds) it has a better result than others.

### Polynomial Fitting and MPC Preprocessing

The waypoints provided by the simulator are tansformed to the car coordinate system. First shifting the origin to the current position of the car, then a 2D rotation is applied on the relatived waypoints, finally transfer into car coordinate system. 

```
X' =   cos(psi) * (ptsx[i] - x) + sin(psi) * (ptsy[i] - y);
Y' =  -sin(psi) * (ptsx[i] - x) + cos(psi) * (ptsy[i] - y);
```

Here X' and Y' are the waypoints in the vehicle coordinate system.

Then a 3-rd polynomial is fitted to the transformed waypoints. The polynomail coefficients are used to calculate the cte and epsi.

### Model Predictive Control with Latency

There are two common approaches to handle the delays in control problem.

1. Latency is taken into account by constraining the controls to the values of the previous iteration for the duration of the latency. Thus the optimal trajectory is computed starting from the time after the latency period. so the dynamics during the latency period is still calculated according to the vehicle model.
2. The prospective position of the car is estimated based on its current speed and heading direction by propagating the position of the car forward until the expected time when actuations are expected to have an effect. This approach is intuitionistic, and easy to handle.

So to handle the actuator latency, I used the state values which are calculated by the model and delay interval  replace the initial one.

```
x_delay = x0 + (v* cos(psi0) * delay);
y_delay = y0 + (v* sin(psi0) * delay);
psi_delay = psi0 + (v*delta/Lf *delay);
v_delay = v + a* delay;
cte_delay = cte0 + (v*sin(epsi0)*delay);
epsi_delay = epsi0 + (v * delta* delay/Lf);
```

### Simulation

One video recorded has been uploaded here:

https://youtu.be/9pKHX7mxBWg

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

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
