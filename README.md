# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---
## Introduction
The project required to create a program that would navigate a car safely through the simulated highway environment. 

The telemetry data provided by the simulator to the program would include an error measured as a deviation from the optimal trajectory and the program would use the error and the PID controller to adjust car’s steering to minimize the error and thus guide a car as close as possible to the optimal trajectory.

## Implementation and Tuning
The implementation of the PID controller is quite simple and it involves returning a desired <steer_value> based on the current error obtained from the simulator and calculated adjustments based on the current error, the previous error and the accumulated error. 


**_P_** - Proportional control
The proportional term computes an adjustment proportional to the cross-track error.  When used by itself it has a tendency to overcorrect and results in large oscillations.

**_D_** - Derivative control
The oscillations caused by Proportional control can be smoothed out by using a term proportional to the change of the cross-track error. 

**_I_** - integrated control
To deal with inherent drift (bias) of the system a third term may be used to adjust the next correction in proportion to the total accumulated error so far.


The difficult part is to find the coefficients for the adjustments that would allow the error to converge to its minimum and stay there for the duration of the simulated drive. To find the coefficients the TWIDDLE algorithm was used.

Given that without proper steering the car just drives off the highway it was difficult to use the TWIDDLE right away. There was no time for it to properly find the right coefficients. A lot of tuning was done by tweaking the initial P, D, I coefficients manually.  From my personal runs it was evident that the coefficient for the error difference term D should be larger than the one for the current error term P. The only way I was able to make TWIDDLE to converge and come up with a result that would keep a car on a highway for a full circle was to make the dp[P] parameter for the P term to start from the number  much smaller than 1 while starting the dp[D] parameter for the D term from 1.  The I term did not make much difference and it is logical assuming the simulator has no inherent bias/drift.  
The coefficients found by TWIDDLE were then used to run a car and it stayed on a highway for a full cycle.

## Performance
Though the car stays on a highway for a full cycle the performance is sometimes is jerky. In tricky places it overshoots the optimal trajectory and then tries to come back.

A short video  is included here:

![ alt text ](./Simulator.gif "")

## Reflection
Given how near-perfect the TWIDDLE-found coefficients performed in a sample with a straight line I am guessing that the simulator telemetry data may be too sparse or the steering is naturally not a real-time adjustment and has a time lag.

Because of the jerky performance I did not try to maximize the car speed as it most likely would make the matters worse. I did attempt to come up with heuristics that would make the car run smoother. For example I tried to do a “progressive” steer calculation where depending on the magnitude of the current CTE I would scale the coefficients of the PID controller. For example:

// progressive PID

    if (fabs(p_error_) < 0.2)
    	result = 0;
    else if (fabs(p_error_) < 2) // 70% steer
    	result = -Kp_ * 0.7 * p_error_ - Kd_ * 0.7 * d_error_ - Ki_ * i_error_;
    else if (fabs(p_error_) < 5) // 80% steer
    	result = -Kp_ * 0.8 * p_error_ - Kd_ * 0.8 * d_error_ - Ki_ * i_error_;
    else if (fabs(p_error_) < 8) // 90% steer
    	result = -Kp_ * 0.9 * p_error_ - Kd_ * 0.9 * d_error_ - Ki_ * i_error_;
    else // 100%
    	result = -Kp_ * 1.0 * p_error_ - Kd_ * 1.0 * d_error_ - Ki_ * i_error_;

But I did not observe any noticeable improvement.

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
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
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

