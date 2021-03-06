# Extended Kalman Filter Project
Self-Driving Car Engineer Nanodegree Program

This project utilizes a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements.  It is run using the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

---

## Src Files
The organization of classes within these files differs from that provided for the project.  Specifically, the KalmanFilter class within src/kalman_filter.cpp contains only private members.  This change was made to correct the Feature Envy (coupling) present between the classes.  The Kalman Filter variables that vary based on sensor type, H measurement matrix and R measurement covariance, are provided to the class as parameters by the FusionEKF class.  Should more sensors need to be added, the FusionEKF class could become the interface class for the Template Design Pattern. 

---

## Runtime

INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)

OUTPUT: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]

---

## Dependencies

* uWebSocketIO for either Linux or Mac systems
* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF `
