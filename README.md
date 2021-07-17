# pathtrack_tools
A collection of tools for path tracking control

TODO: Add some controllers (pure pursuit, LQR, MPC)

## Contents

### Frenet Serret Converter

Transformation between Frenet-Serret coordinate system and global coordinate system.

```c++
pathtrack_tools::FrenetSerretConverter frenet_serret_converter;
// Global coordinate to Frenet-Serret coordinate
const FrenetCoordinate pose_frenet = frenet_serret_converter.global2frenet(cource_manager.get_mpc_cource(), pose_global);
// Frenet-Serret coordinate to Global coordinate
const Pose pose_global = frenet_serret_converter.frenet2global(cource_manager.get_mpc_cource(), pose_frenet);
```



### Course Manager

- Set reference courses from csv
- Reference course: reference waypoint(x,y), reference speed and lane width
- Calculate the curvature of the reference path.
- Fast call by managing in hash

```c++
pathtrack_tools::CourseManager course_manager;
course_manager.set_cource_from_csv("<csv-path>");

// x_f : pose x in Frenet-Serret coordinate
const double path_curvature = course_manager.get_curvature(x_f);
const double reference_speed = course_manager.get_speed(x_f); 
const double lane_width = course_manager.get_drivable_width(x_f);
```



### Vehicle Dynamics Simulator

- Vehicle motion simulator using Kinematic Bicycle model for low speed and Dynamic Bicycle model for high speed.
- Input delay support
- TODO: Upper and lower limits of the control input, dead zone of the steer, and noise on the observed value

```c++
// Observed info of ego vehicle
Pose ego_pose_global; 
Twist ego_twist;

// calculated control input (tire_angle, accel)
double control_input_vec[EGO_INPUT::DIM];

 pathtrack_tools::VehicleDynamicsSimulator vehicle_dynamics_simulator(sampling_time);

// Update ego vehicle pose and twist
const auto [updated_ego_pose_global, updated_ego_twist] = vehicle_dynamics_simulator.update_ego_state(current_time, ego_pose_global, ego_twist, control_input_vec, sampling_time);

```



### Frenet State Filter

- Since the derivative of y_f in the Fresnay coordinate system cannot be transformed from the observed information, estimate
- TODO: Implement the Kalman filter.

```c++
const auto estimated_dy_f = frenet_state_filter.estimate_dy_f(ego_pose_frenet.y_f);
```



## Requirements

- Ubuntu 18.04 or higher
- gcc
- Eigen3
- cmake 3.13 or higher



## Preinstall

1. Install gcc, Eigen

   ```bash
   $ sudo apt install build-essential
   $ sudo apt install libeigen3-dev
   ```

2. Install cmake 3.13 or higher
   https://cmake.org/download/
   ※ NOTE : You have to add installed new cmake path to .bashrc

   ```bash
   export PATH=$HOME/cmake-xxxx/bin/:$PATH
   ```




## How to Build example

Example source code is [here](example/main.cpp)

```bash
$ cd pathtrack_tools/
$ mkdir build
$ cd build/
$ cmake ..  -DCMAKE_BUILD_TYPE=Release
$ make
```



