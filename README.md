# pathtrack_tools
A collection of tools for path tracking control

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

- Manage reference course 

### Vehicle Dynamics Simulator

### Frenet State Filter



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



