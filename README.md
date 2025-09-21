# Vortex Stonefish Sim
This repository contains scenario files and models for simulation of the Vortex NTNU drones.

## Installation
Dependencies
```bash
sudo apt update && sudo apt install -y libglm-dev libsdl2-dev libfreetype6-dev
```
Clone Stonefish
```bash
cd /opt
sudo git clone https://github.com/patrykcieslak/stonefish.git
sudo sed -i '30i#include <cstdint>' stonefish/Library/include/sensors/Sample.h
```
Build
```bash
cd stonefish
sudo mkdir build
cd build
sudo cmake ..
sudo make -j4
sudo make install
```
Ensure correct path
```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
```

## ROS 2 package
Clone and build the [Stonefish ROS 2 package](https://github.com/vortexntnu/stonefish_ros2) in your workspace.

## Usage
Clone this repository and build your workspace.

Launch a scenario:

```bash
ros2 launch stonefish_sim simulation.launch.py scenario:=tacc
```

Available scenarios are tacc, docking, pipeline, structure, orca_demo, freya_demo, orca_freya_demo.

![Image](https://drive.google.com/uc?export=view&id=1Mdg5cXCWC3h63GH70mxJqeO6YI-iAcV7)

![Image](https://drive.google.com/uc?export=view&id=1elYr7ipYRVbe5FjxG1w3hYaTKhNYyyDH)


If you have a less capable computer, other options are to use low rendering quality or turn off rendering completely. For low rendering quality use

```bash
ros2 launch stonefish_sim simulation.launch.py rendering_quality:=low
```

and for no rendering use

```bash
ros2 launch stonefish_sim simulation.launch.py rendering:=false
```

## Creating Your Own Scenario

To create your own scenario file, follow these steps:

1. **Define the Scenario File**:
   - Create a new `.scn` file under the `scenario` folder.
   - This file will define the specific scenario you want to simulate.

2. **Import Objects**:
   - Use objects defined in `.scn` files located in the `objects` folder.
   - These objects can represent various elements of the simulation, such as obstacles or environmental features.

3. **Include Data Files**:
   - The object files in the `objects` folder can import data from files located in the `data/object_files` directory.
   - Typical files in `data/object_files` are `.png` textures and `.obj` files.

4. **Define the Scenario Config**
    - Additionaly you have to create a config file in the `config` directory that defines transformations for components.
    - This step allows easy configuration of scenario parameters that will be useful for testing purposes.




