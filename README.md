# darc_f1tenth_system

This repository contains drivers and configuration files for operating the UC Davis F1tenth Autonomous Race Car.

## Setup Instructions

Follow the F1tenth instructions for initially setting up your car [here](https://f1tenth.org/build.html).

### Dependencies

For the UC Davis F1tenth, on the Jetson Nano, you need the following dependencies. Ensure you have followed the F1tenth Build instructions carefully and installed ROS 2 Foxy [here](https://docs.ros.org/en/foxy/Installation.html). Also, make sure to install NoMachine [here](https://www.nomachine.com/).

### Set Up Your f1tenth_ws

```bash
# Create your ROS 2 workspace
mkdir -p ~/f1tenth_ws/src
cd ~/f1tenth_ws/src

# Clone the code for running algorithms
git clone https://github.com/ian-chuang/f1tenth_gym_ros.git
cd ~/f1tenth_ws/src/f1tenth_gym_ros
git submodule init
git submodule update

# Clone driver and configuration files for running on the real car
cd ~/f1tenth_ws/src
git clone https://github.com/ian-chuang/darc_f1tenth_system.git

# Install ROS 2 dependencies
cd ~/f1tenth_ws
sudo apt-get update
rosdep install -i --from-path src --rosdistro foxy -y
```

> **Warning**: We experienced issues with `rosdep install` where it wouldn't install the packages even though they were properly listed in the `package.xml`. If you encounter this, install the packages manually with `sudo apt-get install ros-foxy-<name-of-the-package>`.

### Install Particle Filter Dependency

```bash
cd ~
sudo pip install cython
git clone https://github.com/f1tenth/range_libc
cd range_libc/pywrapper
./compile_with_cuda.sh  # On the car - compiles GPU ray casting methods
```

### Build ROS 2 Workspace

```bash
cd ~/f1tenth_ws
colcon build

# Source ROS 2
source /opt/ros/foxy/setup.bash && source ~/f1tenth_ws/install/setup.bash

# Optionally, add sourcing workspace to bashrc (so you don't have to call it every time)
echo 'source /opt/ros/foxy/setup.bash && source ~/f1tenth_ws/install/setup.bash' >> ~/.bashrc
```

## Teleop Car Instructions

Open a terminal and start up the car with the command below. This will start up the VESC, LiDAR, and joystick control.

```bash
ros2 launch f1tenth_stack bringup_launch.py
```

Control the car with the joystick. Hold down the LB button and use the left joystick for acceleration and the right joystick for steering.

## SLAM Instructions

There are a couple of prerequisites to get autonomous control working. First, you need a map with SLAM.

1. Start up your car:

    ```bash
    ros2 launch f1tenth_stack bringup_launch.py
    ```

2. Open another terminal and run the command below:

    ```bash
    ros2 launch f1tenth_stack slam_launch.py
    ```

This will start an RViz window showing the SLAM map running. Drive the car around the race track and SLAM will automatically map the race track. Once you sufficiently map the entire track, save the map by clicking the "Save Map" button in RViz. This saves both the YAML and PGM file of the map in the folder you launched SLAM.

## Raceline Optimization Instructions

After running SLAM, you need to generate a raceline to go around the map. This process is CPU intensive, so use a laptop or computer other than the Jetson to run it.

1. Clone the Raceline Optimization repository:

    ```bash
    git clone https://github.com/ian-chuang/Raceline-Optimization.git
    ```

2. Follow the instructions in the README file in that repository to generate your raceline CSV.

## Setting Up Configuration Files

1. Move the map SLAM generated to the `maps` folder in `f1tenth_stack/maps` (make sure it is the raw unmodified map).
2. Move the raceline CSV to the `racelines` folder in `f1tenth_stack/racelines`.

You'll need to modify the `.yaml` files in `f1tenth_stack/config` to properly run the car. The main things you need to change are:

- `particle_filter.yaml`: Change `map_yaml_path` to the path of your map `.yaml` file.
- `pure_pursuit.yaml`: Change `trajectory` to the path of your raceline CSV.
- `obs_detect.yaml`: Change `spline_file_name` to the path of your raceline CSV.

## Running Autonomous Control

To run autonomous control, it is recommended you use NoMachine to connect to the Jetson remotely so that you can control the car while it is disconnected from the monitor. When you disconnect the HDMI, make sure to replace the HDMI cord with the dummy HDMI plug connector. This fakes the HDMI connection and will allow NoMachine to stream the desktop.

To connect to NoMachine, your computer and Jetson need to be connected to the same WiFi network. Campus WiFi has lots of protections and won't allow you to connect, so use the router in the lab instead. If you are having issues connecting to NoMachine, it could be because of a network setting issue. Because the LiDAR is connected through Ethernet and its fixed at IP `192.168.0.10`, it can mess with both the WiFi connection as well as the NoMachine connection. The workaround is to keep the LiDAR at `192.168.0.10` but set the static IP of both the laptop and Jetson Nano to be in the subnet of `192.168.1.XXX`. You might lose your internet connection, but NoMachine should work as long as both machines are on the same subnet of `255.255.255.0`.

1. First, start up your car:

    ```bash
    ros2 launch f1tenth_stack bringup_launch.py
    ```

2. Open another terminal and run the command below:

    ```bash
    ros2 launch f1tenth_stack particle_filter_launch.py
    ```

This starts up the particle filter. You'll see RViz pop up showing a visualization of the map as well as the location of the particle filter odometry.

> **Important**: The particle filter requires a good start pose estimate to work properly. Place the car on the race track and in RViz, set the approximate location of the real car in the simulation by first clicking the "2D Pose Estimate" button in RViz and then clicking and dragging on a location in the map to set the pose.

3. Finally, launch the autonomous algorithm. Open another terminal and type:

    ```bash
    ros2 launch f1tenth_stack autonomous_launch.py
    ```

This will start up our autonomous control suite (pure pursuit, gap follow, obstacle detection). To run the car autonomously, hold down the RB button on the joystick and the car will start running autonomously.

## Common Problems

- If you get an error when you launch `ros2 launch bringup_launch.py` that the VESC dies, especially when you move the steering wheel back and forth, that means the battery is really low. Just replace or recharge your battery. Most issues can be traced back to a low battery.
- If you are having WiFi and/or Bluetooth connection issues, your WiFi antenna might be broken. Try replacing it first.
- Restarting the Jetson is always an option that sometimes works, as well as just rebuilding your ROS 2 workspace:

    ```bash
    cd ~/f1tenth_ws
    rm -rf build install log
    colcon build
    ```
