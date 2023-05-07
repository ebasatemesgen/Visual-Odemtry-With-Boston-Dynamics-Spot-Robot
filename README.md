<img src="./image/spot_gazebo.gif" alt="Spot Robot Simulation with ORB-SLAM3" width="1000" />


# Part 1: Boston Dynamics Spot Robot Setup

This will guide you through setting up the Boston Dynamics Spot robot simulation on your local machine. The instructions provided are for a ROS-based setup using Gazebo as the simulation environment.

## Prerequisites

Make sure you have `sudo` privileges and ROS is already installed on your system.

## Installation Steps

1. Install necessary dependencies:

   ```
   sudo apt install -y python-rosdep
   ```

2. Clone the required repositories:

   ```
   cd ~/catkin_ws/src
   git clone --recursive https://github.com/chvmp/champ
   git clone https://github.com/chvmp/champ_teleop
   git clone -b gazebo https://github.com/chvmp/spot_ros
   git clone https://github.com/chvmp/robots.git
   ```

3. Install ROS dependencies:

   ```
   cd ~/catkin_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

4. Build the workspace:

   ```
   catkin_make
   ```

5. Clone the Gazebo models and worlds collection repository:

   ```
   git clone https://github.com/leonhartyao/gazebo_models_worlds_collection.git
   ```

6. Update the `GAZEBO_MODEL_PATH` and `GAZEBO_RESOURCE_PATH` environment variables in your `~/.bashrc` file:

   ```
   echo "export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:<path to this repo>/models" >> ~/.bashrc
   echo "export GAZEBO_RESOURCE_PATH=\$GAZEBO_RESOURCE_PATH:<path to this repo>/worlds" >> ~/.bashrc
   source ~/.bashrc
   ```

   Replace `<path to this repo>` with the actual path to the `gazebo_models_worlds_collection` repository on your machine.

## Usage

You can now run the Spot robot simulation in Gazebo using the following command:

```
roslaunch spot_gazebo spot.launch world:=<world_file_name>
```

Replace `<world_file_name>` with the name of the Gazebo world you would like to use.

## Customization

You can replace the world file with any of your choice from the `gazebo_models_worlds_collection` repository or create your own custom Gazebo worlds.



# Part 2: ORB-SLAM3-ROS

This section will guide you through setting up the ORB-SLAM3-ROS package for use with the Spot robot simulation from the previous section.

ORB-SLAM3-ROS is a ROS implementation of ORB-SLAM3 V1.0 that focuses on the ROS part. This package uses `catkin_make` and has been tested on Ubuntu 20.04.

## 1. Prerequisites

### Eigen3
```
sudo apt install libeigen3-dev
```

### Pangolin

```
cd ~
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
mkdir build && cd build
cmake ..
make
sudo make install
```

### OpenCV

Check the OpenCV version on your computer (required at least 3.0):

```
python3 -c "import cv2; print(cv2.__version__)"
```

On a freshly installed Ubuntu 20.04.4 LTS with desktop image, OpenCV 4.2.0 is already included. If a newer version is required (>= 3.0), follow installation instruction and change the corresponding OpenCV version in CMakeLists.txt

### (Optional) hector-trajectory-server

Install hector-trajectory-server to visualize the real-time trajectory of the camera/imu. Note that this real-time trajectory might not be the same as the keyframes' trajectory.

```
sudo apt install ros-[DISTRO]-hector-trajectory-server
```

## 2. Installation

```
cd ~/catkin_ws/src
git clone https://github.com/thien94/orb_slam3_ros.git
cd ~/catkin_ws
catkin_make
```
