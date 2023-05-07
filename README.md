
# Boston Dynamics Spot Robot Setup

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
