# Introduction to ROS – Course Project Autonomous Driving - Team 9 - Perception Pipline

## Basic dependencies installation

Before running the project, you need to install ROS Noetic at first. You can find the tutorial for installation of ROS Noetic at https://wiki.ros.org/noetic/Installation/Ubuntu. After completion, you need to install the following dependencies:

```bash
sudo apt update
sudo apt install ros-noetic-depth-image-proc \
                 ros-noetic-cv-bridge ros-noetic-image-transport \
                 ros-noetic-octomap ros-noetic-octomap-msgs ros-noetic-octomap-ros \
                 ros-noetic-move-base \
                 ros-noetic-octomap-server \
                 ros-noetic-navigation
```


## octomap_mapping installation

If you haven't make your catkin_ws, please do this at first:

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
```

Then do this to install octomap_mapping package:

```bash
cd ~/catkin_ws/src
git clone https://github.com/OctoMap/octomap_mapping.git
cd ~/catkin_ws
catkin_make_isolated --install
source ~/catkin_ws/install_isolated/setup.bash
```

Add your workspace to ROS_PACKAGE_PATH:

```bash
echo "source ~/catkin_ws/install_isolated/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Use following codes to check if it is installed correctly:

```bash
roscd octomap_mapping
roslaunch octomap_server octomap_mapping.launch
```

In addition, you also need to install a plugin for connection between octomap and rviz:

```bash
sudo apt-get update
sudo apt-get install ros-noetic-octomap-rviz-plugins
```

### Check the output of octomap and costmap package

The following will show you how to get rostopic information related to octomap and costmap.

At first make sure that you are in the right directory of project, then do this:

```bash
catkin build
```

Execute the following code in all subwindows of Terminator:

```bash
source devel/setup.bash
```

To launch the simulation node, run the following command after having ```roscore``` running:

```bash
roslaunch simulation trial.launch
```

After all programs are loaded, enter the following code in another subwindow to view all rostopics, including octomap, move_base and costmap related topics:

```bash
rostopic list
```

For ease of viewing, the output has been copied into ~/I2ROS_team9/project/src/perception_pipeline/rostopics.txt.
And I will keep updating this .txt along with the progress of our project.



