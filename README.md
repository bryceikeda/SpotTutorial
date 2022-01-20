# Spot Tutorial

Here are the steps to download this repository and test out the spot on your own computer. 

## Installation

Unity 2020.3.21f1 or later is needed for the Unity project
For ROS, I use Ubuntu 18.04 with Melodic installed
### Install Unity 

If you do not have Unity 2020.3.21f1 or later, add the latest 2020.3 release through [Unity Hub](https://unity3d.com/get-unity/download). Follow these links for [Ubuntu 18.04](https://releases.ubuntu.com/18.04/) and [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu). 

### Clone the Spot Tutorial repository and Move the spot_simulation folder

In your Linux environment, make a new directory and call it whatever you like. Then move into the workspace.

```sh
mkdir spot_ws
cd spot_ws/
```

Within the directory, clone this SpotTutorial repository

```sh
git clone https://github.com/bryceikeda/SpotTutorial.git
```

Move the Unity spot_simulation folder elsewhere since it will not be used in ROS. 

### Clone the ROS-TCP-Endpoint into the src directory 

Change directories into the src folder. 

```sh
cd src
```

Then, clone the [ROS-TCP-Endpoint](https://github.com/Unity-Technologies/ROS-TCP-Endpoint) repository. 

```sh
git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git
```

Also clone the [spot_simulation](https://github.com/SoftServeSAG/spot_simulation/tree/spot_control) repository. 

```sh
git clone -b spot_control https://github.com/SoftServeSAG/spot_simulation.git
```

Lastly, clone the [teleop_legged_robot](https://github.com/SoftServeSAG/teleop_legged_robots.git) repository.

```sh
git clone https://github.com/SoftServeSAG/teleop_legged_robots.git
```

### Compile the code

```sh
cd ..
catkin_make
```

## Running the Code

There are two ways you can test your spot. The first is using the bag file I provided in this repo. The second is by using the [spot_simulation](https://github.com/SoftServeSAG/spot_simulation/tree/spot_control) code you cloned above. (Although I'm still working on getting the spot robot to move correctly using this second option)

### Setting your IP address

In Unity, open up the SpotScene from the scenes folder

Under the tab Robotics -> ROS Settings set this to the IP address you are using in ROS. You can find this IP address by typing hostname -I in a linux terminal. If you are having issues with this, watch my first Unity Robotics Hub [video](https://www.youtube.com/watch?v=HV1v8mXNmLA) for more details on how to get this working. 

In your ROS environment, navigate to the params.yaml file and put your same IP address into this file.

```sh
gedit src/ROS-TCP-Endpoint/config/params.yaml
```

### Connecting ROS and Unity

In your ROS environment, source your workspace

```sh
source devel/setup.bash
```

Then launch the ROS tcp endpoint

```sh
roslaunch ros_tcp_endpoint endpoint.launch
```

In your Unity environment, press play. You should see the topics show up on your terminal. 

### Running the provided bag file

Open up a new terminal, source your workspace again and navigate to the bags folder and run the bag.

```sh
cd src/bags
rosbag play joint_data.bag
```

Now you should see the robot moving in Unity

### Running the [spot_simulation](https://github.com/SoftServeSAG/spot_simulation/tree/spot_control) scripts

Open up and source four terminals in your Linux environment then run these launch files in order

Start a controller:

```sh
roslaunch rs_inverse inverse.launch robot_name:="spot1"
```

Run the data converter (Due to some issues with Unity Robotics Hub that they are fixing, you need to run this script for better robot behavior):

```sh
roslaunch unity_data_conversion unity_data_conversion.launch
```

Run the quadruped controller to use Twist() abd Pose() messages:

```sh
roslaunch rs_base quadruped_controller.launch 
```

Run the [teleop_legged_robot](https://github.com/SoftServeSAG/teleop_legged_robots.git) launch file: 

```sh
roslaunch roslaunch teleop_legged_robots teleop.launch robot_name:="spot1"
```
