# Spot Tutorial

Here are the steps to download this repository and test out the spot on your own computer. 

## Installation

Unity 2020.3.21f1 or later is needed for the Unity project
For ROS, I use Ubuntu 18.04 with Melodic installed
### Install Unity 

If you do not have Unity 2020.3.21f1 or later, add the latest 2020.3 release through [Unity Hub](https://unity3d.com/get-unity/download). Follow these links for [Ubuntu 18.04](https://releases.ubuntu.com/18.04/) and [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu). 

## Setup Unity Project

Download this repo and open up the spot_simulation project in Unity. If not already there, open the SpotScene. 

Check to make sure the spot1 game object is added to the scripts in Subscribers and Publishers

Under the tab Robotics -> ROS Settings set this to the IP address you are using in ROS. You can find this IP address by typing hostname -I in a Linux terminal. If you are having issues with this, watch my first Unity Robotics Hub [video](https://www.youtube.com/watch?v=HV1v8mXNmLA) for more details on how to get this working. 

### Clone the necessary ROS components into your ROS environment

In your Linux environment, make a new directory and call it whatever you like. Then move into the workspace and create a src/ directory. 

```sh
mkdir spot_ws
cd spot_ws/
mkdir src
cd src
```

### Clone the ROS-TCP-Endpoint into the src directory 
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

Source your workspace

```sh
source devel/setup.bash
```


### Set your ROS IP addresses

Set your ROS_MASTER_URI and ROS_IP address to the same IP address or the messages won't send between ROS and Unity

```sh
export ROS_MASTER_URI=http://(IP Address):11311
export ROS_IP=(IP Address)
```


## Running the Code

There are two ways you can test your spot. The first is using the bag file I provided in this repo in the ROS folder. The second is by using the [spot_simulation](https://github.com/SoftServeSAG/spot_simulation/tree/spot_control) code you cloned above. (Although I'm still working on getting the spot robot to move correctly using this second option)

Launch the ROS tcp endpoint

```sh
roslaunch ros_tcp_endpoint endpoint.launch
```

In your Unity environment, press play. You should see the topics show up on your terminal. 

### (OPTION 1) Running the provided bag file

Open up a new terminal, source your workspace again. Then copy the bags folder from this repository into your src/ directory. Then play the bag file.

```sh
cd src/bags
rosbag play joint_data.bag
```

Now you should see the robot moving in Unity

### (OPTION 2) Running the [spot_simulation](https://github.com/SoftServeSAG/spot_simulation/tree/spot_control) scripts

Open up and source four terminals in your Linux environment then run these launch files in order

Start a controller:

```sh
roslaunch rs_inverse inverse.launch robot_name:="spot1"
```

Run the quadruped controller to use Twist() abd Pose() messages:

```sh
roslaunch rs_base quadruped_controller.launch 
```

Run the [teleop_legged_robot](https://github.com/SoftServeSAG/teleop_legged_robots.git) launch file: 

```sh
roslaunch teleop_legged_robots teleop.launch robot_name:="spot1"
```

See the list of controls on [teleop_legged_robot](https://github.com/SoftServeSAG/teleop_legged_robots.git). 

## Getting option 2 working better

I have found a couple things help get option two working:

1. If you activate the Subscribers gameobject in Unity, I have foot collisions as well as model states publishing to the controller scripts. This seems to make it a bit more stable.
2. With the linear velocity set to 10 and the angular velocity set to 1.5 on the teleop_legged_robot script, it moves more normally but still drifts to the left. 
3. If you are using a laptop, and are plugged into a seperate monitor, unplug it. This can sometimes make the command behavior more stable. 



