# **_Sahayak Bot_**

Please find the base package for this theme from this repository. 



> This package is tested on **Ubuntu 20.04, ROS-Noetic**. 


### [ROS installation](https://atom-robotics-lab.github.io/wiki/markdown/ros/installation.html)

### Additional Packages 

- "Teleop" package to control `ebot` or any model (all you would need to do is change the message type, do explore in your spare time) from your terminal.

```bash
sudo apt install ros-noetic-teleop-twist-keyboard
```
* Moveit is required for the [UR5 Arm](https://github.com/ros-industrial/universal_robot). It can be installed using 
```bash
sudo apt install ros-noetic-moveit
```

Make sure controller manager is also installed by running the following command:
```bash
sudo apt install ros-noetic-controller-manager
```

### Cloning this repository

Inside your `cakin_ws/src`, enter the following command to clone (download) this repository. 

```bash
git clone repo_url
```

After that you can build the workspace using 
```bash 
catkin_make
```

This step will take time as it installs several models from [Ignition Fuel](https://app.gazebosim.org/dashboard) for the simulation.

### Yolo & Virtualenv Setup

* We use Yolo for object recognition with python. For this we need to setup a virtual environment. For this you could use: `virtualenv` and [`virtualenvwrapper`](https://virtualenvwrapper.readthedocs.io/en/latest/). You can use any other package for setting up the virtual environment like venv, conda etc.

* For using the yolo model we need to install:

```bash
pip install opencv-contrib-python
```

### Launching the Simulation
 * Launch the hospital simulation world:
  ```bash
  roslaunch aws_robomaker_hospital_world hospital.launch
  ```
 ![hospital_simulation_world](https://user-images.githubusercontent.com/23265149/209863291-0fbcd979-58d8-4dc2-b832-a7ffd0f48f0b.png)

 
 * Launch robot_bringup launch file which launches moveit and rviz for visualisation:
 ```bash
 roslaunch ebot_perception ebot_bringup.launch
 ```
 
 * You can also run `rqt_image_view` and select the respective topic to view camera images:
 ```bash
 rosrun rqt_image_view rqt_image_view
 ```
 
 * Launch ebot navigation stack:
 ```bash
 roslaunch ebot_nav navigation.launch
```
 
  * Launch `ebot_handler` launch file:
  ```bash
  roslaunch ebot_handler ebot_handler.launch
  ```
 
 * Activate your yolo python virtual environment. Navigate to your workspace in the terminal before running this. Run perception yolo script:
 ```bash
 cd catkin_ws && rosrun ebot_perception perception_yolo_new.py
 ```
 
 * Now we can go on with publishing a task for the robot:
 ```bash
 rostopic pub /task ebot_handler/Task "pick_room: 'Store_Room'
ob_name: 'GLASS'
drop_room: 'Reception'"
```
 

<!-- > **Note:** Since we have shifted from simple drive download to more sophisticated approach of sharing packages, there are some limitation in the normal approach, mainly the size of upload. However, this is solved by using Githhub's LFS (Large File System). And hence, you might face a little delay (depending upon your network) in downloading the files tracked by LSF.  -->

