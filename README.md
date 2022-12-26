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



<!-- > **Note:** Since we have shifted from simple drive download to more sophisticated approach of sharing packages, there are some limitation in the normal approach, mainly the size of upload. However, this is solved by using Githhub's LFS (Large File System). And hence, you might face a little delay (depending upon your network) in downloading the files tracked by LSF.  -->

