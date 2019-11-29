# FA-Harris: A Fast and Asynchronous Corner Detector for Event Cameras
This code is the reference implementation described in the paper "**FA-Harris: A Fast and Asynchronous Corner Detector for Event Cameras**". This work was developed by Ruoxiang Li at [National University of Defense Technology](https://english.nudt.edu.cn).

In this work, we present our event-corner detection method operating directly on the asychronous event-streams. The asychronous event corner detector, called FA-Harris, works based on the Surface of Active Events structure.
We propose a global SAE construction and updating method and an efficient candidate selection and refinement strategy.
And the FA-Harris algorithm runs 8 times faster than the novel event-based Harris detector.
When considering the accuracy, our method achieves good performance compared to the previous methods.

```
This code has been tested with ROS kinetic on Ubuntu 16.04.
```

## Video

<a href="https://youtu.be/v5CcBVkmI6w" target="_blank"><img src="http://img.youtube.com/vi/v5CcBVkmI6w/0.jpg" 
alt="FA-Harris detector" width="480" height="360" border="10" /></a>

## Publication
If you use this work, please cite the following [publication](https://ruoxianglee.github.io/FA-Harris): 

Ruoxiang, Li and Dianxi, Shi and Yongjun, Zhang and Kaiyue, Li and Ruihao, Li. "**FA-Harris: A Fast and Asynchronous Corner Detector for Event Cameras.**" IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 2019. 

``` 
@inproceedings{li2019fa,
author = {Li, Ruoxiang and Shi, Dianxi and Zhang, Yongjun and Li, Kaiyue and Li, Ruihao},
year = {2019},
month = {09},
pages = {6223--6229},
title = {FA-Harris: A Fast and Asynchronous Corner Detector for Event Cameras}
}
```

## Installation
Requirements: 
* [Eigen 3](https://eigen.tuxfamily.org/dox/) `sudo apt-get install libeigen3-dev`

ROS-related requirements:
* [ROS Kinetic](http://wiki.ros.org/kinetic) 
* [DVS/DAVIS ROS driver](https://github.com/uzh-rpg/rpg_dvs_ros) 

## Run

Clone the repository to your ROS workspace and build it:

    $ cd /path/to/catkin_ws/src
    $ git clone https://github.com/ruoxianglee/fa_harris
    $ roscd fa_harris
    $ catkin build --this

Connect your DAVIS camera and launch the following file:

    $ roslaunch fa_harris corner.launch

Or, you can also play a rosbag file from the [Event Camera Dataset](http://rpg.ifi.uzh.ch/davis_data.html):

    $ roslaunch fa_harris corner.launch rosbag_flag:=1 rosbag_path:=/path/to/ros_bag.bag

## Contact
Please, create an issue if you have questions or bug reports. If you come up with any improvements, please create a pull request. Alternatively, you can also contact me at ruoxianglee@163.com.

## Acknowledgements

Many thanks to Ignacio Alzugaray and Elias Mueggler who shared their code for event-corner detection:

- [arc_star_ros](https://github.com/ialzugaray/arc_star_ros)
- [rpg_corner_events](https://github.com/uzh-rpg/rpg_corner_events)
