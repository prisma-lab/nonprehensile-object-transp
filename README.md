# Nonprehensile object transportation

Implementation of the nonprehensile object transportation method developed in the paper:

*[M. Selvaggio](http://wpage.unina.it/mario.selvaggio/index.html), [J. Cacace](http://wpage.unina.it/jonathan.cacace/), [C. Pacchierotti](https://team.inria.fr/rainbow/fr/team/claudio-pacchierotti/), [F. Ruggiero](http://www.fabioruggiero.name/web/index.php/en/), [P. Robuffo Giordano](https://team.inria.fr/rainbow/fr/team/prg/), "[A Shared-control Teleoperation Architecture for Nonprehensile Object Transportation](http://wpage.unina.it/mario.selvaggio/papers/tro2021.pdf)", IEEE Transactions on Robotics (in press), 2021.* 


[![Nonprehensile object transportation](play_video_figure.png)](https://www.youtube.com/watch?v=5_eReIS7Ku4)

### Citing
```
@article{SelvaggioTRO2021,
  title={A Shared-control Teleoperation Architecture for Nonprehensile Object Transportation},
  author={Selvaggio, Mario and Cacace, Jonathan and Pacchierotti, Claudio and Ruggiero, Fabio and Robuffo Giordano, Paolo},
  journal={IEEE Transactions on Robotics (in press)},
  year={2021}
}
```

# Instructions

The code was tested with Ubuntu 20.04, and ROS Noetic. Different OS and ROS versions are possible but not supported.

### Install dependencies

`sudo apt-get install ros-noetic-ros-control ros-noetic-ros-controllers`

### Clone the following packages 

```cd <CATKIN_WS_DIR>/src```

`git clone https://github.com/mrslvg/nonprehensile-object-transp.git`

`git clone https://github.com/ros/geometry.git`

`git clone https://github.com/ros/kdl_parser.git`

### Compile 

`catkin_make --only-pkg-with-deps kdl_ros_control`

`catkin_make -DCATKIN_WHITELIST_PACKAGES=""`

### Run simulation

`roslaunch lbr_iiwa_launch lbr_iiwa_gazebo_effort_control.launch`

`rosrun shared-control-balancing shared-control-balancing-iiwa_node`

Press play in gazebo to start the simulation
