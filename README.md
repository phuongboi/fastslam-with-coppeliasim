### This repository is my re-implementation of FastSLAM with Pioneer robot and SICK lidar in CoppeliaSim(V-REP)
* My implemantaion is the combine of 2 repository [1](https://github.com/andriusbern/slam) and [2](https://github.com/kunnnnethan/FastSLAM),
where I used room scenerio from [1] and modify the scrip to use ROS interface instead of V-rep python API, I used FastSLAM for grid mapping from [2] using data from V-rep instead of modeling data.
#### [3/11/2023] First commit
* Using map resolution 150x150 (grid resolution is 10cm) to reduce computation complexity
* Sampling 54 among 270 sensor of SICK TIM310
* Using real odometry information
#### TODO
* Fix wrong data transformation
* Convert from velocity signal to odometry
##### CoppeliaSim simulation
![alt text](https://github.com/phuongboi/fastslam-with-coppeliasim/blob/main/result/recording_2023_11_03-20_28-15.gif)
##### Output Map
![alt text](https://github.com/phuongboi/fastslam-with-coppeliasim/blob/main/result/map.gif)   
### Requirements
* CoppeliaSim v4.5.1 linux
* ROS Noetic, rospy
* OpenCV
### Setup
* Launch `roscore` in one terminal before launch Coppeliasim in another terminal to make sure that CoppeliaSim can load ROS plugin properly
* Open vrep_scenario/room_d.ttt in CoppeliaSim and modify child_script of Pioneer_p3dx by v_rep_scenario/rosInterface_slam.lua
* Start CoppeliaSim simulation, make sure topics is work as expect by `rostopic list`
* Run `python fastslam1.py`

### Reference
* [1] https://github.com/andriusbern/slam
* [2] https://github.com/kunnnnethan/FastSLAM
* [3] Montemerlo, Michael, et al. "FastSLAM: A factored solution to the simultaneous localization and mapping problem." Aaai/iaai 593598 (2002).
* [4] Grisetti, Giorgio, Cyrill Stachniss, and Wolfram Burgard. "Improved techniques for grid mapping with rao-blackwellized particle filters." IEEE transactions on Robotics 23.1 (2007): 34-46.
