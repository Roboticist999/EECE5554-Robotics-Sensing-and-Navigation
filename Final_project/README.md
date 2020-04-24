In order to run the modified LeGO-LOAM C++ code

You need all dependencies listed on the following 2 websites: basically ROS, gtsam, PCL, Ceres Solver

https://github.com/HKUST-Aerial-Robotics/A-LOAM

https://github.com/RobustFieldAutonomyLab/LeGO-LOAM

After dependency installation,

Configure "includePath" to the same as that in "/Final_project/EECE5554_project_ws/src/LeGO-LOAM/LeGO-LOAM/src/.vscode/c_cpp_properties.json". I don't know whether it is required for "catkin_make", but if you don't add those paths, your IDE could prompt error for including header files.

$ cd /path/to/EECE5554_project_ws

$ catkin_make -j1

$ source /path/to/EECE5554_project_ws/devel/setup.bash

$ roslaunch lego_loam run.launch

$ rosbag play *.bag --clock /ns1/velodyne_points:=/velodyne_points

Set topics to subscribe to in Rviz as I did in videos in our report.

If you didn't do any substitution of cpp files, you will run the code of the second moving feature filtering method introduced in the slide.

Everytime you do substitution on cpp files, you need to run $ catkin_make -DCATKIN_WHITELIST_PACKAGES="lego_loam" to build lego_loam package.

"modified_legoloam_source4" contains source code of the second moving feature filtering method introduced in the slide. If you want to run it, use files in it to substitute those in "Final_project/EECE5554_project_ws/src/LeGO-LOAM/LeGO-LOAM/src"

"modified_legoloam_source1" contains source code of the first moving feature filtering method introduced in the slide. If you want to run it, use files in it to substitute those in "Final_project/EECE5554_project_ws/src/LeGO-LOAM/LeGO-LOAM/src"

"original_legoloam_source" contains original source code of LeGO-LOAM. If you want to run it, use files in it to substitute those in "Final_project/EECE5554_project_ws/src/LeGO-LOAM/LeGO-LOAM/src"


Acknowledgements: Thanks to the authors of LeGO-LOAM, A-LOAM and LOAM.

