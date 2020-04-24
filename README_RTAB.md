#RTAB-Map
[**rtabmap_ros**](https://github.com/introlab/rtabmap_ros): Main package used for performing visual SLAM.

###Additional Packages Used To Run NEU Dataset
**Note**: These are already installed in this repository

[**image_undistort**](https://github.com/ethz-asl/image_undistort): Stereo package to perform equidistant distortion
model rectification. ROS Image Processing package only supports plumb bob and radial tangent distortion models. Image 
undistort depends on ROS, OpenCV and Eigen.

[**static_transform_mux**](https://github.com/tradr-project/static_transform_mux): Static tf republisher to correct
issues replaying tf frames from /tf_static in rosbag.

###Installation
`sudo apt install libnlopt-dev` (Needed for image_undistort)

`sudo apt-get install ros-melodic-rtabmap-ros`

###Running
#####Terminal 1: Start playing rosbag

`roslaunch vslam_user start_bag.launch`

or for NEU data

`roslaunch vslam_user start_bag.launch neu:=true`

#####Terminal 2: Run RTAB-Map

`roslaunch vslam_user analyze.launch`

or for NEU data

`roslaunch vslam_user analyze.launch neu:=true`


