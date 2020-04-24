#Dataset for ROS

###Setup files

Download the kitti dataset [kitti_2011_09_26_drive_0022_synced.bag](https://drive.google.com/open?id=1TJcH-Aw9yD5G5J0doLqCpa9ca22gw0cI)
and place it inside [vslam_user/resources](src/vslam_user/resources).

Also, download the modified NEU dataset with corrected camera_info topics called [final_100.bag]()
and place it inside [vslam_user/resources](src/vslam_user/resources).

These will be the rosbags referenced when using launch files for RTAB-Map.

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

`roslaunch vslam_user start_bag.launch neu:=false`

or for NEU data

`roslaunch vslam_user start_bag.launch neu:=true`

#####Terminal 2: Run RTAB-Map On Live Data

`roslaunch vslam_user analyze.launch neu:=false`

or for NEU data

`roslaunch vslam_user analyze.launch neu:=true`

###Analyze

Run matlab file `analyze_rtab.m` in analysis folder to visualize poses from collected data.

###Extra Information

[**rtabmap.ini**](src/vslam_user/cfg/rtabmap.ini): contains set of parameters to modify how rtabmap processes images and
performs SLAM. Tested various configurations with different feature detection methods such as optical flow, SURF, and
harris corner detection. Also, tested using a mask on the image and changing the distribution of feature detections. 
Could not get rtabmap to predict pose successfully on NEU dataset. KITTI dataset works without changing these 
parameters. Modified and tested various combinations of parameters under the groups Vis, GFTT, and SURF.

[**replace_camera_info.py**](src/vslam_user/src/replace_camera_info.py): python file to modify and correct original
camera_info topics in NEU dataset.
