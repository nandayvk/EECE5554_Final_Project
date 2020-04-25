# EECE5554_FinalProject

Final Project for EECE5554 performing analysis on various packages using Visual SLAM.

By Blake McHale, Ziyi Yang, and Nanday Kerkar

Refer to the powerpoint [Final Project - VSLAM](https://drive.google.com/open?id=1G9HaCIgNlhQMFVMM0QGIlX2SZOuhA7G60rU7nMprKnE)
for results from package analysis. Please read notes at the bottom of each slide to understand project better.

Packages used for this project were OpenVSLAM, RTAB-Map, and SOFT-Odometry. Descriptions regarding each package and how
to run them are below in their respective sections.

## Table of Contents
<!--ts-->
   * [EECE5554_FinalProject](#eece5554_finalproject)
      * [Table of Contents](#table-of-contents)
      * [OpenVSLAM](#openvslam)
      * [Dataset for ROS](#dataset-for-ros)
         * [Setup files](#setup-files)
      * [RTAB-Map](#rtab-map)
         * [Additional Packages Used To Run NEU Dataset](#additional-packages-used-to-run-neu-dataset)
         * [Installation](#installation)
         * [Building](#building)
         * [Running](#running)
            * [Generate Map](#generate-map)
               * [Terminal 1: Run RTAB-Map On Live Data](#terminal-1-run-rtab-map-on-live-data)
               * [Terminal 2: Start playing rosbag](#terminal-2-start-playing-rosbag)
            * [Perform Localization](#perform-localization)
               * [Terminal 1: Run RTAB-Map On Live Data](#terminal-1-run-rtab-map-on-live-data-1)
               * [Terminal 2: Start playing rosbag](#terminal-2-start-playing-rosbag-1)
         * [Analyze](#analyze)
         * [Files Modified/Created](#files-modifiedcreated)
         * [Launch Files](#launch-files)
         * [Extra Information](#extra-information)
      * [Package Comparison](#package-comparison)

<!-- Added by: bmchale, at: Fri Apr 24 22:25:10 EDT 2020 -->

<!--te-->

## OpenVSLAM

[Documentation](https://openvslam.readthedocs.io/en/master/index.html), [GitHub](https://github.com/xdspacelab/openvslam)

The OpenVSLAM files are located in the [openvslam](openvslam) directory of this repository. The installation procedure 
along with other details are mentioned locally in that directory. You can find a README.md specific to openvslam at
[openvslam/README.md](openvslam/README.md).

## Dataset for ROS

### Setup files

Download the kitti dataset [kitti_2011_09_26_drive_0022_synced.bag](https://drive.google.com/open?id=1TJcH-Aw9yD5G5J0doLqCpa9ca22gw0cI)
and place it inside [src/vslam_user/resources](src/vslam_user/resources).

Also, download the modified NEU dataset with corrected camera_info topics called [final_100.bag](https://drive.google.com/open?id=1FOw9sHZXFzR8m16sh4c17T6liJyaFkjg)
and place it inside [src/vslam_user/resources](src/vslam_user/resources).

These will be the rosbags referenced when using launch files for RTAB-Map.

## RTAB-Map

[**rtabmap_ros**](https://github.com/introlab/rtabmap_ros): Main package used for performing visual SLAM.

### Additional Packages Used To Run NEU Dataset

**Note**: These are already installed in this repository

[**image_undistort**](https://github.com/ethz-asl/image_undistort): Stereo package to perform equidistant distortion
model rectification. ROS Image Processing package only supports plumb bob and radial tangent distortion models. Image 
undistort depends on ROS, OpenCV and Eigen.

[**static_transform_mux**](https://github.com/tradr-project/static_transform_mux): Static tf republisher to correct
issues replaying tf frames from /tf_static in rosbag.

### Installation

`sudo apt install libnlopt-dev` (Needed for image_undistort)

`sudo apt-get install ros-melodic-rtabmap-ros`

### Building

Build the workspace with catkin at the root of the repository.

`cd /path/to/eece5554_finalproject`

`catkin_make`

### Running

RTAB-Map needs to first build a map by running without localization for one complete cycle of the data. This map will 
then be used when running with localization in the second cycle. Follow the steps below in order, first building the map
and playing the bag once, then localizing pose and playing the bag again. If you want to record pose estimations pass
the argument record to start_bag.launch and set it to true.

Make sure to call `source devel/setup.bash` inside every new terminal created.

It is recommended to run rtabmap before playing the bag data, so that the entire dataset is captured.

#### Generate Map

**NOTE**: Follow these steps first and then jump to Perform Localization section.

##### Terminal 1: Run RTAB-Map On Live Data

`roslaunch vslam_user analyze.launch neu:=false localization:=false`

or for NEU data

`roslaunch vslam_user analyze.launch neu:=true localization:=false`

##### Terminal 2: Start playing rosbag

`roslaunch vslam_user start_bag.launch neu:=false`

or for NEU data

`roslaunch vslam_user start_bag.launch neu:=true`

#### Perform Localization

##### Terminal 1: Run RTAB-Map On Live Data

`roslaunch vslam_user analyze.launch neu:=false localization:=true`

or for NEU data

`roslaunch vslam_user analyze.launch neu:=true localization:=true`

##### Terminal 2: Start playing rosbag

`roslaunch vslam_user start_bag.launch neu:=false`

or for NEU data

`roslaunch vslam_user start_bag.launch neu:=true`

### Analyze

Run matlab file `analyze_rtab.m` in analysis folder to visualize poses from collected data.


### Files Modified/Created

Created package vslam_user that contains launch files for starting rosbag and running RTAB-Map processes. Also, has
launch files for creating disparity images and point clouds from NEU dataset.

Modified files in image_undistort to manipulate images after they are rectified. Specifically, added lines 287-319 in 
[stereo_undistort.cpp](src/image_undistort/src/stereo_undistort.cpp).

### Launch Files

Two main launch files exist **start_bag.launch** and **analyze.launch**.

**start_bag.launch arguments**

neu - boolean flag that plays neu dataset when set to true and the KITTI dataset when set to false

record - boolean flag that records a rosbag of all pose topics when set to true

**analyze.launch arguments**

neu - boolean flag that reads from neu dataset when set to true and the KITTI dataset when set to false

localization - boolean flag that tells rtabmap to perform only pose localization using map and visual odometry when set
to true, when false it builds a map and does not localize pose

use_viso - boolean flag that uses viso2 to perform visual odometry instead of default GFTT algorithm in rtab when set to
true

### Extra Information

[**rtabmap.ini**](src/vslam_user/cfg/rtabmap.ini): contains set of parameters to modify how rtabmap processes images and
performs SLAM. Tested various configurations with different feature detection methods such as optical flow, SURF, and
harris corner detection. Also, tested using a mask on the image and changing the distribution of feature detections. 
Could not get rtabmap to predict pose successfully on NEU dataset. KITTI dataset works without changing these 
parameters. Modified and tested various combinations of parameters under the groups Vis, GFTT, and SURF.

[**replace_camera_info.py**](src/vslam_user/src/replace_camera_info.py): python file to modify and correct original
camera_info topics in NEU dataset. Uses updated camera intrinsics and extrinsics given to us.

## MATLAB


1. First download kittiSLAM and SOFT folders separatly, and then download KITTI stereo images dataset at [here](https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_09_26_drive_0009/2011_09_26_drive_0009_sync.zip) for the first dataset 
and [here](https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_09_26_drive_0022/2011_09_26_drive_0022_sync.zip) for the second one, which is the dataset we used for openvslam and RTAB map.

2. Unzip the zipfile under kittiSLAM and SOFT/data folders. For running kittiSLAM, run kittiSLAM.m in matlab. For running SOFT, run softSLAM.m under /SOFT/code/softSLAM.m
(the original setting is to run with 2011_09_26_drive_0009 dataset. To change the dataset, change the file path at line 17,18 kittiSLAM.m and line 39,40 of softSLAM.m 

(note: increase vo_params.feature.nms_tau at line 17 of softSLAM.m if computation pwer is a limitation).

## Package Comparison

Packages were compared with their generated poses. A plot can be generated comparing the SOFT Odometry results to 
RTAB-Map's results by running `analyze_soft_rtab.m` in Matlab. Pose's were not compared for OpenVSLAM because there was
an issue retrieving poses outside the Pangolin viewer.


