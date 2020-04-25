## MATLAB


1. First download kittiSLAM and SOFT folders separatly, and then download KITTI stereo images dataset at [here](https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_09_26_drive_0009/2011_09_26_drive_0009_sync.zip) for the first dataset 
and [here](https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_09_26_drive_0022/2011_09_26_drive_0022_sync.zip) for the second one, which is the dataset we used for openvslam and RTAB map.

2. Unzip the zipfile under kittiSLAM and SOFT/data folders. For running kittiSLAM, run kittiSLAM.m in matlab. For running SOFT, run softSLAM.m under /SOFT/code/softSLAM.m
(the original setting is to run with 2011_09_26_drive_0009 dataset. To change the dataset, change the file path at line 17,18 kittiSLAM.m and line 39,40 of softSLAM.m 

(note: increase vo_params.feature.nms_tau at line 17 of softSLAM.m if computation pwer is a limitation).
