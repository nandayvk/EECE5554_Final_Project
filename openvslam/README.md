# OpenVSLAM: A Versatile Visual SLAM Framework

## Installation

### Source Code

The source code can be viewed from this [GitHub repository](https://github.com/xdspacelab/openvslam).

### Dependencies

OpenVSLAM requires a C++11-compliant compiler. It relies on several open-source libraries as shown below.

**Requirements for OpenVSLAM**

[Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) : version 3.3.0 or later.

[g2o](https://github.com/RainerKuemmerle/g2o) : Please use the latest release. Tested on commit ID [9b41a4e](https://github.com/RainerKuemmerle/g2o/tree/9b41a4ea5ade8e1250b9c1b279f3a9c098811b5a).

[SuiteSparse](http://faculty.cse.tamu.edu/davis/suitesparse.html) : Required by g2o.

[DBoW2](https://github.com/shinsumicco/DBoW2) : Please use the custom version of DBoW2 released in [https://github.com/shinsumicco/DBoW2](https://github.com/shinsumicco/DBoW2).

[yaml-cpp](https://github.com/jbeder/yaml-cpp) : version 0.6.0 or later.

[OpenCV](https://opencv.org/) : version 3.3.1 or later.

### Prerequisites

Install the dependencies via `apt`.

```
sudo apt update -y
sudo apt upgrade -y --no-install-recommends
# basic dependencies
sudo apt install -y build-essential pkg-config cmake git wget curl unzip
# g2o dependencies
sudo apt install -y libatlas-base-dev libsuitesparse-dev
# OpenCV dependencies
sudo apt install -y libgtk-3-dev
sudo apt install -y ffmpeg
sudo apt install -y libavcodec-dev libavformat-dev libavutil-dev libswscale-dev libavresample-dev
# eigen dependencies
sudo apt install -y gfortran
# other dependencies
sudo apt install -y libyaml-cpp-dev libgoogle-glog-dev libgflags-dev

# (if you plan on using PangolinViewer)
# Pangolin dependencies
sudo apt install -y libglew-dev

# (if you plan on using SocketViewer)
# Protobuf dependencies
sudo apt install -y autogen autoconf libtool
# Node.js
curl -sL https://deb.nodesource.com/setup_6.x | sudo -E bash -
sudo apt install -y nodejs
```

**Note**: If a shell script fails because of permission issues try running commands within the script yourself and specifically
run `sudo make install` instead of `make install` at the end

Install Eigen

```
cd /path/to/openvslam/libraries/eigen/eigen-eigen-5a0156e40feb/
source /path/to/openvslam/Eigen.sh
```

Build and install OpenCV

```
cd /path/to/openvslam/libraries/open_cv/opencv-3.4.0/
source /path/to/openvslam/OpenCV_install.sh
```

Build and install the custom DBoW2

```
cd /path/to/openvslam/libraries/DBoW2/
source /path/to/openvslam/DBoW2_install.sh
```

Build and install g2o

```
cd /path/to/openvslam/libraries/g2o/
source /path/to/openvslam/g2o_install.sh
```

Build and install Pangolin

```
cd /path/to/openvslam/libraries/Pangolin/
source /path/to/openvslam/Pangolin_install.sh
```

If you use Ubuntu 18.04 (or later), Protobuf 3.x can be installed via apt or brew.

`sudo apt install -y libprotobuf-dev protobuf-compiler`

Otherwise, please download, build and install Protobuf from source using the below command.

`source /path/to/openvslam/Protobuf_install.sh`

### Build Instructions

```
cd /path/to/openvslam
source building.sh
```

## Download Data

Download the zip file containing the data from [OpenVSLAM Data.zip](https://drive.google.com/file/d/1lH1wCRZSXyrt8wjAXuWWRakEQCIolC9r/view?usp=sharing) and place the individual folders for NEU data and KITTI data in /openvslam/build/ directory of the package created after after building it.

## Execution

### To run the KITTI dataset

```
cd /path/to/openvslam/build/

# run tracking and mapping
./run_image_slam -v ./orb_vocab/orb_vocab.dbow2 -i ./kitti_2_bag/images/ -c ./kitti_2_bag/config.yaml --frame-skip 1 --no-sleep --map-db map_kitti.msg
# click the [Terminate] button to close the viewer
# you can find map.msg in the current directory

# run localization
./run_image_localization -v ./orb_vocab/orb_vocab.dbow2 -i ./kitti_2_bag/images/ -c ./kitti_2_bag/config.yaml --frame-skip 1 --no-sleep --map-db map_kitti.msg
```

### To run the Northeastern University dataset

```
cd /path/to/openvslam/build/

# run tracking and mapping
./run_video_slam -v ./orb_vocab/orb_vocab.dbow2 -m ./neu_data_edited/output.mp4 -c ./neu_data_edited/config.yaml --frame-skip 1 --no-sleep --map-db map_neu.msg
# click the [Reset] button if track of features is lost to try to get new points
# click the [Terminate] button to close the viewer
# you can find map.msg in the current directory

# run localization
./run_video_localization -v ./orb_vocab/orb_vocab.dbow2 -m ./neu_data_edited/output.mp4 -c ./neu_data_edited/config.yaml --frame-skip 1 --no-sleep --map-db map_neu.msg
```


### Note

Please see [**Installation**](https://openvslam.readthedocs.io/en/master/installation.html) chapter in the [documentation](https://openvslam.readthedocs.io/) if any issues are necountered with the above steps.


