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
apt update -y
apt upgrade -y --no-install-recommends
# basic dependencies
apt install -y build-essential pkg-config cmake git wget curl unzip
# g2o dependencies
apt install -y libatlas-base-dev libsuitesparse-dev
# OpenCV dependencies
apt install -y libgtk-3-dev
apt install -y ffmpeg
apt install -y libavcodec-dev libavformat-dev libavutil-dev libswscale-dev libavresample-dev
# eigen dependencies
apt install -y gfortran
# other dependencies
apt install -y libyaml-cpp-dev libgoogle-glog-dev libgflags-dev

# (if you plan on using PangolinViewer)
# Pangolin dependencies
apt install -y libglew-dev

# (if you plan on using SocketViewer)
# Protobuf dependencies
apt install -y autogen autoconf libtool
# Node.js
curl -sL https://deb.nodesource.com/setup_6.x | sudo -E bash -
apt install -y nodejs
```

Download and install Eigen from source.

```
cd /path/to/working/dir
source /path/to/package/Eigen.sh
```

Download, build and install OpenCV from source.

```
cd /path/to/working/dir
source /path/to/package/OpenCV_install.sh
```

Download, build and install the custom DBoW2 from source.

```
cd /path/to/working/dir
source /path/to/package/DBoW2_install.sh
```

Download, build and install g2o.

```
cd /path/to/working/dir
source /path/to/package/g2o_install.sh
```

Download, build and install Pangolin from source.

```
cd /path/to/working/dir
source /path/to/package/Pangolin_install.sh
```

If you use Ubuntu 18.04 (or later), Protobuf 3.x can be installed via apt or brew.

`apt install -y libprotobuf-dev protobuf-compiler`

Otherwise, please download, build and install Protobuf from source using the below command.

`source /path/to/package/Protobuf_install.sh`

### Build Instructions

```
cd /path/to/openvslam
source /path/to/package/building.sh
```

## Execution

### To run the KITTI dataset

```
cd /path/to/openvslam/build/

# run tracking and mapping
./run_image_slam -v ./orb_vocab/orb_vocab.dbow2 -i ./kitti_2_bag/images/ -c ./kitti_2_bag/config.yaml --frame-skip 1 --no-sleep --map-db map.msg
# click the [Terminate] button to close the viewer
# you can find map.msg in the current directory

# run localization
./run_image_localization -v ./orb_vocab/orb_vocab.dbow2 -i ./kitti_2_bag/images/ -c ./kitti_2_bag/config.yaml --frame-skip 1 --no-sleep --map-db map.msg
```

### To run the Northeastern University dataset

```
cd /path/to/openvslam/build/

# run tracking and mapping
./run_video_slam -v ./orb_vocab/orb_vocab.dbow2 -m ./neu_data_edited/output.mp4 -c ./neu_data_edited/config.yaml --frame-skip 1 --no-sleep --map-db map.msg
# click the [Reset] button if track of features is lost to try to get new points
# click the [Terminate] button to close the viewer
# you can find map.msg in the current directory

# run localization
./run_video_localization -v ./orb_vocab/orb_vocab.dbow2 -m ./neu_data_edited/output.mp4 -c ./neu_data_edited/config.yaml --frame-skip 1 --no-sleep --map-db map.msg
```

## Download Data

Download the zip file containing the data from [OpenVSLAM Data.zip](https://drive.google.com/file/d/1lH1wCRZSXyrt8wjAXuWWRakEQCIolC9r/view?usp=sharing) and place the individual folders for NEU data and KITTI data in /openvslam/build/ directory of the package after building it.

### Note

Please see [**Installation**](https://openvslam.readthedocs.io/en/master/installation.html) chapter in the [documentation](https://openvslam.readthedocs.io/) if any issues are necountered with the above steps.


