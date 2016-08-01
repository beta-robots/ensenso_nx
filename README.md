THIS PACKAGE IS UNDER DEVELOPMENT. NOT YET READY !!!

### Overview
This repository holds code of a [ROS](http://www.ros.org) package for point cloud acquisition with  [EnsensoNx](https://en.ids-imaging.com/ensenso-stereo-3d-camera.html) 3D cameras. It is basically a ROS wrapper of the low-level API provided by [IDS](https://en.ids-imaging.com), the manufacturer of the camera. 

### Dependencies
The package has been tested with the following dependencies:
* Ubuntu 14.04
* CMake + gcc
* [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu)
* [Point Cloud Library v1.7](http://www.pointclouds.org/) (shipped with ROS Indigo)
* Ensenso SDK (propietary library from manufacturer IDS)
* UEYE driver 

To install Ensenso SDK dependency, the following steps are required: 

1. Download the SDK from the [IDS website](http://www.ensenso.com/support/sdk-download/) (file EnsensoSDK-1.3.180-x64.deb)
2. Install it with
```shell 
$ sudo dpkg -i EnsensoSDK-1.3.180-x64.deb
```

You also need the ueye driver and tools:
1. Download the UEYE from the [IDS website](http://www.ensenso.com/support/sdk-download/) (file uEye_4.80.2_Linux_64.tgz)
2. Uncompress, move to the folder and run the script (ethernet or usb as needed)
```shell 
$ sudo sh ./ueyesdk-setup-4.80-eth-amd64.gz.run
```

### Download
```shell
$ git clone https://github.com/beta-robots/ensenso_nx.git
```

### Build
```shell
$ catkin_make --only-pkg-with-deps ensenso_nx
```

### Camera IP config
to do ...



