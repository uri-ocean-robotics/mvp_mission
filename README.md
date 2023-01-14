# MVP Mission Package

This repository contains MVP-Helm program and several useful behaviors.

## Installation

Pull the `mvp_msgs` repository if you don't have it already
```bash
git clone --single-branch --branch noetic-devel https://github.com/uri-ocean-robotics/mvp_msgs
```

Pull the repository
```bash
git clone --sigle-branch --branch noetic-devel https://github.com/uri-ocean-robotics/mvp_mission
```

Install dependencies
```bash
rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y
```

Compile using `catkin_make`.

## Citation
```
@INPROCEEDINGS{9977346,
  author={Gezer, Emir Cem and Zhou, Mingxi and Zhao, Lin and McConnell, William},
  booktitle={OCEANS 2022, Hampton Roads},
  title={Working toward the development of a generic marine vehicle framework: ROS-MVP},
  year={2022},
  volume={},
  number={},
  pages={1-5},
  doi={10.1109/OCEANS47191.2022.9977346}}
```

## Funding
This work is supported by the [National Science Foundation](https://www.nsf.gov/) award [#2154901](https://www.nsf.gov/awardsearch/showAward?AWD_ID=2154901&HistoricalAwards=false)
