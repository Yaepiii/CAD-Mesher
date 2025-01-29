# :milky_way: CAD-Mesher: A Convenient, Accurate, Dense Mesh-based Mapping Module in SLAM for Dynamic Environments

The official implementation of CAD-Mesher (A Convenient, Accurate, Dense Mesh-based Mapping Module in SLAM for Dynamic Environments), an accurate meshing module for dynamic environments. CAD-Mesher can easily integrate with various LiDAR odometry to further improve their localization accuracy and build high-quity static mesh maps. This work is submitted for IEEE T-MM.

Welcome to our [website](https://yaepiii.github.io/CAD-Mesher/) for more details.

![Video](./web/resources/CAD-Mesher.mp4)

If you think our work useful for your research, please cite:

```
@misc{jia2024trloefficientlidarodometry,
      title={CAD-Mesher: A Convenient, Accurate, Dense Mesh-based Mapping Module in SLAM for Dynamic Environments}, 
      author={Yanpeng Jia, Fengkui Cao, Ting Wang, Yandong Tang. Shiliang Shao, and Lianqing Liu},
      year={2024},
      eprint={2408.05981},
      archivePrefix={arXiv},
      primaryClass={cs.RO},
      url={https://arxiv.org/abs/2408.05981}, 
}
```

## :mega: New

- Jan. 28. 2025: :smiley_cat: Commit the codes!

## :gear: Installation

### :bookmark_tabs: Dependence

We tested our code in *Ubuntu18.04* with *ROS melodic* and *Ubuntu20.04* with *ROS neotic*.

**1.ROS**

Install ros following [ROS Installation](http://wiki.ros.org/noetic/Installation/Ubuntu). We use the PCL and Eigen library in ROS.

**2.Ceres**

We tested ceres-solver version: 1.14.0 (Error observed with V-2.2).
```
sudo gedit /etc/apt/sources.list
```
Paste the following code at the top of source.list and save it:
```
deb http://cz.archive.ubuntu.com/ubuntu trusty main universe
```
Update:
```
sudo apt-get update
```
Install dependency library:
```
sudo apt-get install liblapack-dev libsuitesparse-dev libcxsparse3.1.2 libgflags-dev 
sudo apt-get install libgoogle-glog-dev libgtest-dev
```
Downloading ceres-solver-1.14.0
```
wget ceres-solver.org/ceres-solver-1.14.0.tar.gz
tar -zxvf ceres-solver-1.14.0.tar.gz
```
Build:
```
cd ceres-solver-1.14.0
mkdir build
cd build
cmake ..
make -j4
sudo make install
```

**3.mesh_tools**

We use mesh_tools to visualize the mesh map with the `mesh_msgs::MeshGeometryStamped` ROS message. mesh_tools also incorporates navigation functions upon mesh map. [Mesh tool introduction](https://github.com/naturerobots/mesh_tools)

Install mesh_tools by:

1. Install [lvr2](https://github.com/uos/lvr2):
```
sudo apt-get install build-essential \
     cmake cmake-curses-gui libflann-dev \
     libgsl-dev libeigen3-dev libopenmpi-dev \
     openmpi-bin opencl-c-headers ocl-icd-opencl-dev \
     libboost-all-dev \
     freeglut3-dev libhdf5-dev qtbase5-dev \
     qt5-default libqt5opengl5-dev liblz4-dev \
     libopencv-dev libyaml-cpp-dev
```
In Ubuntu18.04, use `libvtk6` because `libvtk7` will conflict with `pcl-ros` in melodic.
```
sudo apt-get install  libvtk6-dev libvtk6-qt-dev
```
In Ubuntu 20.04,
```
sudo apt-get install  libvtk7-dev libvtk7-qt-dev
```

then:
```
cd a_non_ros_dir
```
build:
```
git clone https://github.com/uos/lvr2.git
cd lvr2 
mkdir build && cd build
cmake .. && make
sudo make install
```
It may take you some time.

2. Install mesh_tools, (I can not install it from official ROS repos now, so I build it from source)

```
mkdir -p ./slamesh_ws/src
cd slamesh_ws/src
git clone https://github.com/naturerobots/mesh_tools.git
cd ..
rosdep update
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
```

### :pencil: CAD-Mesher

Clone this repository and build:
```
cd cad_mesher_ws/src
git clone https://github.com/Yanpiii/CAD-Mesher.git
cd .. && catkin_make
mkdir cad_mesher_result
source ~/cad_mesher_ws/src/devel/setup.bash
```

## :video_game: How to easily use

<details>
<summary><b>Click here for an usage introduction video!</b></summary>

[![CAD-Mesher](./web/resources/intro.mp4)

</details>








This is the repository that contains source code for the [CAD-Mesher website](https://yaepiii.github.io/CAD-Mesher/).

The code is being organized...

When the article is accepted, the code will be published.

If you are interested in our work or use our method in your work, please consider citing the following:
```
@misc{jia2024cadmesherconvenientaccuratedense,
      title={CAD-Mesher: A Convenient, Accurate, Dense Mesh-based Mapping Module in SLAM for Dynamic Environments}, 
      author={Yanpeng Jia and Fengkui Cao and Ting Wang and Yandong Tang and Shiliang Shao and Lianqing Liu},
      year={2024},
      eprint={2408.05981},
      archivePrefix={arXiv},
      primaryClass={cs.RO},
      url={https://arxiv.org/abs/2408.05981}, 
}
```

# Website License
<a rel="license" href="http://creativecommons.org/licenses/by-sa/4.0/"><img alt="Creative Commons License" style="border-width:0" src="https://i.creativecommons.org/l/by-sa/4.0/88x31.png" /></a><br />This work is licensed under a <a rel="license" href="http://creativecommons.org/licenses/by-sa/4.0/">Creative Commons Attribution-ShareAlike 4.0 International License</a>.
