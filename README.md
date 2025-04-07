# :milky_way: CAD-Mesher: A Convenient, Accurate, Dense Mesh-based Mapping Module in SLAM for Dynamic Environments

The official implementation of CAD-Mesher (A Convenient, Accurate, Dense Mesh-based Mapping Module in SLAM for Dynamic Environments), an accurate meshing module for dynamic environments. CAD-Mesher can easily integrate with various LiDAR odometry to further improve their localization accuracy and build high-quity static mesh maps. This work is submitted for IEEE T-MM.

Welcome to our [website](https://yaepiii.github.io/CAD-Mesher/) for more details.

[![CAD-Mesher](https://res.cloudinary.com/marcomontalbano/image/upload/v1738145804/video_to_markdown/images/youtube--XmaxL6urYHg-c05b58ac6eb4c4700831b2b3070cd403.jpg)](https://www.youtube.com/watch?v=XmaxL6urYHg "CAD-Mesher")

If you think our work useful for your research, please cite:

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

## :mega: New

- Apr. 07. 2025: :tada: The paper is accepted by IEEE T-MM!
- Jan. 28. 2025: :smiley_cat: Codes released!

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

### :flashlight: Usage

<details>
<summary><b>Click here for an usage video!</b></summary>

[![Usage Introduction](https://res.cloudinary.com/marcomontalbano/image/upload/v1738145927/video_to_markdown/images/youtube--89fxz1mswRo-c05b58ac6eb4c4700831b2b3070cd403.jpg)](https://www.youtube.com/watch?v=89fxz1mswRo "Usage Introduction")

</details>

<details>
<summary><b>Click here for an usage introduction!</b></summary>

**1. Modify LiDAR Odometry Cpp**

Usually, aLiDAR odometry will publish the solved poses and the current frame point cloud (e.g., /aft_mapped_to_init and /velodyne_cloud_4 in _A-LOAM_). If not, you need to publish the Odometry topic in the ```nav_msgs::Odometry``` format. And publish point cloud topics in the ```sensor_msgs::PointCloud2``` format. Such as KISS-ICP:

```
odom_publisher_ = pnh_.advertise<nav_msgs::Odometry>("/kiss/odometry", queue_size_);             // odometry topic
pointcloud_publisher_ = pnh_.advertise<sensor_msgs::PointCloud2>("/kiss/pointcloud", queue_size_); // point cloud topic
```

**2. Modify LiDAR Odometry Launch File**

Then, you should to remap the odometry topic to ```/mapping_odom``` and the point cloud topic to ```/maaping_cloud``` in the launch file in the LiDAR odometry node ```<node> </node>``` TAB. such as KISS-ICP:
```
<!-- Odometry node -->
<node pkg="kiss_icp" type="odometry_node" name="odometry_node" output="screen">
<!-- ROS params -->
<remap from="pointcloud_topic" to="$(arg topic)"/>
<param name="odom_frame" value="$(arg odom_frame)"/>
<param name="base_frame" value="$(arg base_frame)"/>
<param name="publish_odom_tf" value="$(arg publish_odom_tf)"/>
<param name="visualize" value="$(arg visualize)"/>
<!-- KISS-ICP params -->
<param name="max_range" value="$(arg max_range)"/>
<param name="min_range" value="$(arg min_range)"/>
<param name="deskew" value="$(arg deskew)"/>
<param name="voxel_size" value="$(arg voxel_size)"/>
<param name="max_points_per_voxel" value="20"/>
<param name="initial_threshold" value="2.0"/>
<param name="min_motion_th" value="0.1" />
<!-- CAD-Mesher-->
<remap from="/kiss/odometry" to="/mapping_odom"/>
<remap from="/kiss/pointcloud" to="/mapping_cloud"/>
</node>
```

**3. Run**

Finally, you can run LiDAR odometry together with our CAD-Mesher meshing module!
```
# Teminator 1: run LiDAR odometry
roslaunch your_lidar_odometry your_lidar_odometry.launch
# Teminator 2: run CAD-Mesher
roslaunch cad_mesher xxx.launch
# Teminator 3: play bag file
rosbag play your_dataset.bag
```

</details>

### :books: Datasets

**KITTI**

For the KITTI dataset, you can select two configs:

- kitti_meshing: Focus more on map construction accuracy.
```
roslaunch cad_mesher kitti_meshing.launch
```

- kitti_odometry: Focus more on pose estimation accuracy.
```
roslaunch cad_mesher kitti_odometry.launch
```

**UrbanLoco**

For the UrbanLoco dataset, we just test it on HongKong sequneces and you can run this config:
```
roslaunch cad_mesher ulhk.launch
```

**GroundRobot**

For the GroundRobot dataset, you can run this config:
```
roslaunch cad_mesher groundrobot.launch
```

**Newer College**

For the Newer College dataset, we generate the bag file from an example part of the dataset (Quad) according to [SHINE-Mapping](https://github.com/PRBonn/SHINE_mapping) for the test, you can click here to download (634 MB):

you can run this config:
```
roslaunch cad_mesher newer_college.launch
```

**MaiCity**

For the MaiCity dataset, you can run the following command to start KISS-ICP and Our CAD-Mesher meshing module:
```
roslaunch cad_mesher maicity.launch
```


## :bar_chart: Evaluation

CAD-Mesher saves all its report to the path `result_path` given in each **launch** file. If you find ros warning: ` Can not open Report file`, create the folder of `result_path` first.

### Localization accuracy

The file `0x_pred.txt` is the KITTI format path. I use [KITTI odometry evaluation tool](https://github.com/LeoQLi/KITTI_odometry_evaluation_tool) for evaluation:

```
cd cad_mesher_ws/cad_mesher_result
git clone https://github.com/LeoQLi/KITTI_odometry_evaluation_tool
cd KITTI_odometry_evaluation_tool/
python evaluation.py --result_dir=.. --eva_seqs=0x_pred
```

The file `pose_evo.txt` is the TUM format path. I use [evo](https://github.com/MichaelGrupp/evo) for evaluation:
```
cd cad_mesher_ws/cad_mesher_result
pip install evo
evo_ape tum groundtruth.txt pose_evo.txt -a -p
```

### Meshing accuracy

To save the mesh map, set parameter `save_mesh_map` in yaml file to `true`. A ply file should be saved in `cad_mesher_ws/cad_mesher_result`.

I follow the modified `TanksAndTemples/evaluation` tool by [SLAMesh](https://github.com/lab-sun/SLAMesh) to evaluate the mesh. You can find it here: [TanksAndTemples/evaluation_rjy](https://github.com/RuanJY/TanksAndTemples)

Then compare the mesh with the ground-truth point cloud map:
```
cd TanksAndTemples_direct_rjy/python_toolbox/evaluation/
python run.py \
--dataset-dir ./data/ground_truth_point_cloud.ply \
--traj-path ./ \
--ply-path ./data/your_mesh.ply
```

Or, you can also evaluate following by [SHINE-Mpping](https://github.com/PRBonn/SHINE_mapping):

Please change the data path and evaluation set-up in ./eval/evaluator.py and then run:
```
python ./eval/evaluator.py
```

### Dynamic removal

We follow [the KTH dynamic benchmark](https://github.com/KTH-RPL/DynamicMap_Benchmark) for dynamic removal evaluation.

To save the .pcd map, set parameter `save_raw_point_clouds` in yaml file to `true`. A pcd file should be saved in `cad_mesher_ws/cad_mesher_result`.

Create the eval data:
```
./export_eval_pcd [folder that you have the output pcd] [method_name_output.pcd] [min_dis to view as the same point]
```

Print the score:
```
python3 scripts/py/eval/evaluate_all.py
```

## :rose: Acknowledgements

We thank the authors of the [SLAMesh](https://github.com/lab-sun/SLAMesh) open-source packages:

- Ruan, Jianyuan and Li, Bo and Wang, Yibo and Sun, Yuxiang, "SLAMesh: Real-time LiDAR Simultaneous Localization and Meshing," in 2023 IEEE International Conference on Robotics and Automation (ICRA), pp. 3546-3552, 2023.


# Website License
<a rel="license" href="http://creativecommons.org/licenses/by-sa/4.0/"><img alt="Creative Commons License" style="border-width:0" src="https://i.creativecommons.org/l/by-sa/4.0/88x31.png" /></a><br />This work is licensed under a <a rel="license" href="http://creativecommons.org/licenses/by-sa/4.0/">Creative Commons Attribution-ShareAlike 4.0 International License</a>.
