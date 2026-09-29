# FAST-LIVO2-RTK

FAST-LIVO2-RTK extends FAST-LIVO2 with RTK/GNSS-constrained global optimization for long-term LiDAR-visual mapping.

**Key highlights include:**

1. Fully Reproducible: Complete open-source software and hardware setup guaranteeing fully reproducible LIVO-RTK experiments.
2. Robust Initialization Module: Built-in initialization featuring cross-correlation-based time offset estimation and hand-eye extrinsic calibration.
3. LIV-RTK Fusion Paradigm: A simple example paradigm for fusing LIVO trajectories with RTK observations.

📬 For further assistance or inquiries, please feel free to contact Chunran Zheng at [zhengcr@connect.hku.hk](mailto:zhengcr@connect.hku.hk).

<div align="center">
  <img src="pics/compare.jpg" width="100%" alt="FAST-LIVO2 and FAST-LIVO2-RTK comparison in a degraded LiDAR-visual scene">
  <br>
  <sub>Collected in a challenging scene with degraded geometry and texture.
<b>a2, b2, c2</b>: FAST-LIVO2 baseline. <b>a1, b1, c1</b>: results after RTK fusion.</sub>
</div>

## 1. Prerequisited

### 1.1 Ubuntu and ROS

Ubuntu 18.04~20.04. See [ROS Installation](http://wiki.ros.org/ROS/Installation).

### 1.2 PCL, Eigen, and OpenCV

PCL>=1.8, Eigen>=3.3.4, OpenCV>=4.2.

### 1.3 Sophus

Install the non-templated/double-only version of Sophus.

```bash
git clone https://github.com/strasdat/Sophus.git
cd Sophus
git checkout a621ff
mkdir build && cd build && cmake ..
make
sudo make install
```

### 1.4 Vikit

Vikit provides the camera models and math utilities required by this project. Put it in your catkin workspace source folder.

```bash
cd ~/catkin_ws/src
git clone https://github.com/xuankuzcr/rpg_vikit.git
```

### 1.5 RTK Dependencies

The RTK branch depends on the GNSS ROS message package used by the u-blox/GVINS toolchain. Put it in the same catkin workspace:

```bash
cd ~/catkin_ws/src
git clone https://github.com/HKUST-Aerial-Robotics/gnss_comm.git
```

GTSAM is used for factor graph-based post-processing optimization.

```bash
git clone https://github.com/borglab/gtsam.git
cd gtsam
mkdir build && cd build
cmake -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON ..
make -j$(nproc)
sudo make install
```

GeographicLib is used for converting geographic coordinates to local Cartesian coordinates.

```bash
sudo apt-get install libgeographic-dev ros-${ROS_DISTRO}-eigen-conversions
```

## 2. Run our examples

Download the provided RTK test rosbag file: [RTK-Dataset](https://drive.google.com/drive/folders/1xun-YL78cbMtyV60uN3FQFsHkkLUWvtJ?usp=sharing).

1. Launch the system and load the configuration file:

```bash
roslaunch fast_livo HH.launch
```

2. Play the rosbag. Once the sequence is finished, press `Enter` in the terminal running the launch file to trigger the backend optimizer.

```bash
rosbag play HH-LVGO-01.bag
```
### SubT_MRS_Hawkins_Long_Corridor_RC

Build the workspace after adding this configuration (ROS Noetic):

```bash
cd /home/ubt/workspace/noetic_ws
source /opt/ros/noetic/setup.bash
catkin_make -j4
source devel/setup.bash
roslaunch fast_livo SubT_MRS_Hawkins_Long_Corridor_RC.launch play_bag:=true
```

The launch file defaults to
`/datasets/super_odom/SubT_MRS_Hawkins_Long_Corridor_RC/Long_Corridor_Rosbag/2026-09-28-11-01-46.bag`.
Use `rviz:=false` for headless execution, `rate:=0.5` for slower playback,
or `bag:=/path/to/another.bag` to override the bag. To play manually, leave
`play_bag:=false` (the default), then run in another sourced terminal:

```bash
rosbag play --clock /datasets/super_odom/SubT_MRS_Hawkins_Long_Corridor_RC/Long_Corridor_Rosbag/2026-09-28-11-01-46.bag --topics /velodyne_points /imu/data /camera_1/image_raw
```

- Inputs: `/velodyne_points` (16 rings, approximately 10 Hz), `/imu/data`
  (approximately 200 Hz), and `/camera_1/image_raw` (640 x 480 BGR).
  Velodyne's FLOAT32 `time` field is in seconds; this configuration uses
  `preprocess/velodyne_time_scale: 1000.0` to convert it to internal milliseconds.
  Other configurations keep the existing default of `0.001`.
- The supplied `*_Intrinsics.yaml` and `*_Extrinsics.yaml` are copied into
  `config/`. The raw camera is **MEI**, so `rectify_mei.py` applies its mirror,
  radial and tangential distortion parameters and publishes `/camera_1/image_rect`.
  The mapper and rectifier share `camera_SubT_MRS_Hawkins_Long_Corridor_RC.yaml`,
  defining a zero-distortion virtual pinhole camera with focal lengths of 320 px.
  This crops the field of view to avoid invalid borders. The node uses NumPy,
  OpenCV and cv_bridge (`python3-numpy`, `python3-opencv`, `ros-noetic-cv-bridge`);
  OpenCV's optional omnidir module is not required at runtime.
- Extrinsics use `p_imu = T_imu_lidar * p_lidar` from `laser_to_imu`, and
  `T_camera_lidar = inverse(T_imu_camera) * T_imu_lidar` from the supplied
  `rgb_camera_to_imu`. The source calibration values are retained; rectification
  does not rotate the camera axes.
- Original sensor header timestamps are preserved. They differ from the 2026
  bag recording timestamps; no recording-to-sensor time offset is applied.
  IMU/image/LiDAR offsets default to zero because no temporal calibration was supplied.
- There is no GNSS in this bag. `gps/gps_en: false` disables the GNSS subscription
  and offline RTK backend; no Enter-key optimization step is needed. The TUM
  trajectory is written to `Log/result/SubT_MRS_Hawkins_Long_Corridor_RC.txt`.
  Debug files use `output/SubT_MRS_Hawkins_Long_Corridor_RC/debug/` by default
  (override with `outputfilepath:=...`). Replaying overwrites the sequence trajectory.

These are initial mapping/noise settings; the supplied files specify geometric
calibration, not tuned noise parameters or trajectory accuracy guarantees.

## 3. Appendix
**Time Synchronization:**

The diagram illustrates the time synchronization scheme among GNSS, LiDAR, and image data.

<div align="center">
  <img src="pics/sync2.jpg" width="100%" alt="Time synchronization diagram">
</div>

**Hardware Platform:**

The table below summarizes the main devices used by the platform.

<div align="center">
  <table align="center">
    <tr>
      <th>Device</th>
      <th>Image</th>
      <th>Model Description</th>
    </tr>
    <tr>
      <td align="center">LiDAR</td>
      <td align="center"><img src="pics/LiDAR.jpg" width="180" alt="Livox Mid-360 LiDAR"></td>
      <td align="center">Model: Livox Mid-360</td>
    </tr>
    <tr>
      <td align="center">Camera</td>
      <td align="center"><img src="pics/camera.jpg" width="180" alt="MV-CB016-10GC-S-W camera"></td>
      <td align="center">Model: MV-CB016-10GC-S-W</td>
    </tr>
    <tr>
      <td align="center">GNSS Receiver</td>
      <td align="center"><img src="pics/ublox.jpg" width="180" alt="u-blox ZED-F9P GNSS receiver"></td>
      <td align="center">Model: u-blox ZED-F9P</td>
    </tr>
    <tr>
      <td align="center">Computing Unit</td>
      <td align="center"><img src="pics/n100.jpg" width="180" alt="N100 mini PC"></td>
      <td align="center">Model: N100 mini PC</td>
    </tr>
    <tr>
      <td align="center">Synchronization Controller</td>
      <td align="center"><img src="pics/stm32.jpg" width="180" alt="STM32 synchronization controller"></td>
      <td align="center">Model: STM32</td>
    </tr>
  </table>
</div>

## 4. Acknowledgements

This repository is built on top of [FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2) and uses several open-source libraries and packages, including [GTSAM](https://github.com/borglab/gtsam), [GeographicLib](https://geographiclib.sourceforge.io/), and [gnss_comm](https://github.com/HKUST-Aerial-Robotics/gnss_comm).
