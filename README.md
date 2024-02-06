<div align="center">

<h1>Multi - LiCa</h1>

Multi - LiDAR-to-LiDAR calibration framework for ROS 2 and non-ROS applications  
  
[![Linux](https://img.shields.io/badge/os-linux-blue.svg)](https://www.linux.org/)
[![Docker](https://badgen.net/badge/icon/docker?icon=docker&label)](https://www.docker.com/)
[![ROS2humble](https://img.shields.io/badge/ros2-humble-blue.svg)](https://docs.ros.org/en/humble/index.html)

<img src="doc/img/calibration_results.png" width="1000px">

</div>

<h2>Introduction</h2>
This project provides an extrinsic calibration framework for multiple LiDAR sensors. It employs the Generalized Iterative Closest Point (GICP) algorithm for LiDAR-to-LiDAR extrinsic calibration and uses the RANdom SAmple Consensus (RANSAC) method to calibrate the pitch and z-distance to the ground of a single LiDAR, assuming other coordinates are known.

<h2>Overview</h2>
Presented is a schematic representation of the calibration framework.  

<img src="doc/img/Multi-LiCa_pipeline.png" width="400px">  

*Motion- and targetless multi - LiDAR-to-LiDAR Calibration Pipeline,  
 developed at the Institute of Automotive Technology, TUM*

<h2>Limitations</h2>

- Our tool was specifically developed for motionless calibration.
- We assume that each LiDAR to be calibrated has either a directly overlapping FOV with the target LiDAR FOV or has overlap with other LiDAR(s) with overlap to the target. This can be cascading dependency to the target.
- We assume that the ground is flat and the environment is static.
- Input point clouds for the calibration are in sensor_msgs/PointCloud2 or in .pcd format.

<h2>Prerequisites</h2>
The bare minimum requirement for our tool is a Linux-based OS and Docker, as we provide a Docker image with our framework.  
You do not need to build anything locally, but you are free to do so as described in the following section.  
For the local build, you will need ROS 2 - humble, Python 3.10 with opend3d, scipy, ros2_numpy and pandas (optional).

<h2>Installation</h2>

<details>
<summary><h3>Docker</h3></summary>
1. Build the Docker image:

    ```
    ./docker/build_docker.sh
    ```

2. Run the container:

    ```
    ./docker/run_docker.sh
    ```
</details>

<details>
<summary> <h3> Local </h3> </summary>
1. Install ROS2 humble (might work with other ROS2 distributions but wasn't tested):
https://docs.ros.org/en/humble/Installation.html 

2. Create a ROS 2 workspace:
    ```
    mkdir -p ~/ros2_ws
    cd ~/ros2_ws
    ```

3. Clone the repository:
    ```
    git clone git@github.com:TUMFTM/Multi_LiCa.git
    ```

4. Install dependencies:
    ```
    cd Multi_LiCa
    pip install --no-cache-dir --upgrade pip
    pip install --no-cache-dir -r requirements.txt
    ```

5. Source the ROS 2 environment and build the project using `colcon`:

    ```
    source /opt/ros/$ROS_DISTRO/setup.bash
    colcon build --symlink-install --packages-up-to multi_lidar_calibrator --cmake-args -DCMAKE_BUILD_TYPE=Release
    ```

6. Configure the parameters to fit your data:
    ```
    vim config/params.yaml
    ```

7. Launch the multi_lidar_calibrator node:

    ```
    ros2 launch multi_lidar_calibrator calibration.launch.py
    ```
</details>

<h2>LiDAR-to-Ground/Base Calibration</h2>
In addition to LiDAR-to-LiDAR calibration, you can perform target LiDAR-to-ground/base calibration if your x,y translation and roll, yaw rotation are precisely known.


<h2>Code coming soon!</h2>

<h2>Citation</h2>
If you use this framework for any academic work, please cite our original paper.