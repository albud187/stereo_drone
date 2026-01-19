# Stereo Drone

Stereo Drone is a real-time computer vision pipeline that performs object location (distance and direction) and size estimation. It fuses imagery from two cameras, as well as camera geometry (intrinsic and extrinsic) to generate point clouds. Each point in the point cloud is representative of a location in 3D space (relative to the drone) of an object detected by the camera. The points in the pointcloud are clustered and associated, to localize indiviudal objects and estimate their size.

The overall data flow is as follows:

1. Left Image + Right Image Camera Geometry (Intrinsic) --> Disparity 

2. Disparity + Camera Geometry (Extrinsic) --> Point Cloud

3. Point Cloud + Clustering --> object distance, direction and size estimation

Stereo Drone implements this pipeline for interactive real time testing in which users can control a drone to observe different objects in a simulated environment while dynamically updating its pointcloud and localization / size estimation of observed objects.

A short [techical report](https://github.com/albud187/stereo_drone/blob/main/assets/technical_report.pdf) describing the computer vision pipeline and the software is also included.

A video demo of this project is linked as follows: [video demo (youtube)](https://www.youtube.com/watch?v=hZ89gX3frcM)

## Key Technologies and Dependancies

ROS2 - Overall data processing / communications and drone control

OpenCV - Image processing and calculation of disparity

Numpy - Matrix operations (reference frame transformation and data transformation)

Matplotlib  - Point Cloud Visualization

Gazebo - Simulation Environment

[SJTU Drone Simulation](https://github.com/albud187/sjtu-drone) - Drone used for simulation

# File Structure

```
/stereo_drone/
├── src/
|   ├── cv_drone/
├── data/
├── worlds/
├── Dockerfile
├── _dockerbuild.sh
├── _run.sh
```

- `src/cv_drone`: Contains core application logic for point cloud generation and visualization as well as teleoperation of the drone. 

- `data/` and `worlds/`: Contains files describing drone start position and identity, as well as description of simulated world. Required for Testing

# Start Up Guide

Note: This project is meant to be run inside the provided docker image that is running on an ubuntu machine.

1. Clone the repo and change directory to it:
```
git clone https://github.com/albud187/stereo_drone.git
cd stereo_drone
```

2. Build the docker container. While in `stereo_drone/` execute:
```
bash _dockerbuild.sh
```

3. Run the docker container. While in `stereo_drone/` execute:
```
bash _run.sh
```

4. You should be in an interactive shell in the container, as root, and at the directory `/workdir/`. Build the ROS2 package by executing:
```
colcon build
```
This only has to be executed when you modify the ROS2 package.

5. Source key files by executing:
```
bash entrypoint.sh
```

6. You can now start the project by executing
```
bash _launch.sh
```

# Usage

This section assumes the docker image and ROS2 package is already built.

1. Launch the container by executing:
```
bash _run.sh
```

You may have to execute `bash entrypoint.sh` inside the container before being able to launch the application.

2. From inside the container launch the project by executing:
```
bash _launch.sh
```

You should see the following screens:

`Gazebo` - simulated world with a drone and various objects
![start](https://github.com/albud187/stereo_drone/blob/main/assets/drone_gazebo_sim.png)

`left_camera` - real time raw camera video feed

`left_depth_map` - real time depth map (white to gray pixels are object detections, with brighter meaning farhter away)

![start](https://github.com/albud187/stereo_drone/blob/main/assets/drone_left_cam.png)

`teleop_node` - click this terminal and use your keyboard to move the drone:
```
w/s = translation forwards/backwards - vx in m/s, positivie is translation forwards and negative is translation backwards
a/d = translation left/right - vy in m/s, positivie is translation left and negative is translation right
i/k = translation up/down - vz in m/s, positive translation is up, negative is translation down
j/l = rotation left/right - rz in rad/sec, positiive is rotatation (yaw) left, negative is rotation right
q = reset velocity to 0
```
![start](https://github.com/albud187/stereo_drone/blob/main/assets/teleop_node.png)

`Point Cloud Plot - Relative to Drone` - real time plot of detections. Detections have 3D info, but are shown in 2D top-down view:
    - red dots = individual detections. Each pixel on `left_depth_map` corresponds to a position in 3D space relative to the drone. These positions are the red dots
    - blue dot = the drone you are controlling
    - gray dots = cluster center. Red dots are clustered and the location of the center of these clusters correspond to the approximate location of detected objects

![start](https://github.com/albud187/stereo_drone/blob/main/assets/point_cloud_plot.png)