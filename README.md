# Localization Fusion

Graduate SLAM course project
Inproving localization accuracy of a welding robot in a ship hull block or around it.

This submodule consists of code to extract useful readings from our rosbag recorded in Gazebo, to apply Kalman filter to fuse different sensor readings, as well as to evaluate RMSE of the outputs and generate animation.

## Preprocessing
*rosbag2mat.m*

This script extracts readings from rosbag and saves to a .mat file for processing in subsequent stage. Topics includes ground truth, PointCloud of the map, PointCloud generated by velodyne LidAR, localization results from HDL algorithm, distance measurements to local features from RealSense depth camera. 

## Camera data simulation and sensor fusion with Kalman filter
*Kalman2D_simulated_camera.m*

This script simulates camera readings by adding Gaussian noise on top of the ground truth data and downsample. It tests out the system that we proposed to supplement LiDAR odometry with local landmark measurements. XY localization results after sensor fusion, along with baseline HDL result (based on LiDAR only) and ground truth locations, are saved in a .mat file for evaluation in the next stage.

## Evaluating result
*evalResults.m*

This script computes the root mean square error (RMSE) of localization results from 3 methods: incremental ICP, baseline HDL, and camera-aided HDL. The first two methods only uses LiDAR odometry. Results from incremental SLAM ICP are computed in ICP_SLAM submodule. 


## Bonus
*animateGen.m*

This script takes in the localization results from 3 methods (incremental ICP, baseline HDL, and camera-aided HDL), plots them against the ground truth, and generates an animation file.


