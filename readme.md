# Final Year Project Assorted Code!

## What is here?
This is a collection of Matlab code that I put together over the course of my FYP. It includes some the examples, and results obtained during my work.


- **/BeamModelFigures** This is just a script to generate a figures for a LiDAR beam model. 
- **/Graph Optimiser** This is a script to perform pose graph optimisation on a 2D pose graph. It is an implementation from a paper (reference in the code and report).
- **/ICP** Very simple implementation of ICP using SVD to find the rotation matrix. 
- **/IMU Calibration** This script was used to find the rotation matrix mapping the LiDAR frame into the IMU frame from data collected on the IMU and an ICP algorithm.
- **/LIDAR Camera Calibration test** This holds the code and results for my attempt at calculating the extrinsic transform between LiDAR and Camera sensors. It processes point clouds and images to extract checkerboard corners, and attempts to find an optimal rotation and tranlation to map one data set into another.
- **/SLAM** This folder contains 2D implementations of grid mapping, particle filter localisation, and finally SLAM. This was the first examples I implemented during the project. 


Have fun...