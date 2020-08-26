# Depthmap to Point Cloud 

This project is to interact with 3d camera which is supported by OPENNI library and Kinect which is supported by FREENECT library to extract RGB point cloud (PCD) buffer/file from the depth map. 
Further computer vision algorithms can be applied in PCD files.

## How to compile

* mkdir build
* cd build
* cmake ..
* make

## How to execute

* cd ../bin (Assumed current directoty is build)
* ./depthmap_to_pcd
