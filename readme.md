# 3D Detection & Tracking Viewer
This project was developed for view 3D object detection and tracking results.
It supports rendering 3D bounding boxes as car models and rendering boxes on images.
Currently, it is still updating. 
## Design pattern
This code includes two parts, one for data loading, other one for visualization of 3D detection and tracking results.
The overall framework of design is as shown below:
![](./doc/125.gif)
## Prepare data 
Kitti detection dataset
```
# For Kitti Detection Dataset         
└── kitti_detection
       ├── ImageSets
       |      ├──test.txt
       |      ├──train.txt
       |      └──val.txt
       ├── testing 
       |      ├──calib
       |      ├──image_2
       |      ├──pred_label_2
       |      └──velodyne      
       └── training
              ├──calib
              ├──image_2
              ├──label_2
              └──velodyne 
```
Kitti tracking dataset
```
# For Kitti Tracking Dataset         
└── kitti_tracking
       ├── ImageSets
       |      ├──test.txt
       |      ├──train.txt
       |      └──val.txt
       ├── testing 
       |      ├──calib
       |      |    ├──0000.txt
       |      |    ├──....txt
       |      |    └──0028.txt
       |      ├──image_02
       |      |    ├──0000
       |      |    ├──....
       |      |    └──0028
       |      ├──pose
       |      |    ├──0000
       |      |    |    └──pose.txt
       |      |    ├──....
       |      |    └──0028
       |      |         └──pose.txt
       |      ├──pred_label_02
       |      |    ├──0000.txt
       |      |    ├──....txt
       |      |    └──0028.txt
       |      └──velodyne
       |           ├──0000
       |           ├──....
       |           └──0028      
       └── training # the structure is same as testing set
              ├──calib
              ├──image_02
              ├──pose
              ├──label_02
              └──velodyne 
```
## Requirements
```
python3
numpy
vedo
vtk
opencv
matplotlab
```
