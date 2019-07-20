# rtab_map_launch_files

Above is the RTAB-Map launch files used to integrate the LIDAR and the Realsense Camera. Parameters can be tuned for better performance. The tutorial that we refered to was this http://wiki.ros.org/rtabmap_ros/Tutorials/SetupOnYourRobot#Switching_between_Mapping_and_Localization (mostly from section 2.1 that combines the Kinect + Odometry + 2D laser. In our case it is hence the LIDAR + Odometry + 2D laser.)
  
This launch file has to be launched after launching the LIDAR and the Realsense Camera. They will thus feed the information into the launch file for the RTAB-Map, allowing it to perform SLAM.  

The output of the RTAB-Map in different conditions are as shown below:  
### Dark with additional features
![alt text](https://github.com/frankienaik/rtab_map_launch_files/blob/master/capstone/dark_w_additional_feats(1).png)
![alt text](https://github.com/frankienaik/rtab_map_launch_files/blob/master/capstone/dark_w_additional_feats.png)    

### Lit with additional features
![alt text](https://github.com/frankienaik/rtab_map_launch_files/blob/master/capstone/lit_w_additional_feats.png)  
  
### Dark with minimal features
![alt text](https://github.com/frankienaik/rtab_map_launch_files/blob/master/capstone/dark_w_min_feats(1).png)
![alt text](https://github.com/frankienaik/rtab_map_launch_files/blob/master/capstone/dark_w_min_feats.png)  
  
### Lit with minimal features
![alt text](https://github.com/frankienaik/rtab_map_launch_files/blob/master/capstone/dark_w_additional_feats(1).png)

## How to Launch
1) Run the RpLIDAR and the Realsense Camera  
2) Run this launch file

## Potential further development
1) Create a python script that reads the video and rewrite the data when a crack is detected. As it is a RGB-D data, we can up the R value such that a red bounding box can be created.



