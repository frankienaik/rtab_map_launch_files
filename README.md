# RTAB-Map Launch Files

Above is the RTAB-Map launch files used to integrate the RPLidar and the Realsense Camera D435i. Parameters can be tuned for better performance. The tutorial that we refered to was this http://wiki.ros.org/rtabmap_ros/Tutorials/SetupOnYourRobot#Switching_between_Mapping_and_Localization (mostly from section 2.1 that combines the Kinect + Odometry + 2D laser. In our case it is hence the LIDAR + Odometry + 2D laser.)

P.S. : The topics (of the LIDAR and RGBD Camera) SHOULD be changed if LIDAR model and RGBD Camera model is different. Instructions below only took into account my hardwares.

## RTAB-Map slight explanation
![alt text](https://github.com/frankienaik/rtab_map_launch_files/blob/master/capstone/2d_3d.PNG)
This image is taken from https://introlab.3it.usherbrooke.ca/mediawiki-introlab/images/7/7a/Labbe18JFR_preprint.pdf. It shows the block diagram of the ROS RTAB-Map node.



## Setting up and launching

#### Running RTAB-Map:
1) The shadow_camera.launch file has to be placed inside $(realsense2_camera)/launch directory.
2) The demo_frankie_mapping.rviz file has to be placed inside $(find rtabmap_ros)/launch/config directory.
3) ```roslaunch start.launch``` to run the RPLIDAR and the Realsense Camera. 
4) If there are port problems, run: ```sudo chmod 666 /dev/ttyRplidar``` the /dev/tty* should be based on your error code.  
5) Choose the odometry you wish to use. The two odometry available is RGBD Odometry and Iterative Closest Point (ICP) Odometry. If RGBD Odometry is to be used, roslaunch icp_odometry_RTAB_Map.launch. Else if ICP Odometry is to be used, ```roslaunch rgbd_odometry_RTAB_Map.launch```. (Both launch files are very different, the icp_odometry_RTAB_Map.launch file is very short and simple, however, it can only run using simulation time or run the RTAB-Map using ICP Odometry. For ```rgbd_odometry_RTAB_Map.launch```, the file is longer as it takes into account more variables, providing it with greater capabilities. Both ways work and it just depends on your personal coding style. I personally prefer the simpler and shorter one as it is more readable. However, it is possible to include both rgbd_odometry and icp_odometry in one single launch file. You may add command line options or arguments such as ```rviz:=true``` or ```rtabmapviz:=true``` if you wish to visualize the mapping.
6) If you want to record the mapping process...
```roslaunch data_recorder.launch``` (Do note that you can change the output_path of the db file and it only works with icp_odometry_RTAB_Map.launch as this is additional and not required in the project)

#### Running the Replay File:
1) ```roslaunch icp_odometry_RTAB_Map.launch use_sim_time:=true```
2) ```rosrun image_view image_view image:=/camera/color/image_raw``` This is to view the camera video
3) ```rosrun rtabmap_ros data_player _database:=~/.ros/output.db _frame_id:=base_footprint odom:=/odom scan:=/scan rgb/image:=camera/color/image_raw depth_registered/image:=/camera/aligned_depth_to_color/image_raw /rgb/camera_info:=/camera/color/camera_info --clock``` This is to view the mapping process

## Results
The output of the RTAB-Map in different conditions are as shown below:  

### Testing with manually controlled UGV

#### Dark with additional features with ICP Odometry
![alt text](https://github.com/frankienaik/rtab_map_launch_files/blob/master/capstone/dark_w_additional_feats(1).png)
![alt text](https://github.com/frankienaik/rtab_map_launch_files/blob/master/capstone/dark_w_additional_feats.png)    

#### Lit with additional features with ICP Odometry
![alt text](https://github.com/frankienaik/rtab_map_launch_files/blob/master/capstone/lit_w_additional_feats.png)  
  
#### Dark with minimal features with ICP Odometry
![alt text](https://github.com/frankienaik/rtab_map_launch_files/blob/master/capstone/dark_w_min_feats(1).png)
![alt text](https://github.com/frankienaik/rtab_map_launch_files/blob/master/capstone/dark_w_min_feats.png)  
  
#### Lit with minimal features with ICP Odometry
![alt text](https://github.com/frankienaik/rtab_map_launch_files/blob/master/capstone/dark_w_additional_feats(1).png)

### Autonomous Flight with Drone

#### with ICP Odometry
![alt text](https://github.com/frankienaik/rtab_map_launch_files/blob/master/capstone/lit_w_additional_feats.png)  

#### with RGBD Odometry
![alt text](https://github.com/frankienaik/rtab_map_launch_files/blob/master/capstone/dark_w_min_feats.png)  

## ICP Odometry vs RGBD Odometry

### ICP Odometry:
- Capable of 3DOF(x, y, yaw), resulting in a mapping only in the x-y plane. This is so as our ICP Odometry is generated with the use of a 2D LIDAR, which does not include a z-axis. With a 3D LIDAR, it is possible to map in the x-y-z plane.
- Not feature dependent.

### RGBD Odometry:
- Capable of 6DOF (x, y, z, roll, pitch, yaw), resulting in a mapping in the x-y-z plane.
- Very feature dependent.

#### ICP Odometry is more reliable and robust as compared to RGBD Odometry. It does not lose its odometry easily and is potentially able to map the x-y-z plane with a 3D LIDAR depending on the parameters. This is shown in the image below depicting the local occupancy grid creation. Grid/3D is automatically set to false if it is a 2D LIDAR.
![alt text](https://github.com/frankienaik/rtab_map_launch_files/blob/master/capstone/2d_3d.PNG)
This image is taken from https://introlab.3it.usherbrooke.ca/mediawiki-introlab/images/7/7a/Labbe18JFR_preprint.pdf. It depicst the local occupancy grid creation. Depending on the parameters and availability of the optional laser scan and point cloud inputs, the local occupancy grid can either be 2D or 3D.

#### Without a 2D LIDAR, I would still recommend ICP Odometry to conduct the mapping. However, the RTAB-Map have to be restarted at every level of inspection where altitude changes. With this, we can manually input the height or altitude of which the drone is at, allowing it to map at that z height. The different session maps would then be stiched together as shown in the images below.
![alt text](https://github.com/frankienaik/rtab_map_launch_files/blob/master/capstone/multi_session_mapping_1.PNG)
![alt text](https://github.com/frankienaik/rtab_map_launch_files/blob/master/capstone/multi_session_mapping_2.PNG)
The images are taken from https://introlab.3it.usherbrooke.ca/mediawiki-introlab/images/7/7a/Labbe18JFR_preprint.pdf. It depicts how the map would be joined together for 5 different sessions.

## Important Variables in RTAB-Map
- rgb/image
- depth/image
- rgb/camera_info
- rgbd_image
- scan
- odom
- odom_info
- frame_id

## Important Nodes in RTAB-Map Node
1) rgbd_sync
```
<node pkg="nodelet" type="nodelet" name="rgbd_sync" args="standalone rtabmap_ros/rgbd_sync" output="screen">

      <!-- change this to fit the output from your Realsense camera -->
      <remap from="rgb/image"       to="/camera/color/image_raw"/> <!-- RGB/Mono image. Should be rectified when subscribe_depth is true. Not required if parameter subscribe_stereo is true (use left/image_rect instead). -->
      <remap from="depth/image"     to="/camera/aligned_depth_to_color/image_raw"/> <!-- Registered depth image. -->
      <remap from="rgb/camera_info" to="/camera/color/camera_info"/> <!-- RGB camera metadata. Not required if parameter subscribe_stereo is true (use left/camera_info instead). -->
      <remap from="rgbd_image"      to="rgbd_image"/> <!-- RGB-D synchronized image, only when subscribe_rgbd is true. -->
    <!-- Should be true for not synchronized camera topics 
           (e.g., false for kinectv2, zed, realsense, true for xtion, kinect360)-->
      <param name="approx_sync"       value="true"/>  <!-- Use approximate time synchronization of input messages. If false, note that the odometry input must have also exactly the same timestamps than the input images. -->
</node>
```

2) icp_odometry
![alt text](https://github.com/frankienaik/rtab_map_launch_files/blob/master/capstone/icp_odometry.PNG)
This image is taken from https://introlab.3it.usherbrooke.ca/mediawiki-introlab/images/7/7a/Labbe18JFR_preprint.pdf. It depicst the block diagram for icp_odometry. Two odometry approaches can be used: Scan-To-Scan or Scan-To-Map.
```
<node pkg="rtabmap_ros" type="icp_odometry" name="icp_odometry" output="screen" >

     <remap from="scan"      to="/scan"/>

     <remap from="odom"      to="/odom"/> 
     <remap from="odom_info"      to="/rtabmap/odom_info"/> <!-- Odometry info. Required if parameter subscribe_odom_info is true. -->
	  
     <param name="frame_id"        type="string" value="base_footprint"/>   <!-- The frame attached to the mobile base. -->
     
     <remap from="rgb/image"       to="/camera/color/image_raw"/> <!-- RGB/Mono image. Should be rectified when subscribe_depth is true. Not required if parameter subscribe_stereo is true (use left/image_rect instead). -->
     <remap from="depth/image"     to="/camera/aligned_depth_to_color/image_raw"/> <!-- Registered depth image. -->
     <remap from="rgb/camera_info" to="/camera/color/camera_info"/> <!-- RGB camera metadata. Not required if parameter subscribe_stereo is true (use left/camera_info instead). -->

     <!-- ICP parameters -->
     <param name="Icp/PointToPlane"  type="string" value="false"/> <!-- "Use point to plane ICP." -->
     <param name="Icp/VoxelSize"     type="string" value="0.05"/> <!-- "Uniform sampling voxel size (0=disabled)." -->
     <param name="Icp/Epsilon"       type="string" value="0.001"/> <!-- "Set the transformation epsilon (maximum allowable difference between two consecutive transformations) in order for an optimization to be considered as having converged to the final solution." -->
     <param name="Icp/PointToPlaneK"  type="string" value="5"/> <!-- "Number of neighbors to compute normals for point to plane if the cloud doesn't have already normals." -->
     <param name="Icp/PointToPlaneRadius"  type="string" value="0.3"/> <!-- "Search radius to compute normals for point to plane if the cloud doesn't have already normals." -->
     <param name="Icp/MaxCorrespondenceDistance" type="string" value="0.1"/> <!-- "Max distance for point correspondences." -->
     <param name="Icp/PM"             type="string" value="true"/> <!-- use libpointmatcher to handle PointToPlane with 2d scans-->
     <param name="Icp/PMOutlierRatio" type="string" value="0.95"/> <!-- "TrimmedDistOutlierFilter/ratio: For convenience when configuration file is not set. For kinect-like point cloud, use 0.65." -->
	  
     <!-- Odometry parameters -->
     <param name="Odom/Strategy"        type="string" value="0"/> <!-- "0=Frame-to-Map (F2M) 1=Frame-to-Frame (F2F) 2=Fovis 3=viso2 4=DVO-SLAM 5=ORB_SLAM2 6=OKVIS 7=LOAM 8=MSCKF_VIO" -->
     <param name="Odom/GuessMotion"     type="string" value="true"/> <!-- "Guess next transformation from the last motion computed." -->
     <param name="Odom/ResetCountdown"  type="string" value="0"/> <!-- "Automatically reset odometry after X consecutive images on which odometry cannot be computed (value=0 disables auto-reset)." -->
     <param name="Odom/ScanKeyFrameThr"  type="string" value="0.9"/> <!-- [Geometry] Create a new keyframe when the number of ICP inliers drops under this ratio of points in last frame's scan. Setting the value to 0 means that a keyframe is created for each processed frame." -->
     
     <param name="Reg/Force3DoF"      type="string" value="false"/> 
     <param name="Optimizer/Strategy" type="string" value="2"/>
</node>
```
Parameters in here can be changed. Important variables are these:
```
<remap from="scan"      to="/scan"/>
<remap from="odom"      to="/odom"/> 
<remap from="odom_info"      to="/rtabmap/odom_info"/> <!-- Odometry info. Required if parameter subscribe_odom_info is true. -->
<param name="frame_id"        type="string" value="base_footprint"/>   <!-- The frame attached to the mobile base. -->
<remap from="rgb/image"       to="/camera/color/image_raw"/> <!-- RGB/Mono image. Should be rectified when subscribe_depth is true. Not required if parameter subscribe_stereo is true (use left/image_rect instead). -->
<remap from="depth/image"     to="/camera/aligned_depth_to_color/image_raw"/> <!-- Registered depth image. -->
<remap from="rgb/camera_info" to="/camera/color/camera_info"/> <!-- RGB camera metadata. Not required if parameter subscribe_stereo is true (use left/camera_info instead). -->
```

3) rgbd_odometry
![alt text](https://github.com/frankienaik/rtab_map_launch_files/blob/master/capstone/rgbd_odometry.PNG)
This image is taken from https://introlab.3it.usherbrooke.ca/mediawiki-introlab/images/7/7a/Labbe18JFR_preprint.pdf. It depicst the block diagram for rgbd_odometry and stereo_odometry. Two odometry approaches can be used: Frame-To-Frame or Frame-To-Map.
```
<node pkg="rtabmap_ros" type="rgbd_odometry" name="rgbd_odometry" output="$(arg output)" args="$(arg rtabmap_args) $(arg odom_args)" launch-prefix="$(arg launch_prefix)">
          <remap from="rgb/image"       to="$(arg rgb_topic_relay)"/>
          <remap from="depth/image"     to="$(arg depth_topic_relay)"/>
          <remap from="rgb/camera_info" to="$(arg camera_info_topic)"/>
          <remap from="rgbd_image"      to="$(arg rgbd_topic_relay)"/>
          <remap from="odom"            to="$(arg odom_topic)"/>
      
          <param name="frame_id"                    type="string" value="$(arg frame_id)"/>
          <param name="odom_frame_id"               type="string" value="$(arg vo_frame_id)"/>
          <param name="publish_tf"                  type="bool"   value="$(arg publish_tf_odom)"/>
          <param name="ground_truth_frame_id"       type="string" value="$(arg ground_truth_frame_id)"/>
          <param name="ground_truth_base_frame_id"  type="string" value="$(arg ground_truth_base_frame_id)"/>
          <param name="wait_for_transform_duration" type="double" value="$(arg wait_for_transform)"/>
          <param name="wait_imu_to_init"            type="bool"   value="$(arg wait_imu_to_init)"/>
          <param name="approx_sync"                 type="bool"   value="$(arg approx_sync)"/>
          <param name="config_path"                 type="string" value="$(arg cfg)"/>
          <param name="queue_size"                  type="int"    value="$(arg queue_size)"/>
          <param name="subscribe_rgbd"              type="bool"   value="$(arg subscribe_rgbd)"/>
          <param name="guess_frame_id"              type="string" value="$(arg odom_guess_frame_id)"/>
          <param name="guess_min_translation"       type="double" value="$(arg odom_guess_min_translation)"/>
          <param name="guess_min_rotation"          type="double" value="$(arg odom_guess_min_rotation)"/>
</node>
```
Parameters in here can be changed. Important variables are these:
```
<remap from="rgb/image"       to="$(arg rgb_topic_relay)"/>
<remap from="depth/image"     to="$(arg depth_topic_relay)"/>
<remap from="rgb/camera_info" to="$(arg camera_info_topic)"/>
<remap from="rgbd_image"      to="$(arg rgbd_topic_relay)"/>
<remap from="odom"            to="$(arg odom_topic)"/>
```

## Potential further development
Do note that suggestions here are based off my blackbox understanding of the crack detection software.
1) Create a python script that reads the video and rewrite the data when a crack is detected. As it is a RGB-D data, we can increase the R value such that a red bounding box can be created. (It is not easy but it is a suggestion)
2) On the flight path, we can change the data color to allow the user to know where a crack was detected.

## Related & Useful Links
1) RTAB-Map parameters https://github.com/introlab/rtabmap/blob/master/corelib/include/rtabmap/core/Parameters.h and http://wiki.ros.org/rtabmap_ros 
2) RTAB-Map as an Open-Source Lidar and Visual SLAM Library for Large-Scale and Long-Term Online Operation https://introlab.3it.usherbrooke.ca/mediawiki-introlab/images/7/7a/Labbe18JFR_preprint.pdf
3) RTAB-Map with LIDAR http://wiki.ros.org/rtabmap_ros/Tutorials/SetupOnYourRobot
4) Transform http://wiki.ros.org/tf
5) RGB-D Handheld Mapping http://wiki.ros.org/rtabmap_ros/Tutorials/HandHeldMapping
6) RTAB-Map launch files https://github.com/introlab/rtabmap_ros/tree/master/launch
