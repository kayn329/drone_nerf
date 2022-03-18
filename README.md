# drone_nerf<br>
## Overview<br>
Repository for CS231A Final Project. We collect data in hardware in the form of image and pose pairs using an IntelRealsense camera and an Optitrack Motion Capture system. We then post process our data by performing image segmentation and converting our poses to relative frames. We finally train a NeRF using these image and pose pairs. Then we use the resulting NeRF to compute a potential trajectory that will make contact with the target. <br>
## Data Collection
To collect our data, we use the [IntelRealSense ROS Wrapper](https://github.com/IntelRealSense/realsense-ros) to collect images, and rosbag the image topic and pose topics.<br>
## Post Processing<br>
To segment our images, we run `segmentation_code.ipynb`, and to convert our poses to be used for NeRF training we run `convert_poses.py`.<br>
## Training<br>
In our training we use the [NeRF codebase](https://github.com/bmild/nerf) with our own config files located in the training folder.<br>
## Trajectory Generation<br>
To generate a potential trajectory through the target object, we modify some code from the original NeRF codebase to find the points corresponding to densities above a certain threshold. We then plan a best fit line through these points to plan a path that is most likely to make contact with the target object.<br>
## Results<br>
Inertial Frame poses, Unsegmented images, Hand-held test images:<br>
![alt text](https://github.com/kayn329/drone_nerf/blob/main/images/nerf_test_1.gif)<br>
Inertial Frame poses, Segmented images, Hand-held test images:<br>
![alt text](https://github.com/kayn329/drone_nerf/blob/main/images/nerf_test_2.gif)<br>
Relative Frame poses, Segmented images, Hand-held test images:<br>
![alt text](https://github.com/kayn329/drone_nerf/blob/main/images/nerf_test_3.gif)<br>
