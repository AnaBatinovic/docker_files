## Docker
Install Docker using installation instruction found [here](https://docs.docker.com/engine/install/ubuntu/).

Run Dockerfile from the project root directory using the following commands:
```bash
# Build a Dockerfile
docker build -t orb_slam3_ros_wrapper orb_slam3_ros_wrapper 

# Run the orb_slam3_cont container for the fist time
export DOCKER_BUILDKIT=1
./orb_slam3_ros_wrapper/first_run.sh

#  Run the container 
./orb_slam3_ros_wrapper/start_docker.sh

# Stop the conatainer
docker stop orb_slam3_ros_wrapper_cont

# Delete the container
docker rm orb_slam3_ros_wrapper_cont
```
### Test ORB_SLAM3_ROS in the Docker container

The package is forked from [thien94/orb_slam3_ros_wrapper](https://github.com/thien94/orb_slam3_ros_wrapper).

To test the package on the Euroc dataset, please downoload the data from [euroc_mav_dataset](http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/).

To run the package in the docker container, in one terminal run:

```
roslaunch orb_slam3_ros_wrapper euroc_monoimu.launch
```

In another terminal run:

```
rosbag play MH_01_easy.bag
```

### Publish Topic
The following topics are published by each node:
- `/orb_slam3/map_points` ([`PointCloud2`](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html)): all keypoints being tracked.
- `/orb_slam3/camera_pose` ([`PoseStamped`](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PoseStamped.html)): current left camera pose in world frame, as returned by ORB-SLAM3.
- `tf`: transformation from camera frame to world frame.