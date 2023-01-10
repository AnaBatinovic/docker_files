## Docker
Install Docker using installation instruction found [here](https://docs.docker.com/engine/install/ubuntu/).

Run Dockerfile from the project root directory using the following commands:
```bash
# Build a Dockerfile
docker build -t realsense_detectron2_ros_cont realsense_detectron2_ros 

# Run the realsense_detectron2_ros_cont container for the fist time
export DOCKER_BUILDKIT=1
./realsense_ros/first_run.sh

#  Run the container 
docker start -i realsense_detectron2_ros_cont

# Stop the conatainer
docker stop realsense_detectron2_ros_cont

# Delete the container
docker rm realsense_detectron2_ros_cont
```

## RealSense camera - usage Instructions for ROS

### Start the camera node
To start the camera node in ROS:

```bash
roslaunch realsense2_camera rs_camera.launch
```

This will stream all camera sensors and publish on the appropriate ROS topics.

Other stream resolutions and frame rates can optionally be provided as parameters to the 'rs_camera.launch' file.

### Published Topics
The published topics differ according to the device and parameters.
After running the above command with D435i attached, the following list of topics will be available (This is a partial list. For full one type `rostopic list`):
- /camera/color/camera_info
- /camera/color/image_raw
- /camera/color/metadata
- /camera/depth/camera_info
- /camera/depth/image_rect_raw
- /camera/depth/metadata
- /camera/extrinsics/depth_to_color
- /camera/extrinsics/depth_to_infra1
- /camera/extrinsics/depth_to_infra2
- /camera/infra1/camera_info
- /camera/infra1/image_rect_raw
- /camera/infra2/camera_info
- /camera/infra2/image_rect_raw
- /camera/gyro/imu_info
- /camera/gyro/metadata
- /camera/gyro/sample
- /camera/accel/imu_info
- /camera/accel/metadata
- /camera/accel/sample
- /diagnostics

If you want to use/check camera depth run:

```bash
roslaunch realsense2_camera rs_camera.launch
```
and the topic `/camera/depth_registered/points` will be available.

For more information, please refer to [realsense-ros](https://github.com/IntelRealSense/realsense-ros).

## ROS Semantic Segmentation

A number of models are available. Default is **resnet50dilated** for encoder and **ppm_deepsup** for decoder.
Download pretrained checkpoints for models from [CSAIL Website](http://sceneparsing.csail.mit.edu/model/pytorch) and copy them to ***src/ros-semantic-segmentation-pytorch/ckpt/{modelname}/***

Copy file from the local folder to the folder in docker container (an example for ade20k-resnet50dilated-ppm_deepsup):

```bash
#  Run the container 
docker start -i realsense_sem_seg_cont

cd ros-semantic-segmentation-pytorch
mkdir ckpt
cd ckpt
mkdir ade20k-resnet50dilated-ppm_deepsup

# Run from local folder
docker cp /source_path/encoder_epoch_20.pth ID_container:/root/catkin_ws/src/ros-semantic-segmentation-pytorch/ckpt/ade20k-resnet50dilated-ppm_deepsup 
docker cp /source_path/decoder_epoch_20.pth ID_container:/root/catkin_ws/src/ros-semantic-segmentation-pytorch/ckpt/ade20k-resnet50dilated-ppm_deepsup 
```
How to find container ID?

```bash
docker ps -a
```
### Inference

Launch file :
- Input topic -> raw image rgb topic.
- gpu_id -> set gpu id for multiple gpu system else 0.
- cfg_filepath -> path to config file. Should be according to model.
- model_ckpt_dir -> path to directory containing downloaded checkpoints.

Configuration file contains option "imgSizes" which takes a tuple of heights, over which input image is resized for inference. This can be tweaked according to GPU capablities.

```bash
roslaunch semantic_segmentation_ros semantic_segmentation.launch
```

### Image transport

If the input in the semgentation node is topic `/camera/color/image_raw/compressed` ([sensor_msgs/CompressedImage](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/CompressedImage.html)), you need to republish the topic to `/camera/color/image_raw`([sensor_msgs/Image](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html)):

```bash
rosrun image_transport republish compressed in:=/camera/color/image_raw raw out:=/camera/color/image_raw
```
