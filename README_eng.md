# `BEVFusion-ROS-TensorRT-CPP`

This repository contains source code and models for BEVFusion online real-time inference using CUDA, TensorRT & ROS.

![](configs/cuda-bevfusion.gif)


# 1 Dependencies Installation

1. assuming that the following are installed:

- Ubuntu 20.04
- ROS2 galactic
- cuda 11.3
- cudnn 8.6.0
- TensorRT 8.5 (source code)

2. set up `rosdep`:

~~~python
# 1. create ros workspace
mkdir -p bevfusion_ws/src

# 2. go into bevfusion_ws/src directory and pull the repo
cd bevfusion_ws/src
git clone https://github.com/linClubs/BEVFusion-ROS-TensorRT.git

# 3. switch to galactic-devel branch
git branch galactic-devel

# 4. use rosdep to install dependencies in src
cd .. 
rosdep install -r -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
~~~

3. To download pretrained models, check out [model download readme](https://github.com/linClubs/BEVFusion-ROS-TensorRT/blob/main/model/readme.md)

4. To output a pytorch model, check out [model output readme](https://github.com/NVIDIA-AI-IOT/Lidar_AI_Solution/blob/master/CUDA-BEVFusion/qat/README.md)

+ Modify the path of `cuda tensorrt cudnn` in `./tool/environment.sh` and run `./tool/build_trt_engine.sh` to generate TensorRT inference model.

~~~python
./tool/build_trt_engine.sh
~~~


5. `rosbag` preparation

+ The official `bevfusion` repo provides pretrained `nuscenes` models

+ To convert the sample nuscenes data to a rosbag for use with this package's node, check out [`nuscenes2rosbag` package](https://github.com/linClubs/nuscenes2rosbag)


+ To convert the rosbag data to the format used by this repo's `galactic-devel` branch, use ROS2 `rosbags` from `pip`:

~~~python
# 1. install rosbags
pip install rosbags

# 2. ros1 bag convert to ros2 bag, result will be "nuscenes-103/"
rosbags-convert nuscenes-103.bag
~~~

+ The sensor parameters in `nuscenes` are fixed and no calibration is required 

However, if you want to hook up real sensors to test your scenarios, you need to **train** a new model and **calibrate** your sensors prior to using this repo.

For sensor calibration, check out this [sensor calibration repo](https://github.com/linClubs/Calibration-Is-All-You-Need)

# 2 Build & Launch

1. Before building, modify the paths of `TensorRT` and `CUDA` in `CMakeLists.txt`:

TODO: THE BELOW SHOULD ALREADY BE IN THE CMAKELISTS FILE, and instead, you can
just run the `environment.sh` script in your current bash session using `source
environment.sh` -- this will define all the environment variables specified in
the `environment.sh` file that the CMakeLists file needs.

So basically, just make sure to run `environment.sh` before you colcon build
anything.

~~~python
...
# cuda
set(CUDA_TOOLKIT_ROOT_DIR /usr/local/cuda-11.3) # CUDA line to change
set(CUDA_INSTALL_TARGET_DIR targets/x86_64-linux)
set(CUDA_INCLUDE_DIRS ${CUDA_TOOLKIT_ROOT_DIR}/${CUDA_INSTALL_TARGET_DIR}/include)
set(CUDA_LIBS ${CUDA_TOOLKIT_ROOT_DIR}/${CUDA_INSTALL_TARGET_DIR}/lib)

# TENSORRT
set(TensorRT_ROOT /home/lin/software/TensorRT-8.5.3.1)  # TensorRT line to change
# set(TensorRT_ROOT ~/share/TensorRT-8.5.3.1)           
set(TensorRT_INCLUDE_DIRS ${TensorRT_ROOT}/include)
set(TensorRT_LIBS ${TensorRT_ROOT}/lib/)
...
~~~

2. Build & launch

+ modify the values of `model_name` and `precision` in `bevfusion_node.launch`

- `model_name: resnet50/resnet50int8/swint`
- `precision:  fp16/int8`

Note that `swint + int8` does not work.

~~~python
# model_name: resnet50/resnet50int8/swint
# precision:  fp16/int8
parameters=[
			{'model_name': 'resnet50'},
			{'precision' : 'int16'}
		]
~~~

~~~python
# 1. build
colcon build --symlink-install

# 2. source your workspace
source install/setup.bash

# 3. launch bevfusion_node
ros2 launch bevfusion bevfusion.launch.py

# 4. play the rosbag
 ros2 bag play nuscenes-103.db3

# 5 display the result in rviz2
rviz2 -d src/BEVFusion-ROS-TensorRT/launch/view.rviz
~~~

---

3. Error Fixes

+ Error 1
if an error pops up during your launch saying " cannot find `tool/simhei.ttf`", search keywords `tool/simhei.ttf` or `UseFont` in your workspace and modify the value of `UseFont` in `/src/common/visualize.cu` to the correct path to `simhei.ttf`


+ Error 2
When running `ros2 run bevfusion bevfusion_node`, if you see `error while loading shared libraries: libspconv.so: cannot open shared object file: No such file or directory`

Fix：
~~~python
# echo LD_LIBRARY_PATH
echo $LD_LIBRARY_PATH

# if libspconv.so path is missing, add it by running the following command in terminal
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/path/to/third_party/3DSparseConvolution/libspconv/lib/x86_64
~~~

---

# References

+ [bevfusion](https://github.com/mit-han-lab/bevfusion)
+ [Lidar_AI_Solution](https://github.com/NVIDIA-AI-IOT/Lidar_AI_Solution)


+ bev perception QQ group - 472648720, join us to learn bev related cool stuff！！！^_^
