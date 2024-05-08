# `BEVFusion-ROS-TensorRT-CPP`

This repository contains source code and models for BEVFusion online real-time inference using CUDA, TensorRT & ROS.

![](configs/cuda-bevfusion.gif)


# Run on Host OS

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

+ To convert the sample nuscenes data to a rosbag for use with this package's node, check out [`nuscenes2rosbag` package](https://github.com/linClubs/nuscenes2rosbag). This is what we'll use to create a ros1 bag file.
  
+ Then, convert the ros1 bag file to a format that we can use with ros2 using the `rosbags` package from pypi.

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


# 2. Run in Docker Container
A Dockerfile has been created to automate creating the environment and packages
required by BEVFusion. **It is HIGHLY recommended you use this approach** so
that you don't have to worry about NVIDIA drivers and packages.

## Prerequisites
- Ubuntu 20.04
- Docker. Do yourself a favor and [install it via](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository) `apt`.
- An NVIDIA-GPU with **PASCAL microarchitecture** or newer. Most cards from the
  last ten years should be fine--see [this
  page](https://en.wikipedia.org/wiki/Category:Nvidia_microarchitectures) for
  more details. This requirement comes from the fact that this docker image uses
  NVIDIA's PyTorch 22.09 image as its base--as this image contains the correct
  CUDA Toolkit and CuDNN versions that work with BEVFusion (that we know of).
  See [this compatability matrix](https://docs.nvidia.com/deeplearning/frameworks/support-matrix/index.html#framework-matrix-2022) for more details--you may find that other
  versions work.
- [NVIDIA Container
  Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html#installation).
  Save yourself some headache and [install it with](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html#installing-with-apt) `apt`.
- Install ORSF [rocker](https://github.com/osrf/rocker). You can run the
  following or or follow the instructions in their repo to install it however
  you'd like. *rocker isn't strictly necessary, but it makes running these
  docker images MUCH more straightforward.*


## Outline
1. Create a new ROS workspace directory and clone this repository into its src
   folder.
2. Build the Docker image.
3. Create a new container from the docker image and mount it on a ROS workspace
   folder.
4. Download model weights
5. Generate TensorRT Engine from model weights
6. Build your workspace
7. Run the BEVFusion ROS Node 

## Steps
### 1. Create a new ROS workspace.
Navigate to where you'd like to house this workspace and run the following:
```
mkdir -p bevfusion_ros_ws/src
```

### 2. Build the Docker Image
1. Clone this repository to the workspace's `src` folder that you made above.
	```
	git clone git@github.com:nlitz88/BEVFusion-ROS-TensorRT.git
	```

2. Navigate to the docker folder in this repo and build the image using the
   Dockerfile.
	```
	cd BEVFusion-ROS-TensorRT/docker
	docker build -t bevfusion_ros .
	```
   _FULL DISCLOSURE: I ran out of time at the end of the semester and just threw this Dockerfile together real quick without testing it. If this doesn't build correctly or install everything correctly, if you're just looking to get started, use the PyTorch base image specified in the Dockerfile. Run a container from that image and install ROS2 galactic within there. If you fix the dockerfile, create an issue and maybe we can merge into into the `ros2-galactic` branch._

## 3. Run a Container From the Image
1. Run the docker container using the `rocker` command line with the workspace
   directory you created above mounted as a volume.

   ```
   rocker --user --ssh --x11 --nvidia --network host --privileged --name bevfusion_ros_cont --volume /path/to/your/workspace/:/workspace -- bevfusion_ros
   ```

   rocker's output is usually pretty long and verbose--so don't be worried if
   you see it spam a ton of messages. When it finishes, it should drop you into
   a new shell session within the container.

   *Note: Some of the flags in the example rocker command above are not strictly
   necessary (like `privileged`). I am including them as they have solved
   certain, hard-to-forsee issues I have encountered in the past.*

## 4. Download Model Weights
***NOTE:** Steps 4 and 5 DO NOT need to be completed more than once.*

Now, before we can build our workspace and get things running, we first have to
download the weights for the models that BEVFusion relies on. Use the links
below to download the model files and place them in the `model` directory so
that it has the directory structure shown below.

[Document Reference](https://github.com/NVIDIA-AI-IOT/Lidar_AI_Solution/tree/master/CUDA-BEVFusion)
- Download the model ([Google Drive](https://drive.google.com/file/d/1bPt3D07yyVuSuzRAHySZVR2N15RqGHHN/view?usp=sharing)) or ([Baidu Drive](https://pan.baidu.com/s/1_6IJTzKlJ8H62W5cUPiSbA?pwd=g6b4))
- Download the test dataset ([Google Drive](https://drive.google.com/file/d/1RO493RSWyXbyS12yWk5ZzrixAeZQSnL8/view?usp=sharing)) or ([Baidu Drive](https://pan.baidu.com/s/1ED6eospSIF8oIQ2unU9WIQ?pwd=mtvt))


After downloading and unzipping the model, copy it to the `BEVFusion-ROS-TensorRT` directory. The directory structure is as follows:

~~~
BEVFusion-ROS-TensorRT/model
.
├── resnet50
│   ├── bevfusion-det.pth
│   ├── camera.backbone.onnx
│   ├── camera.vtransform.onnx
│   ├── default.yaml
│   ├── fuser.onnx
│   ├── head.bbox.onnx
│   └── lidar.backbone.xyz.onnx
├── resnet50int8
│   ├── bevfusion_ptq.pth
│   ├── camera.backbone.onnx
│   ├── camera.vtransform.onnx
│   ├── default.yaml
│   ├── fuser.onnx
│   ├── head.bbox.onnx
│   └── lidar.backbone.xyz.onnx
└── swint
    ├── camera.backbone.onnx
    ├── camera.vtransform.onnx
    ├── default.yaml
    ├── fuser.onnx
    ├── head.bbox.onnx
    └── lidar.backbone.xyz.onnx
~~~


## 5. Generate TensorRT Engines from Model Weight Files
As long as the models are in the directory structure specified above, you should
now be able to generate TensorRT engine files for each of the backbones you just
downloaded. To create these:

1. Navigate to the [`tool`](tool/) directory in this repo
	```
	cd tool/
	```
2. Run the `build_trt_engine.sh` script.

	```
	source ./build_trt_engine.sh
	```

If all goes well, you should see a few `plan` files appear--these are the TRT
engine files.

## 6. Build the ROS Workspace From Within the Container
1. In the shell session within the container, navigate to the workspace folder
   we mounted when we started the container.
   ```
   cd /workspace
   ```
2. Source the ROS installation workspace.
	```
	source /opt/ros/$ROS_DISTRO/setup.bash
	```

	$ROS_DISTRO should == `galactic` unless this image is updated later.

3. Use Colcon to build the bevfusion package.
	```
	colcon build --packages-select bevfusion
	```

	You can also absolutely just build the entire workspace--we just select
	bevfusion for the sake of getting started with this package specifically.

## 7. Launch the BEVFusion Node
1. Use the `bevfusion.launch.py` launch file to run the BEVFusion node:
	```
	ros2 launch bevfusion bevfusion.launch.py
	```

## 8. Preparing Input For the BEVFusion Node.
Now, unless you already have some kind of input source, the node isn't going
to publishing anything meaningful. To get a preview of how the model works,
you'll likely want to download the [mini-portion of the nuscenes
dataset](https://www.nuscenes.org/nuscenes#download), use
[nuscenes2rosbag](https://github.com/nlitz88/nuscenes2rosbag) ROS1 package
to export specific scenes to a  ROS1 rosbag format (`.bag` file), and then
use the [rosbags](https://ternaris.gitlab.io/rosbags/) package to convert
the ROS1 bag to a ROS2 bag. Note that the nuscenes2rosbag tool is basically
a ROS1 node, and it is probably best if you build and use the docker image
that package provides separately from this one!

Alternatively, there are scenes that we have already prepared using the
process above that you can download [from Google Drive
here](https://drive.google.com/file/d/1O3EsWIw-_8RWUs8QU9w4mwMPsdiMvmd_/view?usp=sharing).

## Additional Notes
1. If you need an additional terminal within the container, you can run the
   following to create a new shell  session from a separate terminal:
	```
	docker exec -it bevfusion_ros_cont /bin/bash
	```

	From this session, you could run `rqt` or `rviz2` (after sourcing the
	installation workspace), for example, to monitor/debug the other ROS nodes
	you have running within the container.

# References

+ [bevfusion](https://github.com/mit-han-lab/bevfusion)
+ [Lidar_AI_Solution](https://github.com/NVIDIA-AI-IOT/Lidar_AI_Solution)
+ bev perception QQ group - 472648720, join us to learn bev related cool stuff！！！^_^
