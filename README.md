# 基于视觉演示的机械臂控制系统

## 安装

科隆本仓库至本地并创建虚拟环境。

```
sudo apt install git
sudo apt install python3 python3-pip

git clone 
cd rcsmbvd

conda create -n rcsmbvd python=3.9
conda activate rcsmbvd
```

### 环境

- 系统：Ubuntu 20.04.6 LTS
- 显卡：Nvidia GeforceRTX 2060
- 显卡驱动版本：515.65.01
- CUDA 版本：11.7
- 机械臂：Kinova Gen3 Lite

### 机械臂相关

#### ROS 控制

请按照[ROS 官网](http://wiki.ros.org/ROS/Tutorials)安装 noetic 版本。

安装[Kinova ROS](https://github.com/Kinovarobotics/ros_kortex)

请在项目根目录下执行以下命令

```
sudo rosdep init
rosdep update

sudo python3 -m pip install conan==1.59.0

conan config set general.revisions_enabled=1
conan profile new default --detect > /dev/null
conan profile update settings.compiler.libcxx=libstdc++11 default

mkdir -p catkin_workspace/src
cd catkin_workspace/src
git clone https://github.com/Kinovarobotics/ros_kortex.git

cd ../
rosdep install --from-paths src --ignore-src -y

catkin_make
source devel/setup.bash
```

编译本项目相关 ROS Package

请在项目根目录下执行以下命令

```
mv related_resources/rcsmbvd catkin_workspace/src/rcsmbvd
catkin_make
```

#### API 控制

安装[kortex_api](https://github.com/Kinovarobotics/kortex)

请在项目根目录下执行以下命令

```
cd related_resources
python3 -m pip install kortex_api-2.3.0.post34-py3-none-any.whl
```

### 人体 3D 姿态检测相关

本项目依赖[Frankmocap](https://github.com/facebookresearch/frankmocap)。可根据 Frankmocap 仓库中的相关指引安装其依赖。

本仓库已包含 Frankmocap 相关资源，请使用以下命令安装依赖

```
# Install basic dependencies
sudo apt-get install libglu1-mesa libxi-dev libxmu-dev libglu1-mesa-dev freeglut3-dev libosmesa6-dev

# Install ffmpeg
sudo apt-get install ffmpeg

# Install pytorch and torchvision
conda install pytorch==1.10.0 torchvision==0.11.0 torchaudio==0.10.0 cudatoolkit=11.3 -c pytorch -c conda-forge
```

安装[Detectron2](https://github.com/facebookresearch/detectron2)用于手部检测

```
python -m pip install detectron2 -f https://dl.fbaipublicfiles.com/detectron2/wheels/cu113/torch1.10/index.html
```

在项目根目录下使用下列命令安装 hand detector 相关依赖

```
cd detectors/hand_object_detector/lib
python setup.py build develop
```

## 使用

### 适用预录制视频输入

#### Gazebo 模拟仿真

```
# Start Gazebo simulation
roslaunch kortex_gazebo spawn_kortex_robot.launch arm:="gen3_lite"

# Start the human body 3D detection node
python img2joints.py --input_path path/to/your/video

# Start the robotic arm control node
python -m ros_control.gazebo_service_ctr __ns:=my_gen3_lite
```

#### 机械臂实物控制

```
# Start the Kortex driver
roslaunch kortex_driver kortex_driver.launch  arm:="gen3_lite"

# Start the human body 3D detection node
python img2joints.py --input_path path/to/your/video

# Start the robotic arm control node
python -m ros_control.real_arm_service_ctr
```

### 使用实时摄像头输入

```
# Start ROS
roscore

# tart the human body 3D detection node
python img2joints.py --input_path webcam

# Start the robotic arm control node
python -m ros_control.twist_ctr
```
