# 项目运行与环境配置说明

## 1. 项目运行环境

本项目建议在以下环境中运行：

* 操作系统：Ubuntu 20.04
* ROS 版本：ROS Noetic

> 注意：不建议在虚拟机中运行本项目。虚拟机可能导致图像识别推理时间明显变长，从而影响机械臂抓取任务的正常执行。

---
![page1](https://github.com/pengjiaosu/robot_arm_grasp/blob/main/图片1.png)
![page2](https://github.com/pengjiaosu/robot_arm_grasp/blob/main/图片2.png)
![page3](https://github.com/pengjiaosu/robot_arm_grasp/blob/main/图片3.png)
## 2. 配置环境

进入项目文件夹，在项目根目录下打开终端，依次执行以下命令，确保每一步均无报错。

### 2.1 安装 ROS 相关依赖

```bash
sudo apt install ros-noetic-moveit ros-noetic-joint-trajectory-controller ros-noetic-moveit-core ros-noetic-moveit-ros-planning ros-noetic-moveit-ros-planning-interface
```

### 2.2 安装 Python 依赖

```bash
pip install -r Resources/requirements.txt
```

### 2.3 编译项目

```bash
catkin_make
```

### 2.4 配置 Gazebo 模型文件

将项目 `Resources` 文件夹中的 `models` 文件夹复制到用户主目录下的 `.gazebo` 文件夹中，并与原有模型文件进行合并。

目标路径一般为：

```bash
~/.gazebo/models
```

如果 `.gazebo/models` 文件夹不存在，可以先手动创建：

```bash
mkdir -p ~/.gazebo/models
```

然后将模型文件复制进去：

```bash
cp -r Resources/models/* ~/.gazebo/models/
```

---

## 3. 运行程序

项目运行时需要打开三个终端，分别启动 Gazebo 与 RViz 仿真环境、控制器节点以及用户交互节点。

---

### 3.1 启动 Gazebo 与 RViz

在项目文件夹下打开第一个终端，依次运行：

```bash
source devel/setup.bash
roslaunch ar3_control moveit_gazebo.launch
```

运行成功后，将启动 Gazebo 仿真环境和 RViz 可视化界面。

---

### 3.2 启动机械臂控制与视觉识别节点

在项目文件夹下打开第二个终端，依次运行：

```bash
source devel/setup.bash
roslaunch ar3_control ar3_yolo_controller.launch
```

运行成功后，RViz 中会出现摄像机图像，机械臂会自动回到初始位置。

---

### 3.3 启动用户交互节点

在项目文件夹下打开第三个终端，依次运行：

```bash
source devel/setup.bash
rosrun ar3_control get_input
```

启动后，可根据终端提示输入初始化命令、待抓取数字编号以及目标放置位置，完成机械臂抓取与放置任务。
