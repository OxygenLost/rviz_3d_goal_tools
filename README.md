# RViz 3D Goal Tools | RViz 3D目标工具

[English](#english) | [中文](#chinese)

## English

### Overview
RViz 3D Goal Tools is a ROS package that provides enhanced visualization and interaction tools for setting 3D goals in RViz. This package extends RViz's capabilities by offering intuitive tools for manipulating and visualizing 3D navigation goals.

### Features
- Interactive 3D goal setting in RViz
- Customizable visualization markers
- Support for position and orientation control
- Easy integration with existing ROS navigation stacks

### Installation
```bash
# Clone the package to your ROS workspace
cd ~/catkin_ws/src
git clone https://github.com/your_username/rviz_3d_goal_tools.git

# Build the workspace
cd ..
catkin_make
```

### Usage
1. Add the plugin to your RViz configuration
2. Use the toolbar buttons to activate the 3D goal tools
3. Click and drag in the 3D view to set goals

### Dependencies
- ROS (tested on ROS Noetic)
- RViz
- Qt5

### Requirements
- Point cloud map is required for operation
- The point cloud map should preferably be in box shape for optimal performance

### Known Issues
- When using RViz with GPU rendering, there might be occasional issues where the tool fails to obtain goals
- If you encounter this issue, try switching to CPU rendering mode

### License
This package is released under the MIT License.

---

## Chinese

### 概述
RViz 3D目标工具是一个ROS软件包，为RViz提供增强的3D目标可视化和交互工具。该软件包通过提供直观的工具来操作和可视化3D导航目标，扩展了RViz的功能。

### 特性
- 在RViz中交互式设置3D目标
- 可自定义的可视化标记
- 支持位置和方向控制
- 易于与现有ROS导航功能集成

### 安装方法
```bash
# 将软件包克隆到ROS工作空间
cd ~/catkin_ws/src
git clone https://github.com/your_username/rviz_3d_goal_tools.git

# 编译工作空间
cd ..
catkin_make
```

### 使用方法
1. 将插件添加到RViz配置中
2. 使用工具栏按钮激活3D目标工具
3. 在3D视图中点击并拖动以设置目标

### 依赖项
- ROS（在ROS Noetic上测试通过）
- RViz
- Qt5

### 使用要求
- 需要配合点云地图使用
- 点云地图最好使用box形状以获得最佳效果

### 已知问题
- 在使用RViz的GPU渲染模式时，可能会偶发性地出现无法获取目标的情况
- 如果遇到此问题，可以尝试切换到CPU渲染模式

### 许可证
本软件包基于MIT许可证发布。 