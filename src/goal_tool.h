/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef RVIZ_GOAL_3D_TOOL_SIMPLIFIED_H
#define RVIZ_GOAL_3D_TOOL_SIMPLIFIED_H

#ifndef Q_MOC_RUN
#include <QObject>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "pose_tool.h"
#endif

namespace rviz
{
class StringProperty;
class BoolProperty;

/**
 * @brief 简化的3D导航目标设置工具
 * 
 * 继承简化的Pose3DTool，提供两阶段3D目标设置：
 * 1. 点击获取3D位置（包含真实高度）
 * 2. 拖拽设置方向角度
 */
class Goal3DTool : public Pose3DTool
{
  Q_OBJECT

public:
  Goal3DTool();
  ~Goal3DTool() override = default;
  
  void onInitialize() override;

protected:
  /**
   * @brief 位姿设置完成时的处理
   * @param x X坐标
   * @param y Y坐标  
   * @param z Z坐标（从3D点获取的真实高度）
   * @param theta 绕Z轴旋转角度(弧度)
   */
  void onPoseSet(double x, double y, double z, double theta) override;

private Q_SLOTS:
  void updateTopic();

private:
  // ROS相关
  ros::NodeHandle nh_;
  ros::Publisher pub_;

  // 属性
  StringProperty* topic_property_;

  // 辅助方法
  void initializeProperties();
  bool validatePublisher() const;
  geometry_msgs::PoseStamped createPoseStampedMessage(double x, double y, double z, double theta) const;
  void logGoalInfo(const geometry_msgs::PoseStamped& goal, double theta) const;
};

} // namespace rviz

#endif // RVIZ_GOAL_3D_TOOL_SIMPLIFIED_H