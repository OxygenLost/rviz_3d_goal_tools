/*
 * Copyright (c) 2008, Willow Garage, Inc.
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
#ifndef RVIZ_POSE_3D_TOOL_FINAL_H
#define RVIZ_POSE_3D_TOOL_FINAL_H

#include <rviz/tool.h>
#include <rviz/viewport_mouse_event.h>
#include <QCursor>
#include <QEvent>
#include <OgreVector3.h>
#include <OgreQuaternion.h>
#include <ros/ros.h>
#include <cmath>

namespace rviz
{
// 前置声明，不使用智能指针避免不完整类型问题
class Arrow;

/**
 * @brief 修复版本的3D位姿设置工具
 * 
 * 使用原始指针避免不完整类型编译错误
 */
class Pose3DTool : public Tool
{
public:
  // 简化的状态枚举
  enum State
  {
    Position,     // 位置选择状态
    Orientation   // 方向设置状态  
  };

public:
  Pose3DTool();
  ~Pose3DTool() override;

  // Tool接口实现
  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  int processMouseEvent(ViewportMouseEvent& event) override;

protected:
  /**
   * @brief 位姿设置完成回调
   */
  virtual void onPoseSet(double x, double y, double z, double theta) = 0;

  /**
   * @brief 设置主箭头颜色
   */
  void setArrowColor(float r, float g, float b, float a = 1.0f);

  /**
   * @brief 查找Interact工具
   */
  Tool* findInteractTool();

private:
  // 核心处理方法
  int processPositionState(ViewportMouseEvent& event);
  int processOrientationState(ViewportMouseEvent& event);

  // 3D点选择方法
  bool get3DPointFromSelection(ViewportMouseEvent& event, Ogre::Vector3& result_pos);
  bool projectRayToGroundPlane(ViewportMouseEvent& event, Ogre::Vector3& result_pos);
  
  // 高度精度改善方法
  bool refineHeightUsing3DPatch(ViewportMouseEvent& event, const Ogre::Vector3& original_pos, Ogre::Vector3& refined_pos);
  bool refineHeightUsingRaycast(ViewportMouseEvent& event, const Ogre::Vector3& original_pos, Ogre::Vector3& refined_pos);
  
  // 方向更新方法
  void updateOrientationFromMouse(ViewportMouseEvent& event);
  
  // 3D平面投影方法
  bool getPointOnPlaneFromWindowXY(Ogre::Viewport* viewport, const Ogre::Plane& plane, int x, int y, Ogre::Vector3& result_point);
  
  // 工具状态管理
  void resetTool();
  void cancelOperation(bool deactivate_tool = false);

  // 箭头管理 - 使用原始指针避免不完整类型问题
  void initializeMainArrow();
  void safeDeleteArrow();
  bool safeShowArrow();
  void safeHideArrow();
  
  // 方向箭头创建和更新
  void createOrientationArrow(const Ogre::Vector3& position);

private:
  // 使用原始指针而不是智能指针
  Arrow* main_arrow_;    // 主方向箭头（原始指针）
  
  // 状态变量
  State current_state_;
  Ogre::Vector3 selected_position_;      // 选中的3D位置
  double current_angle_;                 // 当前方向角度（用于箭头显示）
  double target_angle_;                  // 目标角度（用于最终发布）
  
  // 初始点击位置（用于计算拖拽方向）
  int initial_click_x_;
  int initial_click_y_;
  
  // UI状态
  QCursor standard_cursor_;
  QCursor crosshair_cursor_;

  // 空实现方法（保持接口兼容性）
  int processHeightState(ViewportMouseEvent& event);
  void updateHeightIndicators();
  void createHeightArrow(const Ogre::Vector3& position, double angle);
  void clearHeightIndicators();
  void updateOrientationArrow();
  void safeUpdateArrowOrientation();
};

} // namespace rviz

#endif // RVIZ_POSE_3D_TOOL_FINAL_H