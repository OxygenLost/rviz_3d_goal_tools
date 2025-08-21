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

#include "pose_tool.h"
#include <rviz/viewport_mouse_event.h>
#include <rviz/display_context.h>
#include <rviz/selection/selection_manager.h>
#include <rviz/visualization_manager.h>
#include <rviz/view_manager.h>
#include <rviz/render_panel.h>
#include <rviz/ogre_helpers/arrow.h>
#include <rviz/tool_manager.h>

#include <OgreCamera.h>
#include <OgreRay.h>
#include <OgreViewport.h>
#include <OgrePlane.h>
#include <OgreCamera.h>
#include <OgreRay.h>
#include <cmath>
#include <cstdlib>  // for abs
#include <vector>
#include <string>

namespace rviz
{

Pose3DTool::Pose3DTool() 
  : Tool()
  , main_arrow_(nullptr)
  , current_state_(Position)
  , current_angle_(0.0)
  , selected_position_(Ogre::Vector3::ZERO)
  , standard_cursor_(Qt::ArrowCursor)
  , crosshair_cursor_(Qt::CrossCursor)
{
  ROS_INFO("Pose3DTool: 完整版本构造");
}

Pose3DTool::~Pose3DTool()
{
  ROS_INFO("Pose3DTool: 完整版本析构");
  safeDeleteArrow();
}

void Pose3DTool::onInitialize()
{
  ROS_INFO("=== Pose3DTool::onInitialize() 完整版本 ===");
  
  // 初始化箭头显示
  initializeMainArrow();
  
  ROS_INFO("=== Pose3DTool 完整版本初始化完成 ===");
}

void Pose3DTool::activate()
{
  ROS_INFO("activate() - 进入位置选择模式");
  
  // 重置工具状态
  resetTool();
  
  // 设置光标为十字形
  context_->getViewManager()->getRenderPanel()->setCursor(crosshair_cursor_);
  
  // 设置状态显示
  setStatus("按住左键拖拽设置目标位置和方向，右键退出工具");
  
  ROS_INFO("activate() 完成");
}

void Pose3DTool::deactivate()
{
  ROS_INFO("deactivate() - 工具停用");
  
  // 隐藏箭头
  safeHideArrow();
  
  // 恢复标准光标
  context_->getViewManager()->getRenderPanel()->setCursor(standard_cursor_);
  
  // 重置状态
  resetTool();
}

int Pose3DTool::processMouseEvent(ViewportMouseEvent& event)
{
  // ROS_INFO("=== processMouseEvent: 状态=%d, 事件类型=%d ===", current_state_, event.type);
  
  // 根据当前状态分发事件处理
  if (current_state_ == Position)
  {
    return processPositionState(event);
  }
  else if (current_state_ == Orientation)
  {
    return processOrientationState(event);
  }
  
  ROS_INFO("未知状态，跳过事件处理");
  return 0;
}

// ==================== 位置选择状态处理 ====================

int Pose3DTool::processPositionState(ViewportMouseEvent& event)
{
  // 详细事件类型日志
  // ROS_INFO("=== processPositionState: 事件类型=%d, leftDown=%s, rightDown=%s ===", 
  //          event.type, event.leftDown() ? "true" : "false", event.rightDown() ? "true" : "false");
  
  if (event.type == QEvent::MouseMove)
  {
    // 在位置选择状态下，鼠标移动时提供简单的状态更新
    // ROS_INFO("位置选择状态：鼠标移动 - 鼠标位置(%d, %d)", event.x, event.y);
    return 0; // 不需要重新渲染
  }
  else if (event.type == QEvent::MouseButtonPress && event.right())
  {
    // 右键直接退出工具
    ROS_INFO("=== 位置选择状态：右键退出工具 ===");
    cancelOperation(true);  // 完全取消工具激活
    return Render;
  }
  else if (event.rightDown())
  {
    // 右键按下状态，也执行退出工具
    ROS_INFO("=== 位置选择状态：右键按下退出工具 ===");
    cancelOperation(true);
    return Render;
  }
  else if (event.type == QEvent::MouseButtonPress && event.left())
  {
    ROS_INFO("=== 鼠标按下：开始位置选择和拖拽！坐标(%d, %d) ===", event.x, event.y);
    
    // 保存初始点击位置（RViz经典2D Nav Goal方式）
    initial_click_x_ = event.x;
    initial_click_y_ = event.y;
    ROS_INFO("保存初始点击位置: (%d, %d)", initial_click_x_, initial_click_y_);
    
    // 真正的3D点选择（已验证工作正常）
    ROS_INFO("开始调用get3DPointFromSelection...");
    Ogre::Vector3 position;
    bool selection_success = get3DPointFromSelection(event, position);
    ROS_INFO("get3DPointFromSelection结果: %s", selection_success ? "成功" : "失败");
    
    if (selection_success)
    {
      // 成功获取3D点，保存位置并切换到方向设置状态
      selected_position_ = position;
      current_state_ = Orientation;
      current_angle_ = 0.0;  // 初始角度
      
      ROS_INFO("✅ 位置选择完成: (%.3f, %.3f, %.3f)，切换到方向设置状态", 
               position.x, position.y, position.z);
      
      // 更新状态显示
      setStatus("拖拽中...松开鼠标确认方向，右键退出工具");
      
      // 创建方向指示箭头
      ROS_INFO("创建方向指示箭头...");
      createOrientationArrow(position);
      
      return Render;
    }
    else
    {
      ROS_WARN("⚠️ 无法在此位置获取有效的3D点，坐标(%d, %d) - 请尝试点击有几何体的区域", event.x, event.y);
      setStatus("点击了空白区域，请点击有几何体的位置");
      return 0;
    }
  }
  else
  {
    ROS_INFO("processPositionState: 未处理的事件类型 %d", event.type);
  }
  
  return 0;
}

// ==================== 方向设置状态处理 ====================

int Pose3DTool::processOrientationState(ViewportMouseEvent& event)
{
  // ROS_INFO("=== processOrientationState: 事件类型=%d, left=%s, leftDown=%s ===", 
  //          event.type, event.left() ? "true" : "false", event.leftDown() ? "true" : "false");
  
  if (event.type == QEvent::MouseMove && event.left())
  {
    // 拖拽中：实时更新方向预览
    // ROS_INFO("拖拽中：更新方向预览，坐标(%d, %d)", event.x, event.y);
    updateOrientationFromMouse(event);
    updateOrientationArrow();
    return Render;
  }
  else if (event.type == QEvent::MouseButtonRelease)
  {
    // 鼠标松开：确认最终位置和方向
    ROS_INFO("=== 鼠标松开：方向设置完成！事件详情: left=%s ===", 
             event.left() ? "true" : "false");
    
    // 确保角度是最新的
    updateOrientationFromMouse(event);
    
    ROS_INFO("✅ 最终目标: 位置(%.3f, %.3f, %.3f), 角度%.1f°", 
             selected_position_.x, selected_position_.y, selected_position_.z,
             current_angle_ * 180.0 / M_PI);
    
    // 发布最终目标（使用原始计算的目标角度，不是显示用的反向角度）
    ROS_INFO("📤 发布目标角度: 显示角度%.1f° -> 目标角度%.1f°", 
             current_angle_ * 180.0 / M_PI, target_angle_ * 180.0 / M_PI);
    onPoseSet(selected_position_.x, selected_position_.y, selected_position_.z, target_angle_);
    
    // 隐藏箭头，但不重置工具状态，保持工具激活
    safeHideArrow();
    current_state_ = Position;  // 回到位置选择状态，准备下一个目标
    setStatus("按住左键拖拽设置目标位置和方向，右键退出工具");
    
    return Render;  // 移除Finished标志，保持工具激活
  }
  else if (event.type == QEvent::MouseMove && !event.left())
  {
    // 鼠标移动但没有按下：可能是意外情况，记录但不处理
    ROS_INFO("方向状态下鼠标移动但无按键，坐标(%d, %d)", event.x, event.y);
    return 0;
  }
  else if (event.type == QEvent::MouseButtonPress && event.right())
  {
    // 右键直接退出工具
    ROS_INFO("=== 方向设置状态：右键退出工具 ===");
    cancelOperation(true);  // 完全取消工具激活
    return Render;
  }
  else if (event.rightDown())
  {
    // 右键按下状态，也执行退出工具
    ROS_INFO("=== 方向设置状态：右键按下退出工具 ===");
    cancelOperation(true);
    return Render;
  }
  else
  {
    ROS_INFO("processOrientationState: 未处理的事件 - 类型=%d, left=%s", 
             event.type, event.left() ? "true" : "false");
  }
  
  return 0;
}

// ==================== 3D点选择实现 ====================

bool Pose3DTool::get3DPointFromSelection(ViewportMouseEvent& event, Ogre::Vector3& result_pos)
{
  ROS_INFO("=== get3DPointFromSelection 开始 ===");
  
  if (!context_)
  {
    ROS_ERROR("Display context为空");
    return false;
  }
  
  // 检查必要的组件
  if (!context_->getSelectionManager())
  {
    ROS_ERROR("SelectionManager为空");
    return false;
  }
  
  if (!event.viewport)
  {
    ROS_ERROR("Viewport为空");
    return false;
  }
  
  ROS_INFO("尝试使用SelectionManager获取3D点...");
  
  try 
  {
    // 确保我们有有效的SelectionManager
    SelectionManager* sel_mgr = context_->getSelectionManager();
    if (!sel_mgr)
    {
      ROS_ERROR("SelectionManager为空");
      return false;
    }
    
    // 关键：确保SelectionManager已经初始化，但不强制重复初始化
    try 
    {
      sel_mgr->initialize();
    }
    catch (...)
    {
      // 初始化可能已经完成，忽略异常
    }
    
    // 重要：确保viewport和相机状态正确
    if (!event.viewport)
    {
      ROS_ERROR("Viewport为空");
      return false;
    }
    
    Ogre::Camera* camera = event.viewport->getCamera();
    if (!camera)
    {
      ROS_ERROR("相机为空");
      return false;
    }
    
    // 检查鼠标坐标是否在viewport范围内
    int vp_width = event.viewport->getActualWidth();
    int vp_height = event.viewport->getActualHeight();
    
    if (event.x < 0 || event.x >= vp_width || event.y < 0 || event.y >= vp_height)
    {
      ROS_WARN("鼠标坐标超出viewport范围: (%d, %d) in %dx%d", 
               event.x, event.y, vp_width, vp_height);
      return false;
    }
    
    ROS_INFO("调用SelectionManager::get3DPoint坐标(%d, %d) 在viewport %dx%d", 
              event.x, event.y, vp_width, vp_height);
    
    // 重要：确保SelectionManager有最新的渲染数据，但要安全地调用
    try 
    {
      context_->queueRender();
    }
    catch (...)
    {
      ROS_WARN("queueRender调用失败");
    }
    
    // 关键：在try-catch保护下调用SelectionManager
    bool success = false;
    try 
    {
      ROS_INFO("正在调用 sel_mgr->get3DPoint...");
      // 这是SelectionManager的正确调用方式
      success = sel_mgr->get3DPoint(event.viewport, event.x, event.y, result_pos);
      ROS_INFO("sel_mgr->get3DPoint 调用完成，返回值: %s", success ? "true" : "false");
    }
    catch (const Ogre::Exception& e)
    {
      ROS_ERROR("SelectionManager调用时Ogre异常: %s", e.what());
      return false;
    }
    catch (const std::exception& e) 
    {
      ROS_ERROR("SelectionManager调用时标准异常: %s", e.what());
      return false;
    }
    catch (...)
    {
      ROS_ERROR("SelectionManager调用时未知异常");
      return false;
    }
    
    if (success)
    {
      // 验证返回的坐标是否有效
      if (std::isfinite(result_pos.x) && std::isfinite(result_pos.y) && std::isfinite(result_pos.z))
      {
        ROS_INFO("SelectionManager获取原始3D点: (%.3f, %.3f, %.3f)", 
                 result_pos.x, result_pos.y, result_pos.z);
        
        // 尝试使用3DPatch进行高度精度改善
        Ogre::Vector3 refined_pos;
        if (refineHeightUsing3DPatch(event, result_pos, refined_pos))
        {
          result_pos = refined_pos;
          ROS_INFO("高度精度改善后的3D点: (%.3f, %.3f, %.3f)", 
                   result_pos.x, result_pos.y, result_pos.z);
        }
        
        return true;
      }
      else
      {
        ROS_WARN("SelectionManager返回无效坐标，包含NaN或Inf");
        return false;
      }
    }
    else
    {
      ROS_WARN("SelectionManager未找到3D点（可能点击了空白区域或无几何体）");
      
      // 轻量级状态检查，不进行破坏性清理
      try 
      {
        // 仅在安全的情况下触发一次渲染
        if (context_)
        {
          context_->queueRender();
        }
      }
      catch (...)
      {
        // 忽略渲染异常
      }
      ROS_INFO("SelectionManager返回false，请尝试点击有几何体的区域");
      
      return false;
    }
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("get3DPointFromSelection整体异常: %s", e.what());
    return false;
  }
  
  // 如果SelectionManager失败，返回false表示无法获取3D点
  ROS_WARN("SelectionManager无法获取3D点");
  
  // 轻量级错误恢复，不破坏SelectionManager状态
  try 
  {
    if (context_)
    {
      context_->queueRender();
    }
  }
  catch (...)
  {
    // 静默处理渲染异常
  }
  
  return false;
}

bool Pose3DTool::refineHeightUsing3DPatch(ViewportMouseEvent& event, const Ogre::Vector3& original_pos, Ogre::Vector3& refined_pos)
{
  try 
  {
    SelectionManager* sel_mgr = context_->getSelectionManager();
    if (!sel_mgr)
    {
      return false;
    }
    
    // 使用3x3的patch来获取更精确的高度信息
    const int patch_size = 3;
    const int half_patch = patch_size / 2;
    
    std::vector<Ogre::Vector3> patch_points;
    bool patch_success = sel_mgr->get3DPatch(
      event.viewport,
      event.x - half_patch,  // 中心周围的小区域
      event.y - half_patch,
      patch_size,
      patch_size,
      true,  // skip_missing = true
      patch_points
    );
    
    if (patch_success && !patch_points.empty())
    {
      // 计算有效点的平均高度
      float valid_z_sum = 0.0f;
      int valid_count = 0;
      
      for (const auto& point : patch_points)
      {
        if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z))
        {
          valid_z_sum += point.z;
          valid_count++;
        }
      }
      
      if (valid_count > 0)
      {
        refined_pos = original_pos;
        refined_pos.z = valid_z_sum / valid_count;  // 使用平均高度
        
        // 检查高度差异是否合理
        float height_diff = std::abs(refined_pos.z - original_pos.z);
        if (height_diff > 0.001f)  // 超过1mm的差异才记录
        {
          ROS_DEBUG("高度精度改善: 原始=%.4f, 改善后=%.4f, 差异=%.4f", 
                   original_pos.z, refined_pos.z, height_diff);
        }
        
        return true;
      }
    }
    
    // Patch方法失败，尝试使用射线投影作为高度验证
    return refineHeightUsingRaycast(event, original_pos, refined_pos);
  }
  catch (const std::exception& e)
  {
    ROS_DEBUG("3DPatch高度改善失败: %s", e.what());
    return false;
  }
}

bool Pose3DTool::refineHeightUsingRaycast(ViewportMouseEvent& event, const Ogre::Vector3& original_pos, Ogre::Vector3& refined_pos)
{
  try 
  {
    Ogre::Camera* camera = event.viewport->getCamera();
    if (!camera)
    {
      return false;
    }
    
    // 将屏幕坐标转换为标准化坐标
    float norm_x = ((float)event.x / (float)event.viewport->getActualWidth()) * 2.0f - 1.0f;
    float norm_y = 1.0f - ((float)event.y / (float)event.viewport->getActualHeight()) * 2.0f;
    
    // 创建射线
    Ogre::Ray ray = camera->getCameraToViewportRay(norm_x, norm_y);
    
    // 使用原始位置的XY坐标，但用射线投影验证高度
    // 创建一个水平面通过原始点
    Ogre::Plane horizontal_plane(Ogre::Vector3::UNIT_Z, -original_pos.z);
    
    std::pair<bool, Ogre::Real> intersection = ray.intersects(horizontal_plane);
    if (intersection.first)
    {
      Ogre::Vector3 ray_point = ray.getPoint(intersection.second);
      
      // 检查射线投影点与原始点的水平距离
      float horizontal_dist = std::sqrt(
        (ray_point.x - original_pos.x) * (ray_point.x - original_pos.x) +
        (ray_point.y - original_pos.y) * (ray_point.y - original_pos.y)
      );
      
      // 如果水平距离合理（小于10cm），使用射线投影的高度
      if (horizontal_dist < 0.1f)
      {
        refined_pos = original_pos;
        refined_pos.x = ray_point.x;  // 使用射线投影的精确XY
        refined_pos.y = ray_point.y;
        
        ROS_DEBUG("射线投影高度验证: 水平距离=%.4f", horizontal_dist);
        return true;
      }
    }
    
    return false;
  }
  catch (const std::exception& e)
  {
    ROS_DEBUG("射线投影高度验证失败: %s", e.what());
    return false;
  }
}

// ==================== 方向更新 ====================

void Pose3DTool::updateOrientationFromMouse(ViewportMouseEvent& event)
{
  ROS_INFO("=== updateOrientationFromMouse 被调用 ===");
  
  try
  {
    // 学习更优雅的3D平面投影方法
    // 创建一个与目标点同高度的水平平面
    Ogre::Plane ground_plane(Ogre::Vector3::UNIT_Z, selected_position_.z);
    Ogre::Vector3 cur_pos;
    
    ROS_INFO("目标点位置: (%.3f, %.3f, %.3f)", 
             selected_position_.x, selected_position_.y, selected_position_.z);
    
    // 将当前鼠标位置投影到平面上
    if (getPointOnPlaneFromWindowXY(event.viewport, ground_plane, event.x, event.y, cur_pos))
    {
      ROS_INFO("鼠标投影到平面的位置: (%.3f, %.3f, %.3f)", cur_pos.x, cur_pos.y, cur_pos.z);
      
      // 计算从目标点到当前投影点的角度（3D世界坐标）
      double dx_world = cur_pos.x - selected_position_.x;
      double dy_world = cur_pos.y - selected_position_.y;
      
      // 检查移动距离是否足够
      double distance_world = sqrt(dx_world * dx_world + dy_world * dy_world);
      if (distance_world < 0.05)  // 世界坐标下5cm的移动
      {
        ROS_INFO("3D世界移动距离太小(%.3fm)，保持当前角度", distance_world);
        return;
      }
      
      // 分别计算显示角度和目标角度
      current_angle_ = atan2(-dy_world, -dx_world);  // 箭头显示用（反向）
      target_angle_ = atan2(dy_world, dx_world);     // 最终目标用（原始）
      
      ROS_INFO("✅ 3D世界角度: %.1f° (%.3f弧度), 3D移动距离: %.3fm", 
               current_angle_ * 180.0 / M_PI, current_angle_, distance_world);
      ROS_INFO("   3D方向: dx=%.3f, dy=%.3f", dx_world, dy_world);
    }
    else
    {
      ROS_WARN("无法将鼠标位置投影到平面，坐标(%d, %d)", event.x, event.y);
    }
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("方向更新异常: %s", e.what());
  }
  
  ROS_INFO("=== updateOrientationFromMouse 完成 ===");
}

// ==================== 3D平面投影实现 ====================

bool Pose3DTool::getPointOnPlaneFromWindowXY(Ogre::Viewport* viewport, const Ogre::Plane& plane, int x, int y, Ogre::Vector3& result_point)
{
  if (!viewport)
  {
    ROS_ERROR("Viewport为空");
    return false;
  }
  
  Ogre::Camera* camera = viewport->getCamera();
  if (!camera)
  {
    ROS_ERROR("相机为空");
    return false;
  }
  
  try
  {
    // 将屏幕坐标转换为归一化设备坐标
    float screen_x = static_cast<float>(x) / static_cast<float>(viewport->getActualWidth());
    float screen_y = static_cast<float>(y) / static_cast<float>(viewport->getActualHeight());
    
    // 创建射线从相机到屏幕点
    Ogre::Ray ray = camera->getCameraToViewportRay(screen_x, screen_y);
    
    // 计算射线与平面的交点
    std::pair<bool, Ogre::Real> intersection = ray.intersects(plane);
    
    if (intersection.first)
    {
      // 有交点，计算交点位置
      result_point = ray.getPoint(intersection.second);
      ROS_INFO("射线与平面相交，交点: (%.3f, %.3f, %.3f)", 
               result_point.x, result_point.y, result_point.z);
      return true;
    }
    else
    {
      ROS_WARN("射线与平面不相交");
      return false;
    }
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("平面投影计算异常: %s", e.what());
    return false;
  }
}

// ==================== 箭头管理 ====================

void Pose3DTool::initializeMainArrow()
{
  ROS_INFO("=== 开始初始化主箭头 ===");
  
  try
  {
    if (!context_)
    {
      ROS_ERROR("context为空，无法初始化箭头");
      return;
    }
    
    if (!context_->getSceneManager())
    {
      ROS_ERROR("SceneManager为空，无法初始化箭头");
      return;
    }
    
    ROS_INFO("创建Arrow对象...");
    main_arrow_ = new Arrow(context_->getSceneManager(), nullptr);
    
    if (!main_arrow_)
    {
      ROS_ERROR("Arrow对象创建失败");
      return;
    }
    
    ROS_INFO("设置箭头属性...");
    main_arrow_->setScale(Ogre::Vector3(1.0f, 1.0f, 1.0f));
    
    // 安全检查SceneNode
    if (main_arrow_->getSceneNode())
    {
      main_arrow_->getSceneNode()->setVisible(false);
      ROS_INFO("箭头SceneNode设置成功");
    }
    else
    {
      ROS_ERROR("箭头SceneNode为空");
    }
    
    setArrowColor(1.0f, 0.0f, 1.0f, 1.0f); // 洋红色
    ROS_INFO("✅ 主箭头初始化成功");
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("❌ 箭头初始化异常: %s", e.what());
    main_arrow_ = nullptr;
  }
}

void Pose3DTool::safeDeleteArrow()
{
  if (main_arrow_)
  {
    try
    {
      delete main_arrow_;
      main_arrow_ = nullptr;
      ROS_DEBUG("箭头删除成功");
    }
    catch (const std::exception& e)
    {
      ROS_ERROR("箭头删除异常: %s", e.what());
      main_arrow_ = nullptr;
    }
  }
}

void Pose3DTool::setArrowColor(float r, float g, float b, float a)
{
  if (main_arrow_)
  {
    try
    {
      main_arrow_->setColor(r, g, b, a);
      ROS_DEBUG("箭头颜色设置: (%.1f, %.1f, %.1f, %.1f)", r, g, b, a);
    }
    catch (const std::exception& e)
    {
      ROS_DEBUG("箭头颜色设置异常: %s", e.what());
    }
  }
}

bool Pose3DTool::safeShowArrow()
{
  if (main_arrow_ && main_arrow_->getSceneNode())
  {
    main_arrow_->getSceneNode()->setVisible(true);
    return true;
  }
  return false;
}

void Pose3DTool::safeHideArrow()
{
  if (main_arrow_ && main_arrow_->getSceneNode())
  {
    main_arrow_->getSceneNode()->setVisible(false);
  }
}

// ==================== 工具状态管理 ====================

void Pose3DTool::resetTool()
{
  current_state_ = Position;
  current_angle_ = 0.0;
  target_angle_ = 0.0;
  selected_position_ = Ogre::Vector3::ZERO;
}

void Pose3DTool::cancelOperation(bool deactivate_tool)
{
  ROS_INFO("=== 取消操作：重置工具状态 ===");
  
  // 隐藏所有箭头
  safeHideArrow();
  
  // 重置工具状态
  resetTool();
  
  if (deactivate_tool && context_)
  {
    // 切换到Interact工具（类似ESC行为）
    ROS_INFO("右键取消：切换到Interact工具");
    Tool* interact_tool = findInteractTool();
    if (interact_tool)
    {
      context_->getToolManager()->setCurrentTool(interact_tool);
      ROS_INFO("已切换到Interact工具");
    }
    else
    {
      // 如果找不到Interact工具，则取消当前工具
      context_->getToolManager()->setCurrentTool(nullptr);
      ROS_INFO("未找到Interact工具，工具已取消激活");
    }
  }
  else
  {
    // 只重置状态，保持工具激活
    setStatus("按住左键拖拽设置目标位置和方向，右键退出工具");
    ROS_INFO("操作已取消，工具重置为初始状态");
  }
}

Tool* Pose3DTool::findInteractTool()
{
  if (!context_ || !context_->getToolManager())
  {
    ROS_WARN("DisplayContext或ToolManager为空");
    return nullptr;
  }
  
  ToolManager* tool_manager = context_->getToolManager();
  
  // 尝试几种常见的Interact工具名称
  std::vector<std::string> interact_names = {
    "rviz/Interact",
    "Interact", 
    "rviz/MoveCamera",
    "MoveCamera",
    "rviz/Select",
    "Select"
  };
  
  for (const std::string& name : interact_names)
  {
    for (int i = 0; i < tool_manager->numTools(); ++i)
    {
      Tool* tool = tool_manager->getTool(i);
      if (tool && tool->getClassId().toStdString() == name)
      {
        ROS_INFO("找到Interact工具: %s", name.c_str());
        return tool;
      }
    }
  }
  
  // 如果没找到特定的工具，返回第一个工具（通常是默认工具）
  if (tool_manager->numTools() > 0)
  {
    Tool* first_tool = tool_manager->getTool(0);
    if (first_tool)
    {
      ROS_INFO("使用第一个工具作为默认: %s", first_tool->getClassId().toStdString().c_str());
      return first_tool;
    }
  }
  
  ROS_WARN("未找到任何可用的工具");
  return nullptr;
}

// ==================== 空实现方法 (保持兼容性) ====================

int Pose3DTool::processHeightState(ViewportMouseEvent& event) { return 0; }
void Pose3DTool::updateHeightIndicators() {}
void Pose3DTool::createHeightArrow(const Ogre::Vector3& position, double angle) {}
void Pose3DTool::clearHeightIndicators() {}
void Pose3DTool::updateOrientationArrow()
{
  if (!main_arrow_ || !main_arrow_->getSceneNode())
  {
    return;
  }
  
  try 
  {
    // 设置箭头位置
    main_arrow_->setPosition(selected_position_);
    
    // 修正箭头方向：RViz Arrow默认向下(-Z轴)，需要旋转到水平面
    // 第一步：将默认的向下方向旋转到X轴正方向（向前）
    Ogre::Quaternion orient_x;
    orient_x.FromAngleAxis(Ogre::Degree(90), Ogre::Vector3::UNIT_Y);  // 绕Y轴旋转90度，从-Z到+X
    
    // 第二步：应用我们计算的角度（绕Z轴旋转）
    Ogre::Quaternion rotation;
    rotation.FromAngleAxis(Ogre::Radian(current_angle_), Ogre::Vector3::UNIT_Z);
    
         // 第三步：组合旋转（如果方向还是反的，可以尝试 orient_x * rotation）
     Ogre::Quaternion final_orientation = rotation * orient_x;
     main_arrow_->setOrientation(final_orientation);
     
     ROS_INFO("箭头方向设置: 基准旋转90°(Y轴) + 角度%.1f°(Z轴)", current_angle_ * 180.0 / M_PI);
    
    // 确保箭头可见
    main_arrow_->getSceneNode()->setVisible(true);
    
    ROS_INFO("箭头更新: 位置(%.3f, %.3f, %.3f), 角度%.1f°", 
              selected_position_.x, selected_position_.y, selected_position_.z,
              current_angle_ * 180.0 / M_PI);
  }
  catch (const std::exception& e)
  {
    ROS_WARN("箭头更新异常: %s", e.what());
  }
}

void Pose3DTool::createOrientationArrow(const Ogre::Vector3& position)
{
  // 确保箭头已初始化
  if (!main_arrow_)
  {
    initializeMainArrow();
  }
  
  if (main_arrow_)
  {
    // 设置箭头初始位置和方向
    main_arrow_->setPosition(position);
    
    // 初始方向指向X轴正方向（0度）
    current_angle_ = 0.0;
    
    // 使用与updateOrientationArrow相同的方向设置逻辑
    Ogre::Quaternion orient_x;
    orient_x.FromAngleAxis(Ogre::Degree(90), Ogre::Vector3::UNIT_Y);  // 从-Z到+X
    Ogre::Quaternion rotation;
    rotation.FromAngleAxis(Ogre::Radian(current_angle_), Ogre::Vector3::UNIT_Z);
    Ogre::Quaternion initial_orientation = rotation * orient_x;
    main_arrow_->setOrientation(initial_orientation);
    
    // 显示箭头
    safeShowArrow();
    
    ROS_INFO("方向箭头创建完成，位置: (%.3f, %.3f, %.3f)", 
             position.x, position.y, position.z);
    
    setStatus("拖拽鼠标设置目标方向，松开左键确认，右键退出工具");
  }
}

void Pose3DTool::safeUpdateArrowOrientation() 
{
  updateOrientationArrow();
}

 } // namespace rviz