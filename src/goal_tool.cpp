/*
 * 简化版本Goal3DTool - 配合简化的Pose3DTool基类
 */

#include "goal_tool.h"

#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>

#include <rviz/display_context.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/tool_manager.h>

namespace rviz
{

Goal3DTool::Goal3DTool()
{
  ROS_INFO("Goal3DTool: 简化版本构造");
  shortcut_key_ = 'h'; // 使用'h'避免冲突
  
  initializeProperties();
}

void Goal3DTool::onInitialize()
{
  ROS_ERROR("=== Goal3DTool::onInitialize() 简化版本 ===");
  
  // 调用基类初始化
  Pose3DTool::onInitialize();
  
  // 设置工具名称
  setName("3D Nav Goal");
  
  // 设置箭头颜色为洋红色以区别于其他工具
  setArrowColor(1.0f, 0.0f, 1.0f, 1.0f); // 洋红色
  
  // 初始化发布器
  updateTopic();
  
  ROS_ERROR("Goal3DTool 初始化完成");
}

void Goal3DTool::onPoseSet(double x, double y, double z, double theta)
{
  ROS_INFO("=== Goal3DTool::onPoseSet() ===");
  ROS_INFO("设置3D目标:");
  ROS_INFO("  位置: (%.3f, %.3f, %.3f)", x, y, z);
  ROS_INFO("  方向: %.1f° (%.3f弧度)", theta * 180.0 / M_PI, theta);
  
  if (!validatePublisher())
  {
    ROS_ERROR("发布器无效，无法发布3D目标");
    return;
  }
  
  try
  {
    // 创建并发布消息
    geometry_msgs::PoseStamped goal = createPoseStampedMessage(x, y, z, theta);
    pub_.publish(goal);
    
    // 记录详细信息
    logGoalInfo(goal, theta);
    
    // 检查订阅者
    if (pub_.getNumSubscribers() > 0)
    {
      ROS_INFO("✅ 3D目标发布成功，订阅者数量: %d", pub_.getNumSubscribers());
    }
    else
    {
      ROS_WARN("⚠️  话题'%s'当前无订阅者", topic_property_->getStdString().c_str());
    }
    
    // 目标设置完成，保持工具激活状态，可继续设置下一个目标
    ROS_INFO("目标设置完成，工具保持激活状态，可继续设置下一个目标");
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("❌ 发布3D目标时异常: %s", e.what());
  }
}

// ==================== 私有槽函数 ====================

void Goal3DTool::updateTopic()
{
  try
  {
    std::string topic_name = topic_property_->getStdString();
    pub_ = nh_.advertise<geometry_msgs::PoseStamped>(topic_name, 1);
    ROS_INFO("Goal3DTool: 开始发布到话题 '%s'", topic_name.c_str());
  }
  catch (const ros::Exception& e)
  {
    ROS_ERROR_STREAM_NAMED("Goal3DTool", "话题设置失败: " << e.what());
  }
}


// ==================== 私有辅助方法 ====================

void Goal3DTool::initializeProperties()
{
  // 创建话题属性
  topic_property_ = new StringProperty(
    "Topic", 
    "goal_3d",
    "发布3D导航目标的话题名称",
    getPropertyContainer(), 
    SLOT(updateTopic()), 
    this
  );
}

bool Goal3DTool::validatePublisher() const
{
  if (!pub_)
  {
    ROS_ERROR("发布器未初始化");
    return false;
  }
  
  if (topic_property_->getStdString().empty())
  {
    ROS_ERROR("话题名称为空");
    return false;
  }
  
  return true;
}

geometry_msgs::PoseStamped Goal3DTool::createPoseStampedMessage(double x, double y, double z, double theta) const
{
  if (!context_)
  {
    throw std::runtime_error("Display context为空");
  }
  
  geometry_msgs::PoseStamped goal;
  
  // 设置头信息
  goal.header.frame_id = context_->getFixedFrame().toStdString();
  goal.header.stamp = ros::Time::now();
  
  // 设置位置（直接使用3D点的完整坐标）
  goal.pose.position.x = x;
  goal.pose.position.y = y;
  goal.pose.position.z = z; // 真实的3D高度
  
  // 设置方向（绕Z轴旋转）
  tf2::Quaternion quat;
  quat.setRPY(0.0, 0.0, theta);
  goal.pose.orientation = tf2::toMsg(quat);
  
  return goal;
}

void Goal3DTool::logGoalInfo(const geometry_msgs::PoseStamped& goal, double theta) const
{
  ROS_INFO("📄 3D目标详细信息:");
  ROS_INFO("  坐标系: %s", goal.header.frame_id.c_str());
  ROS_INFO("  3D位置: (%.3f, %.3f, %.3f)", 
           goal.pose.position.x, 
           goal.pose.position.y, 
           goal.pose.position.z); // 显示真实高度
  ROS_INFO("  四元数: (%.3f, %.3f, %.3f, %.3f)", 
           goal.pose.orientation.x,
           goal.pose.orientation.y, 
           goal.pose.orientation.z,
           goal.pose.orientation.w);
  ROS_INFO("  角度: %.1f° (%.3f弧度)", theta * 180.0 / M_PI, theta);
}

} // namespace rviz

// 导出插件
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz::Goal3DTool, rviz::Tool)