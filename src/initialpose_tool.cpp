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

#include <tf/transform_listener.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "rviz/display_context.h"
#include "rviz/properties/string_property.h"

#include "initialpose_tool.h"

namespace rviz
{

    InitialPose3DTool::InitialPose3DTool()
    {
        shortcut_key_ = 'i'; // 使用'i'作为快捷键

        topic_property_ = new StringProperty("Topic", "initialpose",
                                             "The topic on which to publish initial pose.",
                                             getPropertyContainer(), SLOT(updateTopic()), this);
    }

    void InitialPose3DTool::onInitialize()
    {
        Pose3DTool::onInitialize();
        setName("3D Pose Estimate");
        updateTopic();
    }

    void InitialPose3DTool::updateTopic()
    {
        pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(topic_property_->getStdString(), 1);
    }

    void InitialPose3DTool::onPoseSet(double x, double y, double z, double theta)
    {
        std::string fixed_frame = context_->getFixedFrame().toStdString();

        // 创建四元数
        tf::Quaternion quat;
        quat.setRPY(0.0, 0.0, theta);

        // 创建带时间戳的位姿
        tf::Stamped<tf::Pose> p = tf::Stamped<tf::Pose>(tf::Pose(quat, tf::Point(x, y, z+0.5)),
                                                        ros::Time::now(), fixed_frame);

        // 转换为ROS消息类型
        geometry_msgs::PoseWithCovarianceStamped initialpose;
        geometry_msgs::PoseStamped pose_stamped;
        tf::poseStampedTFToMsg(p, pose_stamped);

        // 填充PoseWithCovarianceStamped消息
        initialpose.header = pose_stamped.header;
        initialpose.pose.pose = pose_stamped.pose;

        // 设置协方差矩阵（使用默认值）
        for (int i = 0; i < 36; i++)
        {
            initialpose.pose.covariance[i] = 0.0;
        }

        // 设置对角线元素为适当的值
        initialpose.pose.covariance[0] = 0.25;  // x
        initialpose.pose.covariance[7] = 0.25;  // y
        initialpose.pose.covariance[14] = 0.25; // z
        initialpose.pose.covariance[21] = 0.1;  // roll
        initialpose.pose.covariance[28] = 0.1;  // pitch
        initialpose.pose.covariance[35] = 0.1;  // yaw

        ROS_INFO("Setting initial pose: Frame:%s, Position(%.3f, %.3f, %.3f), Orientation(%.3f, %.3f, %.3f, %.3f) = Angle: %.3f\n",
                 fixed_frame.c_str(),
                 initialpose.pose.pose.position.x,
                 initialpose.pose.pose.position.y,
                 initialpose.pose.pose.position.z,
                 initialpose.pose.pose.orientation.x,
                 initialpose.pose.pose.orientation.y,
                 initialpose.pose.pose.orientation.z,
                 initialpose.pose.pose.orientation.w,
                 theta);

        pub_.publish(initialpose);
    }

} // end namespace rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz::InitialPose3DTool, rviz::Tool)