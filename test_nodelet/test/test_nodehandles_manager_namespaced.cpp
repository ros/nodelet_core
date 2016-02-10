/*********************************************************************
* test_nodehandles_same_namespaces.cpp
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2016, University of Patras
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of University of Patras nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Authors: Aris Synodinos
*********************************************************************/

#include <gtest/gtest.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Byte.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <ros/ros.h>

TEST(SameNamespaces, NodeletPrivateNodehandle) {
  ros::NodeHandle nh;
  ros::Duration(2).sleep();
  ros::master::V_TopicInfo master_topics;
  ros::master::getTopics(master_topics);
  bool found_topic = false;

  for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++) {
    const ros::master::TopicInfo& info = *it;
    if (info.datatype.compare("std_msgs/Bool") == 0) {
      found_topic = true;
      EXPECT_STREQ("/nodehandle_test/private", info.name.c_str());
    }
  }
  EXPECT_TRUE(found_topic);
}

TEST(SameNamespaces, NodeletNamespacedNodehandle) {
  ros::NodeHandle nh;
  ros::Duration(2).sleep();
  ros::master::V_TopicInfo master_topics;
  ros::master::getTopics(master_topics);
  bool found_topic = false;

  for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++) {
    const ros::master::TopicInfo& info = *it;
    if (info.datatype.compare("std_msgs/Byte") == 0) {
      found_topic = true;
      EXPECT_STREQ("/namespaced", info.name.c_str());
    }
  }
  EXPECT_TRUE(found_topic);
}

TEST(SameNamespaces, NodeletGlobalNodehandle) {
  ros::NodeHandle nh;
  ros::Duration(2).sleep();
  ros::master::V_TopicInfo master_topics;
  ros::master::getTopics(master_topics);
  bool found_topic = false;

  for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++) {
    const ros::master::TopicInfo& info = *it;
    if (info.datatype.compare("std_msgs/Time") == 0) {
      found_topic = true;
      EXPECT_STREQ("/global", info.name.c_str());
    }
  }
  EXPECT_TRUE(found_topic);
}

TEST(SameNamespaces, NodePrivateNodehandle) {
  ros::NodeHandle nh("~");
  ros::Publisher pub = nh.advertise<std_msgs::Empty>("private", 10);
  ros::master::V_TopicInfo master_topics;
  ros::master::getTopics(master_topics);
  bool found_topic = false;

  for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++) {
    const ros::master::TopicInfo& info = *it;
    if (info.datatype.compare("std_msgs/Empty") == 0) {
      found_topic = true;
      EXPECT_STREQ("/test_nodehandles/private", info.name.c_str());
    }
  }
  EXPECT_TRUE(found_topic);
}

TEST(SameNamespaces, NodeNamespacedNodehandle) {
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<std_msgs::String>("namespaced", 10);
  ros::master::V_TopicInfo master_topics;
  ros::master::getTopics(master_topics);
  bool found_topic = false;

  for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++) {
    const ros::master::TopicInfo& info = *it;
    if (info.datatype.compare("std_msgs/String") == 0) {
      found_topic = true;
      EXPECT_STREQ("/namespaced", info.name.c_str());
    }
  }
  EXPECT_TRUE(found_topic);
}

TEST(SameNamespaces, NodeGlobalNodehandle) {
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<std_msgs::Float32>("/public", 10);
  ros::master::V_TopicInfo master_topics;
  ros::master::getTopics(master_topics);
  bool found_topic = false;

  for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++) {
    const ros::master::TopicInfo& info = *it;
    if (info.datatype.compare("std_msgs/Float32") == 0) {
      found_topic = true;
      EXPECT_STREQ("/public", info.name.c_str());
    }
  }
  EXPECT_TRUE(found_topic);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_nodehandles_manager_namespaced");
  return RUN_ALL_TESTS();
}
