/*
 * Copyright (c) 2023, Open Source Robotics Foundation
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

#include <bond/Status.h>
#include <pluginlib/class_list_macros.hpp>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

namespace test_nodelet
{

class RequestStop : public nodelet::Nodelet
{
private:
  void onInit() override
  {
    ros::NodeHandle pnh = getPrivateNodeHandle();
    std::string managerName {};
    managerName = pnh.param("manager_name", managerName);
    waitForBond_ = pnh.param("wait_for_bond", waitForBond_);
    
    ros::NodeHandle nh = getNodeHandle();
    pub_ = nh.advertise<std_msgs::String>("ready", 1, true);
    timer_ = nh.createSteadyTimer(ros::WallDuration(0.1), &RequestStop::callback, this, false);
    if (waitForBond_)
      sub_ = nh.subscribe(ros::names::append(managerName, "bond"), 4, &RequestStop::bond, this);
  }
  
  void bond(const bond::Status&)
  {
    numBondMsgs_ += 1;
  }

  void callback(const ros::SteadyTimerEvent&)
  {
    // If running non-standalone, wait for the bonds to establish; otherwise, killing
    // this nodelet too early would mean waiting 10 seconds for bond connect timeout, instead
    // of waiting just 4 seconds for bond heartbeat timeout. Each end of the bond sends
    // one message per second, so we wait for 2 messages to be sent from each end.

    if (waitForBond_ && numBondMsgs_ < 4)
      return;

    timer_.stop();

    std_msgs::String ready;
    ready.data = this->getPrivateNodeHandle().getNamespace();
    NODELET_DEBUG("Ready");
    pub_.publish(ready);

    // Run an infinite loop which can only be ended by an async call to requestShutdown().
    while (ros::ok() && this->ok())
      ros::WallDuration(0.1).sleep();

    ready.data = "interrupted";
    NODELET_INFO("Interrupted");
    std::cout << "Interrupted" << std::endl;
    pub_.publish(ready);
  }

  bool waitForBond_ {false};
  size_t numBondMsgs_ {0u};
  ros::Publisher pub_;
  ros::Subscriber sub_;
  ros::SteadyTimer timer_;
};

PLUGINLIB_EXPORT_CLASS(test_nodelet::RequestStop, nodelet::Nodelet)
}
