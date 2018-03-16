/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, JSK Lab, University of Tokyo.
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
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
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
 *********************************************************************/

/* Author: Kentaro Wada <www.kentaro.wada@gmail.com>
 */

#include <nodelet_topic_tools/nodelet_lazy.h>
#include <std_msgs/String.h>

namespace test_nodelet_topic_tools
{

class NodeletLazyString: public nodelet_topic_tools::NodeletLazy
{
public:
protected:
  virtual void onInit()
  {
    nodelet_topic_tools::NodeletLazy::onInit();
    pub_ = advertise<std_msgs::String>(*pnh_, "output", 1);
    onInitPostProcess();
  }

  virtual void subscribe()
  {
    sub_ = pnh_->subscribe("input", 1, &NodeletLazyString::callback, this);
  }

  virtual void unsubscribe()
  {
    sub_.shutdown();
  }

  virtual void callback(const std_msgs::String::ConstPtr& msg)
  {
    pub_.publish(msg);
  }

  ros::Publisher pub_;
  ros::Subscriber sub_;

private:
};

}  // namespace test_nodelet_topic_tools

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(test_nodelet_topic_tools::NodeletLazyString, nodelet::Nodelet);
