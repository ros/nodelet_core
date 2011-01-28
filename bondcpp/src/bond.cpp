/*
 * Copyright (c) 2009, Willow Garage, Inc.
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

// Author: Stuart Glaser

#include <bondcpp/bond.h>
#include <boost/thread/thread_time.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <uuid/uuid.h>

namespace bond {

static std::string makeUUID()
{
  uuid_t uuid;
  uuid_generate_random(uuid);
  char uuid_str[40];
  uuid_unparse(uuid, uuid_str);
  return std::string(uuid_str);
}

Bond::Bond(const std::string &topic, const std::string &id,
           boost::function<void(void)> on_broken,
           boost::function<void(void)> on_formed)
  :

  bondsm_(new BondSM(this)),
  sm_(*bondsm_),
  topic_(topic),
  id_(id),
  instance_id_(makeUUID()),
  on_broken_(on_broken),
  on_formed_(on_formed),
  sisterDiedFirst_(false),
  started_(false),

  connect_timer_(ros::WallDuration(), boost::bind(&Bond::onConnectTimeout, this)),
  heartbeat_timer_(ros::WallDuration(), boost::bind(&Bond::onHeartbeatTimeout, this)),
  disconnect_timer_(ros::WallDuration(), boost::bind(&Bond::onDisconnectTimeout, this))
{
  setConnectTimeout(bond::Constants::DEFAULT_CONNECT_TIMEOUT);
  setDisconnectTimeout(bond::Constants::DEFAULT_DISCONNECT_TIMEOUT);
  setHeartbeatTimeout(bond::Constants::DEFAULT_HEARTBEAT_TIMEOUT);
  setHeartbeatPeriod(bond::Constants::DEFAULT_HEARTBEAT_PERIOD);
  pub_ = nh_.advertise<bond::Status>(topic_, 5);
}

Bond::~Bond()
{
  breakBond();
  if (!waitUntilBroken(ros::Duration(1.0)))
  {
    ROS_DEBUG("Bond failed to break on destruction %s (%s)", id_.c_str(), instance_id_.c_str());
  }

  // Must destroy the subscription before locking mutex_: shutdown()
  // will block until the status callback completes, and the status
  // callback locks mutex_ (in flushPendingCallbacks).
  sub_.shutdown();

  boost::mutex::scoped_lock lock(mutex_);
  connect_timer_.cancel();
  heartbeat_timer_.cancel();
  disconnect_timer_.cancel();
  publishingTimer_.stop();
  pub_.shutdown();
}


void Bond::setConnectTimeout(double dur)
{
  if (started_) {
    ROS_ERROR("Cannot set timeouts after calling start()");
    return;
  }

  connect_timeout_ = dur;
  connect_timer_.setDuration(ros::WallDuration(connect_timeout_));
}

void Bond::setDisconnectTimeout(double dur)
{
  if (started_) {
    ROS_ERROR("Cannot set timeouts after calling start()");
    return;
  }

  disconnect_timeout_ = dur;
  disconnect_timer_.setDuration(ros::WallDuration(disconnect_timeout_));
}

void Bond::setHeartbeatTimeout(double dur)
{
  if (started_) {
    ROS_ERROR("Cannot set timeouts after calling start()");
    return;
  }

  heartbeat_timeout_ = dur;
  heartbeat_timer_.setDuration(ros::WallDuration(heartbeat_timeout_));
}

void Bond::setHeartbeatPeriod(double dur)
{
  if (started_) {
    ROS_ERROR("Cannot set timeouts after calling start()");
    return;
  }

  heartbeat_period_ = dur;
}


void Bond::start()
{
  boost::mutex::scoped_lock lock(mutex_);
  connect_timer_.reset();
  sub_ = nh_.subscribe<bond::Status>(topic_, 30, boost::bind(&Bond::bondStatusCB, this, _1));

  publishingTimer_ = nh_.createWallTimer(ros::WallDuration(heartbeat_period_), &Bond::doPublishing, this);
  started_ = true;
}

void Bond::setFormedCallback(boost::function<void(void)> on_formed)
{
  boost::mutex::scoped_lock lock(mutex_);
  on_formed_ = on_formed;
}

void Bond::setBrokenCallback(boost::function<void(void)> on_broken)
{
  boost::mutex::scoped_lock lock(mutex_);
  on_broken_ = on_broken;
}

bool Bond::waitUntilFormed(ros::Duration d)
{
  boost::mutex::scoped_lock lock(mutex_);
  boost::system_time deadline(boost::get_system_time() +
                              boost::posix_time::microseconds(d.toNSec()/1000L));
  while (sm_.getState().getId() == SM::WaitingForSister.getId())
  {
    if (d < ros::Duration(0))
      condition_.wait(mutex_);
    else if (!condition_.timed_wait(mutex_, deadline))
      break;
  }
  return sm_.getState().getId() != SM::WaitingForSister.getId();
}

bool Bond::waitUntilBroken(ros::Duration d)
{
  boost::mutex::scoped_lock lock(mutex_);
  boost::system_time deadline(boost::get_system_time() +
                              boost::posix_time::microseconds(d.toNSec()/1000L));
  while (sm_.getState().getId() != SM::Dead.getId())
  {
    if (d < ros::Duration(0))
      condition_.wait(mutex_);
    else if (!condition_.timed_wait(mutex_, deadline))
      break;
  }
  return sm_.getState().getId() == SM::Dead.getId();
}

bool Bond::isBroken()
{
  boost::mutex::scoped_lock lock(mutex_);
  return sm_.getState().getId() == SM::Dead.getId();
}

void Bond::breakBond()
{
  {
    boost::mutex::scoped_lock lock(mutex_);
    if (sm_.getState().getId() != SM::Dead.getId())
    {
      sm_.Die();
      publishStatus(false);
    }
  }
  flushPendingCallbacks();
}


void Bond::onConnectTimeout()
{
  {
    boost::mutex::scoped_lock lock(mutex_);
    sm_.ConnectTimeout();
  }
  flushPendingCallbacks();
}
void Bond::onHeartbeatTimeout()
{
  {
    boost::mutex::scoped_lock lock(mutex_);
    sm_.HeartbeatTimeout();
  }
  flushPendingCallbacks();
}
void Bond::onDisconnectTimeout()
{
  {
    boost::mutex::scoped_lock lock(mutex_);
    sm_.DisconnectTimeout();
  }
  flushPendingCallbacks();
}

void Bond::bondStatusCB(const bond::Status::ConstPtr &msg)
{
  // Filters out messages from other bonds and messages from ourself
  if (msg->id == id_ && msg->instance_id != instance_id_)
  {
    {
      boost::mutex::scoped_lock lock(mutex_);

      if (sister_instance_id_.empty())
        sister_instance_id_ = msg->instance_id;
      if (sister_instance_id_ != msg->instance_id) {
        ROS_ERROR("More than two locations are trying to use a single bond (topic: %s, id: %s).  "
                  "You should only instantiate at most two bond instances for each (topic, id) pair.",
                  topic_.c_str(), id_.c_str());
        return;
      }

      if (msg->active)
      {
        sm_.SisterAlive();
      }
      else
      {
        sm_.SisterDead();

        // Immediate ack for sister's death notification
        if (sisterDiedFirst_)
          publishStatus(false);
      }
    }
    flushPendingCallbacks();
  }
}

void Bond::doPublishing(const ros::WallTimerEvent &e)
{
  boost::mutex::scoped_lock lock(mutex_);
  if (sm_.getState().getId() == SM::WaitingForSister.getId() ||
      sm_.getState().getId() == SM::Alive.getId())
  {
    publishStatus(true);
  }
  else if (sm_.getState().getId() == SM::AwaitSisterDeath.getId())
  {
    publishStatus(false);
  }
  else
  {
    publishingTimer_.stop();
  }

}

void Bond::publishStatus(bool active)
{
  bond::Status::Ptr msg(new bond::Status);
  msg->header.stamp = ros::Time::now();
  msg->id = id_;
  msg->instance_id = instance_id_;
  msg->active = active;
  msg->heartbeat_timeout = heartbeat_timeout_;
  msg->heartbeat_period = heartbeat_period_;
  pub_.publish(msg);
}


void Bond::flushPendingCallbacks()
{
  std::vector<boost::function<void(void)> > callbacks;
  {
    boost::mutex::scoped_lock lock(mutex_);
    callbacks = pending_callbacks_;
    pending_callbacks_.clear();
  }

  for (size_t i = 0; i < callbacks.size(); ++i)
    callbacks[i]();
}

} // namespace


void BondSM::Connected()
{
  b->connect_timer_.cancel();
  b->condition_.notify_all();
  if (b->on_formed_)
    b->pending_callbacks_.push_back(b->on_formed_);
}

void BondSM::SisterDied()
{
  b->sisterDiedFirst_ = true;
}

void BondSM::Death()
{
  b->condition_.notify_all();
  b->heartbeat_timer_.cancel();
  b->disconnect_timer_.cancel();
  if (b->on_broken_)
    b->pending_callbacks_.push_back(b->on_broken_);
}

void BondSM::Heartbeat()
{
  b->heartbeat_timer_.reset();
}

void BondSM::StartDying()
{
  b->heartbeat_timer_.cancel();
  b->disconnect_timer_.reset();
  b->publishingTimer_.setPeriod(ros::WallDuration(bond::Constants::DEAD_PUBLISH_PERIOD));
}