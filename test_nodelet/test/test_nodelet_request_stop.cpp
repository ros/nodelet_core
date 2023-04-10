/*
 * Copyright (c) 2022, Open Source Robotics Foundation
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

#include "gtest/gtest.h"

#include <thread>

#include <nodelet/loader.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>

/**
 * \brief A mock of the nodelet::Nodelet class with empty onInit().
 */
struct TestNodelet : public nodelet::Nodelet
{
  /**
   * \brief If wasOk is not null, set the value of this->ok() to it while the object is destroyed.
   */
  ~TestNodelet() override
  {
    if (this->wasOk != nullptr)
      *this->wasOk = this->ok();
  }

  /**
   * \brief Empty.
   */
  void onInit() override
  {
  }
  
  bool sleepCb(const ros::Duration& duration, volatile bool& startedExecuting)
  {
    volatile bool sleepRunning {true};
    volatile bool sleepOk {false};

    timer = this->getNodeHandle().createWallTimer(ros::WallDuration(0.001),
      [=,&sleepOk,&sleepRunning,&startedExecuting](const ros::WallTimerEvent&) {
        startedExecuting = true;
        sleepOk = this->sleep(duration);
	sleepRunning = false;
      }, true);

    // no access to this-> past this point; the nodelet might already be destroyed

    while (sleepRunning)
      ros::WallDuration(1e-3).sleep();

    return sleepOk;
  }

  /**
   * \brief Sleep for the given amount of time, checking this->ok() for possible interruptions.
   * \param[in] duration Duration of the sleep.
   * \return The value of this->ok() after the sleep.
   */
  bool sleep(const ros::Duration& duration)
  {
    const auto endTime = ros::Time::now() + duration;

    while (ros::Time::now() < endTime && this->ok())
      ros::WallDuration(1e-3).sleep();

    if (ros::Time::now() < endTime)
      NODELET_INFO_STREAM("Ended " << (endTime - ros::Time::now()) << " s before deadline");

    return this->ok();
  }

  //! \brief The value of this->ok() during destruction.
  volatile bool* wasOk {nullptr};
  
  ros::WallTimer timer;
};

/**
 * \brief Test `ok()` and `requestStop()` functions.
 */
TEST(StatefulNodelet, ok)  // NOLINT
{
  bool wasOk {true};
  {
    TestNodelet nodelet;
    nodelet.wasOk = &wasOk;
    nodelet.init("test", {}, {});
    EXPECT_TRUE(nodelet.ok());

    nodelet.requestStop();
    EXPECT_FALSE(nodelet.ok());
  }
  EXPECT_FALSE(wasOk);
}

/**
 * \brief Test interrupting `sleep()` function with system time.
 */
TEST(StatefulNodelet, sleepInterruptSystime)  // NOLINT
{
  ros::Time::init();  // use system time
  ASSERT_TRUE(ros::Time::useSystemTime());

  // Create a thread that sleeps for 1 second. After 0.1 s, check that it has started, then interrupt it, and after
  // another 0.1 s, check that it has finished (if not, `finished` should still be false after the 0.2 s of waiting).

  TestNodelet nodelet;
  nodelet.init("test", {}, {});  
  bool started = false;
  bool finished = false;
  std::thread t([&]()
    {
      started = true;
      EXPECT_FALSE(nodelet.sleep(ros::Duration(1)));
      finished = true;
    });
  
  for (size_t i = 0; i < 1000 && !started; ++i)
    ros::WallDuration(0.001).sleep();
  EXPECT_TRUE(started);
  EXPECT_FALSE(finished);
  
  nodelet.requestStop();
  ros::WallDuration(0.1).sleep();

  EXPECT_TRUE(finished);
  
  t.join();
}

/**
 * \brief Test interrupting `sleep()` function with simulation time.
 */
TEST(StatefulNodelet, sleepInterruptSimtime)  // NOLINT
{
  ros::Time::setNow({10, 0});  // use simulation time
  ASSERT_TRUE(ros::Time::isSimTime());

  // Create a thread that sleeps for 1 second. After 0.1 s, check that it has started, then interrupt it, and after
  // another 0.1 s, check that it has finished (if not, `finished` should still be false after the 0.2 s of waiting).

  TestNodelet nodelet;
  nodelet.init("test", {}, {});
  bool started = false;
  bool finished = false;
  std::thread t([&]()
    {
      started = true;
      EXPECT_FALSE(nodelet.sleep({1, 0}));
      finished = true;
    });
  
  for (size_t i = 0; i < 1000 && !started; ++i)
    ros::WallDuration(0.001).sleep();
  EXPECT_TRUE(started);
  EXPECT_FALSE(finished);
  
  ros::Time::setNow(ros::Time(10.1));
  
  nodelet.requestStop();
  ros::WallDuration(0.1).sleep();

  EXPECT_TRUE(finished);
  
  t.detach();  // detach the thread so that it doesn't block if the sleep did not end
}

/**
 * \brief Test finishing `sleep()` function with system time.
 */
TEST(StatefulNodelet, sleepFinishSystime)  // NOLINT
{
  ros::Time::init();  // use system time
  ASSERT_TRUE(ros::Time::isSystemTime());

  // Create a thread that sleeps for 1 second. After 0.1 s, check that it has started but not finished. After another
  // 1 s, check that it has finished.

  TestNodelet nodelet;
  nodelet.init("test", {}, {});
  bool started = false;
  bool finished = false;
  std::thread t([&]()
    {
      started = true;
      EXPECT_TRUE(nodelet.sleep(ros::Duration(1)));
      finished = true;
    });
  
  for (size_t i = 0; i < 1000 && !started; ++i)
    ros::WallDuration(0.001).sleep();
  EXPECT_TRUE(started);
  EXPECT_FALSE(finished);
  
  ros::WallDuration(1.1).sleep();

  EXPECT_TRUE(finished);
  
  t.detach();  // detach the thread so that it doesn't block if the sleep did not end
}

/**
 * \brief Test finishing `sleep()` function with simulation time.
 */
TEST(StatefulNodelet, sleepFinishSimtime)  // NOLINT
{
  ros::Time::setNow({10, 0});  // use simulation time
  ASSERT_TRUE(ros::Time::isSimTime());

  // Create a thread that sleeps for 1 second. After 0.1 s, check that it has started but not finished. After another
  // 1.1 s, check that it has finished.

  TestNodelet nodelet;
  nodelet.init("test", {}, {});
  bool started = false;
  bool finished = false;
  std::thread t([&]()
    {
      started = true;
      EXPECT_TRUE(nodelet.sleep({1, 0}));
      finished = true;
    });
  
  for (size_t i = 0; i < 1000 && !started; ++i)
    ros::WallDuration(0.001).sleep();
  EXPECT_TRUE(started);
  EXPECT_FALSE(finished);
  
  ros::Time::setNow(ros::Time(11.1));
  ros::WallDuration(0.1).sleep();

  EXPECT_TRUE(finished);
  
  t.detach();  // detach the thread so that it doesn't block if the sleep did not end
}

/**
 * \brief Test `ok()` on an uninitialized and unmanaged nodelet.
 */
TEST(StatefulNodelet, uninitialized)  // NOLINT
{
  bool wasOk {true};
  {
    TestNodelet nodelet;
    nodelet.wasOk = &wasOk;
    nodelet.requestStop();
    ros::WallDuration(0.1).sleep();
  }
  EXPECT_FALSE(wasOk);
}

/**
 * \brief Test that `sleep()` can be interrupted by nodelet unloading in system time.
 */
TEST(StatefulNodelet, sleepInterruptByUnloadSystime)  // NOLINT
{
  ros::Time::init();
  ASSERT_TRUE(ros::Time::isSystemTime());
  
  // Can't be shared_ptr - it would lead to a segfault as Loader::unload() destroys the callback queues that the nodelet
  // needs for its own destruction (to unregister its subscription helper). If the nodelet would outlive Loader in a
  // shared_ptr, its destruction would therefore fail.
  TestNodelet* nodelet;
  // Loader is the standard class used for managing nodelets in a nodelet manager. We pass it a custom create_instance
  // function that disables the normal pluginlib lookup algorithm and allows us to create the instances manually.
  // We create it in a separate detached thread so that its destructor (which waits for all worker threads to finish)
  // does not block the other tests (in case this test would never finish). One of the worker threads is spinning the
  // nodelet's callback queue, so it might happen that it gets stuck.
  nodelet::Loader* loaderPointer {nullptr};
  bool stop = false;
  std::thread loaderThread([&]()
    {
      nodelet::Loader l([&](const std::string&)
        {
          return boost::shared_ptr<TestNodelet>(nodelet = new TestNodelet);
        });
      loaderPointer = &l;
      // Keep loaderThread running until the end of this test.
      while (!stop && ros::ok())
      {
        ros::spinOnce();
        ros::WallDuration(0, 1000000).sleep();
      }
    });
  // Wait until loaderThread creates the loader
  while (loaderPointer == nullptr)
    ros::WallDuration(0, 1000).sleep();
  // Detach the loaderThread so that it doesn't cause a segfault when exiting the program (as we can't join() it).
  loaderThread.detach();
  nodelet::Loader& loader = *loaderPointer;
  
  // Load a nodelet using the Loader. This will trigger the custom create_instance function and set `nodelet`.
  EXPECT_TRUE(loader.load("my_nodelet", "MyNodelet", {}, {}));
  ASSERT_NE(nullptr, nodelet);
  volatile bool wasOk {true};
  nodelet->wasOk = &wasOk;
  EXPECT_TRUE(nodelet->ok());

  // First, try if the sleep will finish if waiting enough time
  
  volatile bool started = false;
  volatile bool startedExecuting = false;
  volatile bool finished = false;
  std::thread t([&]()
    {
      started = true;
      EXPECT_TRUE(nodelet->sleepCb(ros::Duration(1), startedExecuting));
      EXPECT_TRUE(nodelet->ok());
      finished = true;
    });
  
  for (size_t i = 0; i < 1000 && !started; ++i)
    ros::WallDuration(0.001).sleep();
  EXPECT_TRUE(started);
  EXPECT_FALSE(finished);

  for (size_t i = 0; i < 1000 && !startedExecuting; ++i)
    ros::WallDuration(0.001).sleep();
  EXPECT_TRUE(startedExecuting);
  
  ros::WallDuration(1.01).sleep();
  EXPECT_TRUE(finished);

  // Now, test that the sleep can be interrupted.

  started = false;
  startedExecuting = false;
  finished = false;
  std::thread t2([&]()
    {
      started = true;
      EXPECT_FALSE(nodelet->sleepCb({1, 0}, startedExecuting));
      finished = true;
      ros::WallDuration(0.1).sleep();
      // cannot test EXPECT_TRUE(nodelet->ok()) here as nodelet is no longer valid
      EXPECT_FALSE(wasOk);
    });

  for (size_t i = 0; i < 1000 && !started; ++i)
    ros::WallDuration(0.001).sleep();
  EXPECT_TRUE(started);
  EXPECT_FALSE(finished);

  for (size_t i = 0; i < 1000 && !startedExecuting; ++i)
    ros::WallDuration(0.001).sleep();
  EXPECT_TRUE(startedExecuting);

  EXPECT_TRUE(wasOk);
  loader.unload("my_nodelet");
  ros::WallDuration(0.1).sleep();  // wait for the unloading to clean callback queues
  EXPECT_FALSE(wasOk);

  ros::WallDuration(0.1).sleep();
  EXPECT_TRUE(finished);
  
  t.detach();  // detach the threads so that they don't block if the sleep did not end
  t2.detach();
  stop = true;  // stop the loaderThread
}

/**
 * \brief Test that `sleep()` can be interrupted by nodelet unloading in sim time.
 */
TEST(StatefulNodelet, sleepInterruptByUnloadSimtime)  // NOLINT
{
  // The test runs with paused sim time to make the sleep() call infinitely waiting.
  ros::Time::setNow({10, 0});
  ASSERT_TRUE(ros::Time::isSimTime());
  ASSERT_EQ(10, ros::Time::now().sec);
  
  // Can't be shared_ptr - it would lead to a segfault as Loader::unload() destroys the callback queues that the nodelet
  // needs for its own destruction (to unregister its subscription helper). If the nodelet would outlive Loader in a
  // shared_ptr, its destruction would therefore fail.
  TestNodelet* nodelet;
  // Loader is the standard class used for managing nodelets in a nodelet manager. We pass it a custom create_instance
  // function that disables the normal pluginlib lookup algorithm and allows us to create the instances manually.
  // We create it in a separate detached thread so that its destructor (which waits for all worker threads to finish)
  // does not block the other tests (in case this test would never finish). One of the worker threads is spinning the
  // nodelet's callback queue, so it might happen that it gets stuck.
  nodelet::Loader* loaderPointer {nullptr};
  bool stop = false;
  std::thread loaderThread([&]()
    {
      nodelet::Loader l([&](const std::string&)
        {
          return boost::shared_ptr<TestNodelet>(nodelet = new TestNodelet);
        });
      loaderPointer = &l;
      // Keep loaderThread running until the end of this test.
      while (!stop && ros::ok())
      {
        ros::spinOnce();
        ros::WallDuration(0, 1000000).sleep();
      }
    });
  // Wait until loaderThread creates the loader
  while (loaderPointer == nullptr)
    ros::WallDuration(0, 1000).sleep();
  // Detach the loaderThread so that it doesn't cause a segfault when exiting the program (as we can't join() it).
  loaderThread.detach();
  nodelet::Loader& loader = *loaderPointer;
  
  // Load a nodelet using the Loader. This will trigger the custom create_instance function and set `nodelet`.
  EXPECT_TRUE(loader.load("my_nodelet", "MyNodelet", {}, {}));
  EXPECT_NE(nullptr, nodelet);
  volatile bool wasOk {true};
  nodelet->wasOk = &wasOk;
  EXPECT_TRUE(nodelet->ok());

  // First, try if the sleep will finish if waiting enough time
  
  bool started = false;
  bool startedExecuting = false;
  bool finished = false;
  std::thread t([&]()
    {
      EXPECT_TRUE(nodelet->ok());
      started = true;
      EXPECT_TRUE(nodelet->sleepCb({1, 0}, startedExecuting));
      finished = true;
      EXPECT_TRUE(nodelet->ok());
    });
  
  for (size_t i = 0; i < 1000 && !started; ++i)
    ros::WallDuration(0.001).sleep();
  EXPECT_TRUE(started);
  EXPECT_FALSE(finished);

  for (size_t i = 0; i < 1000 && !startedExecuting; ++i)
    ros::WallDuration(0.001).sleep();
  EXPECT_TRUE(startedExecuting);

  EXPECT_TRUE(nodelet->ok());
  
  ros::Time::setNow(ros::Time(11.1));
  ros::WallDuration(0.1).sleep();
  EXPECT_TRUE(finished);

  EXPECT_TRUE(nodelet->ok());
  
  // Now, test that the sleep can be interrupted.

  ros::Time::setNow(ros::Time(10));
  
  started = false;
  startedExecuting = false;
  finished = false;
  std::thread t2([&]()
    {
      EXPECT_TRUE(nodelet->ok());
      started = true;
      EXPECT_FALSE(nodelet->sleepCb({1, 0}, startedExecuting));
      finished = true;
      // cannot test EXPECT_TRUE(nodelet->ok()) here as nodelet is no longer valid
      ros::WallDuration(0.1).sleep();
      EXPECT_FALSE(wasOk);
    });

  ros::Time::setNow(ros::Time(10.1));  // not enough for the sleep to finish 

  for (size_t i = 0; i < 1000 && !started; ++i)
    ros::WallDuration(0.001).sleep();
  EXPECT_TRUE(started);
  EXPECT_FALSE(finished);

  for (size_t i = 0; i < 1000 && !startedExecuting; ++i)
    ros::WallDuration(0.001).sleep();
  EXPECT_TRUE(startedExecuting);

  EXPECT_TRUE(nodelet->ok());
  EXPECT_TRUE(wasOk);
  loader.unload("my_nodelet");
  ros::WallDuration(0.1).sleep();  // wait for the unloading to clean callback queues
  EXPECT_FALSE(wasOk);

  ros::WallDuration(0.1).sleep();
  EXPECT_TRUE(finished);
  
  t.detach();  // detach the threads so that they don't block if the sleep did not end
  t2.detach();
  stop = true;  // stop the loaderThread
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_nodelet_request_stop");
  ros::start();
  return RUN_ALL_TESTS();
}
