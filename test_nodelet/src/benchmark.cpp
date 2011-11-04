#include <nodelet/detail/callback_queue_manager.h>
#include <nodelet/detail/callback_queue.h>
#include <ros/callback_queue.h>
#include <ros/time.h>

#include <boost/thread.hpp>
#include <boost/detail/atomic_count.hpp>
#include <cstdio>

using namespace nodelet::detail;
using boost::detail::atomic_count;

static const long NUM_CALLBACKS = 1e7;

atomic_count g_count(NUM_CALLBACKS);
boost::mutex g_mutex;
boost::condition_variable g_cond;

class MyCallback : public ros::CallbackInterface
{
public:
  ros::CallbackInterface::CallResult call()
  {
    if (--g_count == 0)
    {
      boost::mutex::scoped_lock lock(g_mutex);
      g_cond.notify_all();
    }

    return Success;
  }
};
typedef boost::shared_ptr<MyCallback> MyCallbackPtr;

int main(int argc, char** argv)
{
  CallbackQueueManager man;
  CallbackQueuePtr queue(new CallbackQueue(&man));
  man.addQueue(queue, true);

  double start = ros::WallTime::now().toSec();
  
  for (long i = 0; i < NUM_CALLBACKS; ++i)
  {
    MyCallbackPtr cb(new MyCallback);
    queue->addCallback(cb, 0);
  }

  {
    boost::mutex::scoped_lock lock(g_mutex);
    g_cond.wait(lock);
  }

  double end = ros::WallTime::now().toSec();
  printf("Total time = %.3f\n", end - start);

  return 0;
}
