#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Image.h>
#include <boost/pool/pool_alloc.hpp>

namespace sensor_msgs {
ROS_DECLARE_MESSAGE_WITH_ALLOCATOR(Image, PoolImage, boost::fast_pool_allocator)
}

namespace {

constexpr char LOGNAME[] = "pool_allocator_demo";
constexpr char ROS_THREADS = 2;
constexpr size_t WIDTH = 640;
constexpr size_t HEIGHT = 480;

// Useful template for creating messages from a message pool
template <typename T>
boost::shared_ptr<T> make_shared_from_pool() {
  using allocator_t = boost::fast_pool_allocator<boost::shared_ptr<T>>;
  return boost::allocate_shared<T, allocator_t>(allocator_t());
}

class TestProducer {
 public:
  TestProducer(ros::NodeHandle nh) {
    image_pub_ = nh.advertise<sensor_msgs::PoolImage>("image", 1);
    duration_pub_ = nh.advertise<std_msgs::Float64>("profile_last_duration", 10);
    timer_ = nh.createTimer(ros::Duration(0.01), &TestProducer::run, this);
    timer_.start();
  }

  ~TestProducer() { timer_.stop(); }

 private:
  ros::Publisher image_pub_;
  ros::Publisher duration_pub_;
  ros::Timer timer_;
  size_t index_ = 0;

  void run(const ros::TimerEvent& timer_event) {
    auto duration_msg = make_shared_from_pool<std_msgs::Float64>();
    duration_msg->data = timer_event.profile.last_duration.toSec();
    duration_pub_.publish(duration_msg);

    auto image_msg = make_shared_from_pool<sensor_msgs::PoolImage>();

    image_msg->header.stamp = ros::Time::now();

    image_msg->height = HEIGHT;
    image_msg->width = WIDTH;

    // allocate some data, this isn't a real image, this is an example.
    image_msg->data.resize(HEIGHT * WIDTH * 32);
    image_msg->data[0] = index_++;

    ROS_INFO_STREAM_NAMED(LOGNAME, "pub\t" << static_cast<int>(image_msg->data[0])
                                           << "\t" << image_msg.get());

    image_pub_.publish(image_msg);
  }
};

class TestConsumer {
 public:
  TestConsumer(ros::NodeHandle nh) {
    image_sub_ = nh.subscribe("image", 10, &TestConsumer::cb, this);
  }

 private:
  ros::Subscriber image_sub_;

  void cb(boost::shared_ptr<const sensor_msgs::PoolImage> msg) {
    ROS_INFO_STREAM_NAMED(LOGNAME, "sub\t" << static_cast<int>(msg->data[0])
                                           << "\t" << msg.get());
  }
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, LOGNAME);
  ros::AsyncSpinner spinner(ROS_THREADS);
  spinner.start();

  ros::NodeHandle nh;
  TestProducer test_producer(nh);
  TestConsumer test_consumer(nh);

  ros::waitForShutdown();
}
