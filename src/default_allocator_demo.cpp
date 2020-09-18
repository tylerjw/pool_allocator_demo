#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>

namespace {

constexpr char LOGNAME[] = "default_allocator_demo";
constexpr char ROS_THREADS = 2;
constexpr size_t WIDTH = 640;
constexpr size_t HEIGHT = 480;

class TestProducer {
 public:
  TestProducer(ros::NodeHandle nh) {
    imu_pub_ = nh.advertise<sensor_msgs::Imu>("imu", 1);
    duration_pub_ = nh.advertise<std_msgs::Float64>("profile_last_duration", 10);
    timer_ = nh.createTimer(ros::Duration(0.005), &TestProducer::run, this);
    timer_.start();
  }

  ~TestProducer() { timer_.stop(); }

 private:
  ros::Publisher imu_pub_;
  ros::Publisher duration_pub_;
  ros::Timer timer_;
  size_t index_ = 0;

  void run(const ros::TimerEvent& timer_event) {
    auto duration_msg = boost::make_shared<std_msgs::Float64>();
    duration_msg->data = timer_event.profile.last_duration.toSec();
    duration_pub_.publish(duration_msg);

    auto imu_msg = boost::make_shared<sensor_msgs::Imu>();

    imu_msg->header.stamp = ros::Time::now();

    imu_msg->orientation_covariance[0] = index_++;

    ROS_INFO_STREAM_NAMED(LOGNAME, "pub\t" << static_cast<int>(imu_msg->orientation_covariance[0])
                                           << "\t" << imu_msg.get());

    imu_pub_.publish(imu_msg);
  }
};

class TestConsumer {
 public:
  TestConsumer(ros::NodeHandle nh) {
    imu_sub_ = nh.subscribe("imu", 10, &TestConsumer::cb, this);
  }

 private:
  ros::Subscriber imu_sub_;

  void cb(boost::shared_ptr<const sensor_msgs::Imu> msg) {
    ROS_INFO_STREAM_NAMED(LOGNAME, "sub\t" << static_cast<int>(msg->orientation_covariance[0])
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
