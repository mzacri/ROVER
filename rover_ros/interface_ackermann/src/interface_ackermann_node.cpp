#include <ros/ros.h>
#include <simple_ackermann_msgs/SimpleAckermannStamped.h>
#include "recursive_bayesian_filter_ros/OdomMeasurementAckermann.h"

class Interface
{
public:
  ros::NodeHandle nh;
  ros::Publisher pub;
  ros::Subscriber sub;

  Interface()
	: nh("~") 
	, pub(nh.advertise<recursive_bayesian_filter_ros::OdomMeasurementAckermann>("/state_measure_odom", 2))
	, sub(nh.subscribe("/state_measure", 2, &Interface::cb,this))
  {
  }

  void cb(const simple_ackermann_msgs::SimpleAckermannStampedConstPtr& msg)
  {
	recursive_bayesian_filter_ros::OdomMeasurementAckermann oma;
	oma.header = msg->header;
	oma.v = msg->command.speed;
	oma.phi = msg->command.steering;
	oma.covariance = msg->command.covariance;

	pub.publish(oma);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "interface_ackermann");

  auto a = Interface();

  ros::spin();
}
