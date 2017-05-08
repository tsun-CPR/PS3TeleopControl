#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

class TeleopJoyPS3 {
public:
  TeleopJoyPS3();
  ~TeleopJoyPS3();
  void joyCB(const sensor_msgs::JoyConstPtr& joy);
  void vel_publish();
  ros::NodeHandle sh_;
  ros::NodeHandle ph_;
  ros::Timer timer_;

private:
  int linear_, angular_;
  double linear_scale_, angular_scale_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;
  geometry_msgs::Twist to_pub;
};

TeleopJoyPS3::TeleopJoyPS3() : linear_(1), angular_(2), linear_scale_(0.3), angular_scale_(2.0) {
    vel_pub_ = ph_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1, true);
    joy_sub_ = sh_.subscribe("/joy",1, &TeleopJoyPS3::joyCB, this);

    timer_ = sh_.createTimer(ros::Duration(0.1), boost::bind(&TeleopJoyPS3::vel_publish, this));
}

TeleopJoyPS3::~TeleopJoyPS3() {

}

void TeleopJoyPS3::joyCB(const sensor_msgs::Joy::ConstPtr& joy) {
  geometry_msgs::Twist twistmsg;
  twistmsg.linear.x = linear_scale_ * joy->axes[linear_];
  twistmsg.angular.z = angular_scale_ * joy->axes[angular_];

  to_pub = twistmsg;
}

void TeleopJoyPS3::vel_publish() {
    vel_pub_.publish(to_pub);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "joy_teleop_sub");
  //ros::init(argc, argv, "joy_teleop_pub");
  TeleopJoyPS3 teleopJoy;
  ros::spin();
}
