#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

class TeleopJoyPS3 {
public:
  TeleopJoyPS3();
  ~TeleopJoyPS3();
  void joyCB(const sensor_msgs::JoyConstPtr& joy);
  ros::NodeHandle nh_;
  //ros::NodeHandle nh_pub_;

private:
  int linear_, angular_;
  double linear_scale, angular_scale_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;
};

TeleopJoyPS3::TeleopJoyPS3() : linear_(1), angular_(2) {
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1);
  joy_sub_ = nh_.subscribe("/joy",1, &TeleopJoyPS3::joyCB, this);
}

TeleopJoyPS3::~TeleopJoyPS3() {

}

void TeleopJoyPS3::joyCB(const sensor_msgs::Joy::ConstPtr& joy) {
  geometry_msgs::Twist twistmsg;
  twistmsg.linear.x = joy->axes[linear_];
  twistmsg.angular.z = joy->axes[angular_];
  ros::Rate loop_rate(60);
  vel_pub_.publish(twistmsg);
}
int main(int argc, char** argv) {
  ros::init(argc, argv, "joy_teleop_sub");
  //ros::init(argc, argv, "joy_teleop_pub");
  TeleopJoyPS3 teleopJoy;

  ros::spin();
}
