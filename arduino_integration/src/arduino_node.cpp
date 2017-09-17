#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"

using namespace std;

std_msgs::Float64 servo_1_1_position, servo_2_2_position,
  servo_3_1_position, servo_4_1_position, arm_velocity;
  
ros::Publisher pub1;
ros::Publisher pub2;
ros::Publisher pub3;
ros::Publisher pub4;

void callback(const sensor_msgs::JointState &msg){
  servo_1_1_position.data = msg.position[0];
  servo_2_2_position.data = msg.position[1];
  servo_3_1_position.data = msg.position[2];
  servo_4_1_position.data = msg.position[3];
  pub1.publish(servo_1_1_position);
  pub2.publish(servo_2_2_position);
  pub3.publish(servo_3_1_position);
  pub4.publish(servo_4_1_position);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "arduino_node");
  ros::NodeHandle n;
  // Instancing multiple publishers. Subject to change for a list of
  // joint_states. 
  pub1 = n.advertise<std_msgs::Float64>("servo_1_1_position", 100);
  pub2 = n.advertise<std_msgs::Float64>("servo_2_2_position", 100);
  pub3 = n.advertise<std_msgs::Float64>("servo_3_1_position", 100);
  pub4 = n.advertise<std_msgs::Float64>("servo_4_1_position", 100);
  
  // Subscribing to the joint_state_publisher.
  ros::Subscriber sub = n.subscribe("five_dof_arm/joint_states", 10000, callback);
  ros::spin();
  return 0;
}
