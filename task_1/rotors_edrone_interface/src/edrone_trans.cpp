
#include "rotors_edrone_interface/edrone_trans.h"

#include <mav_msgs/default_topics.h>

Edrone_Trans::Edrone_Trans() {
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ctrl_pub_ = nh_.advertise<mav_msgs::RollPitchYawrateThrust> (
    mav_msgs::default_topics::COMMAND_ROLL_PITCH_YAWRATE_THRUST, 50);

  control_msg_.roll = 0;
  control_msg_.pitch = 0;
  control_msg_.yaw_rate = 0;
  control_msg_.thrust.x = 0;
  control_msg_.thrust.y = 0;
  control_msg_.thrust.z = 0;
  current_yaw_vel_ = 0;

  pnh.param("axis_direction_roll", axes_.roll_direction, 1);
  pnh.param("axis_direction_pitch", axes_.pitch_direction, 1);
  pnh.param("axis_direction_thrust", axes_.thrust_direction, 1);

  pnh.param("max_v_xy", max_.v_xy, 0.8);  // [m/s]
  pnh.param("max_roll", max_.roll, 4.5 * M_PI / 180.0);  // [rad]
  pnh.param("max_pitch", max_.pitch, 4.5 * M_PI / 180.0);  // [rad]
  pnh.param("max_yaw_rate", max_.rate_yaw, 45.0 * M_PI / 180.0);  // [rad/s]
  pnh.param("max_thrust", max_.thrust, 30.0);  // [N]
  pnh.param("division_factor", div_factor, 85.0);  
  pnh.param("v_yaw_step", v_yaw_step_, 0.05);  // [rad/s]
  pnh.param("v_yaw_step", v_yaw_step_, 0.05);
  pnh.param("thrust_weight_offset_newtons",thrust_weight_offset_newtons_, 14.896);
  edrone_sub_ = nh_.subscribe("/drone_command", 50, &Edrone_Trans::TransCallback, this);
  namespace_ = nh_.getNamespace();
}

void Edrone_Trans::StopMav() {
  control_msg_.roll = 0;
  control_msg_.pitch = 0;
  control_msg_.yaw_rate = 0;
  control_msg_.thrust.x = 0;
  control_msg_.thrust.y = 0;
  control_msg_.thrust.z = 0;
}

void Edrone_Trans::TransCallback(const edrone_client::edrone_msgs::ConstPtr& msg) {
  control_msg_.roll = ((msg->rcRoll-1500) * max_.roll/div_factor)*axes_.roll_direction;
  control_msg_.pitch = ((msg->rcPitch-1500) * max_.pitch/div_factor)* axes_.pitch_direction;
  control_msg_.yaw_rate = current_yaw_vel_;
  //Option 1
  if(msg->rcThrottle>1499)
    control_msg_.thrust.z = ((msg->rcThrottle-1500)*(max_.thrust-thrust_weight_offset_newtons_)/500.0)+(thrust_weight_offset_newtons_);
  else
    control_msg_.thrust.z = (1+((msg->rcThrottle-1500)/500.0))*(thrust_weight_offset_newtons_);
  //Option 2 Vishal's gains work on this option
  // control_msg_.thrust.z = ((msg->rcThrottle-1500) * max_.thrust / 100.0 * axes_.thrust_direction)+thrust_weight_offset_newtons_;
  //Option 3
  // control_msg_.thrust.z = ((msg->rcThrottle-1500) * max_.thrust / 50.0 * axes_.thrust_direction)+thrust_weight_offset_newtons_;  

  ros::Time update_time = ros::Time::now();
  control_msg_.header.stamp = update_time;
  control_msg_.header.frame_id = "rotors_edrone_frame";
  Publish();
}

void Edrone_Trans::Publish() {
  ctrl_pub_.publish(control_msg_);
}
int main(int argc, char** argv) {
  ros::init(argc, argv, "rotors_edrone_interface");
  Edrone_Trans edrone_trans;
  ros::spin();
  return 0;
}
