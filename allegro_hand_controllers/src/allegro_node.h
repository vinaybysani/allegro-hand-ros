//
// Created by felixd on 10/1/15.
//

#ifndef PROJECT_ALLEGRO_NODE_COMMON_H
#define PROJECT_ALLEGRO_NODE_COMMON_H

// Defines DOF_JOINTS.
#include "allegro_hand_driver/allegroCANProtocol.h"

#include <string>
#include <boost/thread/thread.hpp>

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/String.h"

// Forward declaration.
class controlAllegroHand;

// Topic names.
const std::string JOINT_STATE_TOPIC = "/allegroHand/joint_states";
const std::string DESIRED_STATE_TOPIC = "/allegroHand/joint_cmd";


class AllegroNode {
 public:

  AllegroNode();

  virtual ~AllegroNode();

  void publishData();

  void desiredStateCallback(const sensor_msgs::JointState &desired);

  void updateWriteReadCAN();

  void updateController();

  // This is the main method that must be implemented by the various
  // controller nodes.
  virtual void computeDesiredTorque() {
    ROS_ERROR("Called virtual function!");
  };

  ros::Timer startTimerCallback();

  void timerCallback(const ros::TimerEvent &event);

 protected:

  double current_position[DOF_JOINTS] = {0.0};
  double previous_position[DOF_JOINTS] = {0.0};

  double current_position_filtered[DOF_JOINTS] = {0.0};
  double previous_position_filtered[DOF_JOINTS] = {0.0};

  double current_velocity[DOF_JOINTS] = {0.0};
  double previous_velocity[DOF_JOINTS] = {0.0};
  double current_velocity_filtered[DOF_JOINTS] = {0.0};

  double desired_torque[DOF_JOINTS] = {0.0};

  std::string whichHand;  // Right or left hand.

  // ROS stuff
  ros::NodeHandle nh;
  ros::Publisher joint_state_pub;
  ros::Subscriber joint_cmd_sub_;

  // Store the current and desired joint states.
  sensor_msgs::JointState current_joint_state;
  sensor_msgs::JointState desired_joint_state_;

  // ROS Time
  ros::Time tstart;
  ros::Time tnow;
  double dt;

  // CAN device
  controlAllegroHand *canDevice;
  boost::mutex *mutex;

  // Flags
  int lEmergencyStop = 0;
  long frame = 0;
};

#endif //PROJECT_ALLEGRO_NODE_COMMON_H
