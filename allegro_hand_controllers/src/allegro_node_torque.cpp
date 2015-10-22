using namespace std;

#include "allegro_node_torque.h"
#include <stdio.h>

#include "ros/ros.h"
#include "allegro_hand_driver/controlAllegroHand.h"

// Topics
const std::string TORQUE_CMD_TOPIC = "/allegroHand/torque_cmd";
const std::string LIB_CMD_TOPIC = "/allegroHand/lib_cmd";



// Constructor: subscribe to topics.
AllegroNodeTorque::AllegroNodeTorque()
  : AllegroNode() {  // Call super constructor.

  initController(whichHand);

  torque_cmd_sub = nh.subscribe(
                     TORQUE_CMD_TOPIC, 1, &AllegroNodeTorque::setTorqueCallback, this, ros::TransportHints().tcpNoDelay());
  lib_cmd_sub = nh.subscribe(
                  LIB_CMD_TOPIC, 1, &AllegroNodeTorque::libCmdCallback, this);
}

AllegroNodeTorque::~AllegroNodeTorque() {
  ROS_INFO("PD controller node is shutting down");
}

// Called when a desired joint position message is received
void AllegroNodeTorque::setTorqueCallback(const sensor_msgs::JointState &msg) {

  // No need to lock a mutex here
  for (int i = 0; i < DOF_JOINTS; i++)
    desired_torque[i] = msg.effort[i];


  controlTorque = true;
}

// Called when an external (string) message is received
void AllegroNodeTorque::libCmdCallback(const std_msgs::String::ConstPtr &msg) {
  ROS_INFO("CTRL: Heard: [%s]", msg->data.c_str());

  const std::string lib_cmd = msg->data.c_str();

  // Compare the message received to an expected input
  if (lib_cmd.compare("on") == 0)
    controlTorque = true;
  else if (lib_cmd.compare("off") == 0)
    controlTorque = false;

}

void AllegroNodeTorque::computeDesiredTorque() {
  // Position PD control for the desired joint configurations.

  if(controlTorque){
    //    for (int i = 0; i < DOF_JOINTS; i++)
    //      desired_torque[i] = desired_torque[i];
  }else{
    for (int i = 0; i < DOF_JOINTS; i++)
      desired_torque[i] = 0.0;
  }

}

void AllegroNodeTorque::initController(const std::string &whichHand) {

  controlTorque = false;


  printf("*************************************\n");
  printf("      Joint PD Control Method        \n");
  printf("-------------------------------------\n");
  printf("  Only 'H', 'O', 'S', 'Space' works. \n");
  printf("*************************************\n");
}

void AllegroNodeTorque::doIt(bool polling) {
  // Main spin loop, uses the publisher/subscribers.
  if (polling) {
    ROS_INFO("Polling = true.");
    while (ros::ok()) {
      updateController();
      ros::spinOnce();
    }
  } else {
    ROS_INFO("Polling = false.");

    // Timer callback (not recommended).
    ros::Timer timer = startTimerCallback();
    ros::spin();
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "allegro_hand_core_torque");
  AllegroNodeTorque allegroNode;

  bool polling = false;
  if (argv[1] == std::string("true")) {
    polling = true;
  }
  allegroNode.doIt(polling);
}
