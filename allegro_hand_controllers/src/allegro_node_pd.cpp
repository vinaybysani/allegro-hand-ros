using namespace std;

#include "allegro_node_pd.h"
#include <stdio.h>

#include "ros/ros.h"
#include "allegro_hand_common/controlAllegroHand.h"

// Topics
const std::string JOINT_CMD_TOPIC = "/allegroHand/joint_cmd";
const std::string LIB_CMD_TOPIC = "/allegroHand/lib_cmd";

#define RADIANS_TO_DEGREES(radians) ((radians) * (180.0 / M_PI))
#define DEGREES_TO_RADIANS(angle) ((angle) / 180.0 * M_PI)

// Default parameters.
double k_p[DOF_JOINTS] =
        {
                // Default P Gains for PD Controller, loaded if
                // 'gains_pd.yaml' file is not loaded.
                600.0, 600.0, 600.0, 1000.0, 600.0, 600.0, 600.0, 1000.0,
                600.0, 600.0, 600.0, 1000.0, 1000.0, 1000.0, 1000.0, 600.0
        };

double k_d[DOF_JOINTS] =
        {
                // Default D Gains for PD Controller, loaded if
                // 'gains_pd.yaml' file is not loaded.
                15.0, 20.0, 15.0, 15.0, 15.0, 20.0, 15.0, 15.0,
                15.0, 20.0, 15.0, 15.0, 30.0, 20.0, 20.0, 15.0
        };

double home_pose[DOF_JOINTS] =
        {
                // Default (HOME) position (degrees), set at system start if
                // no 'initial_position.yaml' parameter is loaded.
                0.0, -10.0, 45.0, 45.0,  0.0, -10.0, 45.0, 45.0,
                5.0, -5.0, 50.0, 45.0, 60.0, 25.0, 15.0, 45.0
        };

std::string pGainParams[DOF_JOINTS] =
        {
                "~gains_pd/p/j00", "~gains_pd/p/j01", "~gains_pd/p/j02",
                "~gains_pd/p/j03",
                "~gains_pd/p/j10", "~gains_pd/p/j11", "~gains_pd/p/j12",
                "~gains_pd/p/j13",
                "~gains_pd/p/j20", "~gains_pd/p/j21", "~gains_pd/p/j22",
                "~gains_pd/p/j23",
                "~gains_pd/p/j30", "~gains_pd/p/j31", "~gains_pd/p/j32",
                "~gains_pd/p/j33"
        };

std::string dGainParams[DOF_JOINTS] =
        {
                "~gains_pd/d/j00", "~gains_pd/d/j01", "~gains_pd/d/j02",
                "~gains_pd/d/j03",
                "~gains_pd/d/j10", "~gains_pd/d/j11", "~gains_pd/d/j12",
                "~gains_pd/d/j13",
                "~gains_pd/d/j20", "~gains_pd/d/j21", "~gains_pd/d/j22",
                "~gains_pd/d/j23",
                "~gains_pd/d/j30", "~gains_pd/d/j31", "~gains_pd/d/j32",
                "~gains_pd/d/j33"
        };

std::string initialPosition[DOF_JOINTS] =
        {
                "~initial_position/j00", "~initial_position/j01",
                "~initial_position/j02",
                "~initial_position/j03",
                "~initial_position/j10", "~initial_position/j11",
                "~initial_position/j12",
                "~initial_position/j13",
                "~initial_position/j20", "~initial_position/j21",
                "~initial_position/j22",
                "~initial_position/j23",
                "~initial_position/j30", "~initial_position/j31",
                "~initial_position/j32",
                "~initial_position/j33"
        };

// Constructor: subscribe to topics.
AllegroNodePD::AllegroNodePD()
        : AllegroNode() {  // Call super constructor.

  initController(whichHand);

  joint_cmd_sub = nh.subscribe(
          JOINT_CMD_TOPIC, 3, &AllegroNodePD::setJointCallback, this);
  lib_cmd_sub = nh.subscribe(
          LIB_CMD_TOPIC, 1, &AllegroNodePD::libCmdCallback, this);
}

AllegroNodePD::~AllegroNodePD() {
  ROS_INFO("PD controller node is shutting down");
}

// Called when a desired joint position message is received
void AllegroNodePD::setJointCallback(const sensor_msgs::JointState &msg) {
  mutex->lock();

  for (int i = 0; i < DOF_JOINTS; i++)
    desired_position[i] = msg.position[i];

  mutex->unlock();
  controlPD = true;
}

// Called when an external (string) message is received
void AllegroNodePD::libCmdCallback(const std_msgs::String::ConstPtr &msg) {
  ROS_INFO("CTRL: Heard: [%s]", msg->data.c_str());

  const std::string lib_cmd = msg->data.c_str();

  // Compare the message received to an expected input
  if (lib_cmd.compare("pdControl") == 0)
    controlPD = true;

  else if (lib_cmd.compare("home") == 0) {
    for (int i = 0; i < DOF_JOINTS; i++)
      desired_position[i] = DEGREES_TO_RADIANS(home_pose[i]);
    controlPD = true;
  }
  else if (lib_cmd.compare("off") == 0)
    controlPD = false;

  else if (lib_cmd.compare("save") == 0)
    for (int i = 0; i < DOF_JOINTS; i++)
      desired_position[i] = current_position[i];
}

void AllegroNodePD::computeDesiredTorque() {
  // Position PD control for the desired joint configurations.
  if (controlPD) {
    for (int i = 0; i < DOF_JOINTS; i++) {
      desired_torque[i] =
              k_p[i] * (desired_position[i] - current_position_filtered[i])
              - k_d[i] * current_velocity_filtered[i];
      desired_torque[i] = desired_torque[i] / canDevice->torqueConversion();
    }
  } else {
    for (int i = 0; i < DOF_JOINTS; i++)
      desired_torque[i] = 0.0;
  }
}

void AllegroNodePD::initController(const std::string &whichHand) {
  // set gains_pd via gains_pd.yaml or to default values
  if (ros::param::has("~gains_pd")) {
    ROS_INFO("CTRL: PD gains loaded from param server.");
    for (int i = 0; i < DOF_JOINTS; i++) {
      ros::param::get(pGainParams[i], k_p[i]);
      ros::param::get(dGainParams[i], k_d[i]);
    }
  }
  else {
    // gains will be loaded every control iteration
    ROS_WARN("CTRL: PD gains not loaded");
    ROS_WARN("Check launch file is loading /parameters/gains_pd.yaml");
    ROS_WARN("Loading default PD gains...");
  }

  // set initial position via initial_position.yaml or to default values
  if (ros::param::has("~initial_position")) {
    ROS_INFO("CTRL: Initial Pose loaded from param server.");
    for (int i = 0; i < DOF_JOINTS; i++) {
      ros::param::get(initialPosition[i], desired_position[i]);
      desired_position[i] = DEGREES_TO_RADIANS(desired_position[i]);
    }
  }
  else {
    ROS_WARN("CTRL: Initial postion not loaded.");
    ROS_WARN("Check launch file is loading /parameters/initial_position.yaml");
    ROS_WARN("Loading Home position instead...");

    // Home position
    for (int i = 0; i < DOF_JOINTS; i++)
      desired_position[i] = DEGREES_TO_RADIANS(home_pose[i]);
  }
  controlPD = false;

  printf("*************************************\n");
  printf("      Joint PD Control Method        \n");
  printf("-------------------------------------\n");
  printf("  Only 'H', 'O', 'S', 'Space' works. \n");
  printf("*************************************\n");
}

void AllegroNodePD::doIt(bool polling) {
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
  ros::init(argc, argv, "allegro_hand_core_pd");
  AllegroNodePD allegroNodePD;

  bool polling = false;
  if (argv[1] == std::string("true")) {
    polling = true;
  }
  allegroNodePD.doIt(polling);
}
