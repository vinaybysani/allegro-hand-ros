#ifndef __ALLEGRO_NODE_PD_H__
#define __ALLEGRO_NODE_PD_H__

#include "allegro_node.h"


// Joint-space PD control of the Allegro hand.
//
// Allows you to save a position and command it to the hand controller.
// Controller gains are loaded from the ROS parameter server.
class AllegroNodeTorque : public AllegroNode {

 public:
    AllegroNodeTorque();

    ~AllegroNodeTorque();

    // Main spin code: just waits for messages.
    void doIt(bool polling = false);

    // Sets desired joint positions based on joint positions in a JointState
    // message.
    void setTorqueCallback(const sensor_msgs::JointState &msg);

    // Uses the String received command to set the hand into its home
    // position, or saves the grasp in order to go into PD control mode. Also
    // can turn the hand off.
    void libCmdCallback(const std_msgs::String::ConstPtr &msg);

    // Loads all gains and initial positions from the parameter server.
    void initController(const std::string &whichHand);

    // PD control happens here.
    void computeDesiredTorque();

 protected:
    // Handles external joint command (sensor_msgs/JointState).
    ros::Subscriber torque_cmd_sub;

    // Handles defined grasp commands (std_msgs/String).
    ros::Subscriber lib_cmd_sub;

    // If true, PD control is active.
    bool controlTorque = false;

};

#endif  // __ALLEGRO_NODE_PD_H__
