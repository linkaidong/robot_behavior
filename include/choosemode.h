//
// Created by yawara on 23-9-2.
//

#ifndef SRC_CHOOSEMODE_H
#define SRC_CHOOSEMODE_H

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include "ros/ros.h"
#include <XmlRpcException.h>
#include <rm_common/decision/controller_manager.h>
#include <rm_common/decision/calibration_queue.h>
#include <rm_common//decision/command_sender.h>
#include <control_msgs/QueryCalibrationState.h>
#include <rm_msgs/DbusData.h>

namespace behavior_tree {
class ShutDown : public BT::SyncActionNode {
public:
    ShutDown(const std::string &name, const BT::NodeConfiguration &config, ros::NodeHandle &root_nh,
               const ros::NodeHandle &decision_nh);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};

class ChooseMode : public BT::SyncActionNode{
public:
    ChooseMode(const std::string &name, const BT::NodeConfiguration &config, ros::NodeHandle &root_nh,
               const ros::NodeHandle &decision_nh);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

    rm_common::CalibrationQueue *gimbal_calibration_{}, *shooter_calibration_{};
    rm_common::ControllerManager controller_manager_;
//    rm_common::DoubleBarrelCommandSender* double_barrel_cmd_sender_;
    rm_msgs::DbusData dbus_;
    bool check_rc_state_{false};
private:
    void init();
    void controllerCalibrate();
    void controllerUpdate();
    void dbusCallback(const rm_msgs::DbusData::ConstPtr& data);
    void checkRcState(const ros::Time &time);

    ros::Subscriber dbus_sub_;
    std::string mode;


};
}// behavior_tree
#endif //SRC_CHOOSEMODE_H
