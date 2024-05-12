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
             const ros::NodeHandle &decision_nh)
             : BT::SyncActionNode(name, config) {
        ros::NodeHandle shutdown_nh(decision_nh, "shutdown");
    }

    static BT::PortsList providedPorts() {
        BT::PortsList ports_list;
        return ports_list;
    }

    BT::NodeStatus tick() override{
//    if(check_rc_state_) {
//        ROS_INFO("Your State is RC_OFF");
//    } else {
//        ROS_INFO("Your State is at rm_msgs::DbusData::DOWN");
//    }
        return BT::NodeStatus::SUCCESS;
    }
};

class ChooseMode : public BT::SyncActionNode{
public:
    ChooseMode(const std::string &name, const BT::NodeConfiguration &config, ros::NodeHandle &root_nh,
               const ros::NodeHandle &decision_nh)
               : BT::SyncActionNode(name, config) , controller_manager_(root_nh) {

        ros::NodeHandle choosemode_nh(decision_nh, "choosemode");
//    ros::NodeHandle barrel_nh(decision_nh, "barrel");
//    double_barrel_cmd_sender_ = new rm_common::DoubleBarrelCommandSender(barrel_nh);

        /*   sentry_init */
        try{
            XmlRpc::XmlRpcValue gimbal_calibration_param, shooter_calibration_param;
            root_nh.getParam("gimbal_calibration", gimbal_calibration_param);
            gimbal_calibration_ = new rm_common::CalibrationQueue(gimbal_calibration_param, choosemode_nh, controller_manager_);
            root_nh.getParam("shooter_calibration", shooter_calibration_param);
            shooter_calibration_ = new rm_common::CalibrationQueue(shooter_calibration_param, choosemode_nh, controller_manager_);
        }catch (XmlRpc::XmlRpcException& e){
            ROS_ERROR("%s", e.getMessage().c_str());
        }
        controller_manager_.startStateControllers();
        controllerCalibrate();
        /*   sentry_init */

        dbus_sub_ = root_nh.subscribe<rm_msgs::DbusData>("/dbus_data", 10, &ChooseMode::dbusCallback, this);
    }

    static BT::PortsList providedPorts() {
        return { BT::OutputPort<std::string>("mode") };
    }

    BT::NodeStatus tick() override{
        init();
        ros::Time time = ros::Time::now();
        checkRcState(time);
        if(!check_rc_state_){
            mode = "shutdown";
            ROS_INFO("Your RC is OFF");
            controller_manager_.stopMainControllers();
        } else {
            switch(dbus_.s_r) {
                case rm_msgs::DbusData::DOWN:
                    mode = "shutdown";
                    controller_manager_.stopMainControllers();
                    break;
                case rm_msgs::DbusData::MID:
                    mode = "manual";
                    controller_manager_.startMainControllers();
                    break;
                case rm_msgs::DbusData::UP:
                    mode = "auto";
                    controller_manager_.startMainControllers();
                    break;
                default :
                    ROS_INFO("You have a big problem in your switch!!!");
                    controller_manager_.stopMainControllers();
                    break;
            }
        }
        setOutput<std::string>("mode", mode);
        return BT::NodeStatus::SUCCESS;
    }

private:
    void init() {
        controllerUpdate();
    }

//bool IsAutoMode::CalibrationServer() {
//    ros::service::waitForService("/controllers/gimbal_calibration_controller/is_calibrated");
//    control_msgs::QueryCalibrationState cali_respone;
//    client_.call(cali_respone);
//    auto calibrated_flag = cali_respone.response.is_calibrated;
//    return calibrated_flag;
//}

    void controllerCalibrate() {
        controller_manager_.startMainControllers();
        gimbal_calibration_->reset();
        shooter_calibration_->reset();
        gimbal_calibration_->stopController();
        shooter_calibration_->stopController();
//    double_barrel_cmd_sender_->init();
    }

    void controllerUpdate() {
        ros::Time time = ros::Time::now();
        gimbal_calibration_->update(time);
        shooter_calibration_->update(time);
        controller_manager_.update();
    }

    void checkRcState(const ros::Time &time) {
        if(check_rc_state_ && (time - dbus_.stamp).toSec() > 0.3){
            ROS_INFO("Remote controller OFF");
            check_rc_state_ = false;
        }
        if(!check_rc_state_ && (time - dbus_.stamp).toSec() < 0.3){
            ROS_INFO("Remote controller ON");
            check_rc_state_ = true;
        }
    }

    void dbusCallback(const rm_msgs::DbusData::ConstPtr& data) { dbus_ = *data; }
    rm_common::CalibrationQueue *gimbal_calibration_{}, *shooter_calibration_{};
    rm_common::ControllerManager controller_manager_;
//    rm_common::DoubleBarrelCommandSender* double_barrel_cmd_sender_;
    rm_msgs::DbusData dbus_;
    ros::Subscriber dbus_sub_;
    std::string mode;
    bool check_rc_state_{false};
};
}// behavior_tree
#endif //SRC_CHOOSEMODE_H
