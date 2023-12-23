//
// Created by yawara on 23-9-2.
//

#include "choosemode.h"

namespace behavior_tree {
ShutDown::ShutDown(const std::string &name, const BT::NodeConfiguration &config, ros::NodeHandle &root_nh,
                   const ros::NodeHandle &decision_nh)
    : BT::SyncActionNode(name, config) {
    ros::NodeHandle shutdown_nh(decision_nh, "shutdown");
}

BT::PortsList ShutDown::providedPorts(){
    BT::PortsList ports_list;
    return ports_list;
}

BT::NodeStatus ShutDown::tick(){
//    if(check_rc_state_) {
//        ROS_INFO("Your State is RC_OFF");
//    } else {
//        ROS_INFO("Your State is at rm_msgs::DbusData::DOWN");
//    }
    return BT::NodeStatus::SUCCESS;
}

ChooseMode::ChooseMode(const std::string &name, const BT::NodeConfiguration &config, ros::NodeHandle &root_nh,
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

/*   sentry_init */
void ChooseMode::controllerCalibrate() {
    controller_manager_.startMainControllers();
    gimbal_calibration_->reset();
    shooter_calibration_->reset();
    gimbal_calibration_->stopController();
    shooter_calibration_->stopController();
//    double_barrel_cmd_sender_->init();
}

void ChooseMode::controllerUpdate(){
    ros::Time time = ros::Time::now();
    gimbal_calibration_->update(time);
    shooter_calibration_->update(time);
    controller_manager_.update();
}

//bool IsAutoMode::CalibrationServer() {
//    ros::service::waitForService("/controllers/gimbal_calibration_controller/is_calibrated");
//    control_msgs::QueryCalibrationState cali_respone;
//    client_.call(cali_respone);
//    auto calibrated_flag = cali_respone.response.is_calibrated;
//    return calibrated_flag;
//}

void ChooseMode::init() {
    controllerUpdate();
}
/*   sentry_init */

void ChooseMode::checkRcState(const ros::Time &time) {
    if(check_rc_state_ && (time - dbus_.stamp).toSec() > 0.3){
        ROS_INFO("Remote controller OFF");
        check_rc_state_ = false;
    }
    if(!check_rc_state_ && (time - dbus_.stamp).toSec() < 0.3){
        ROS_INFO("Remote controller ON");
        check_rc_state_ = true;
    }
}

void ChooseMode::dbusCallback(const rm_msgs::DbusData::ConstPtr& data){
    dbus_ = *data;
}

BT::PortsList ChooseMode::providedPorts(){
    return { BT::OutputPort<std::string>("mode") };
}

BT::NodeStatus ChooseMode::tick(){
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

}// behavior_tree