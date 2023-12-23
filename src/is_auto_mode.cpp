//
// Created by yawara on 23-9-2.
//

#include "is_auto_mode.h"

namespace behavior_tree {
IsAutoMode::IsAutoMode(const std::string &name, const BT::NodeConfiguration &config, ros::NodeHandle &root_nh,
                       const ros::NodeHandle &decision_nh)
                        : BT::SyncActionNode(name, config), controller_manager_(root_nh){

    ros::NodeHandle is_auto_mode_nh(decision_nh, "is_auto_mode");
    vel_2d_cmd_sender_ = new rm_common::Vel2DCommandSender(is_auto_mode_nh);

    /*   sentry_init */
    try{
        XmlRpc::XmlRpcValue gimbal_calibration_param, shooter_calibration_param;
        root_nh.getParam("gimbal_calibration", gimbal_calibration_param);
        gimbal_calibration_ = new rm_common::CalibrationQueue(gimbal_calibration_param, is_auto_mode_nh, controller_manager_);
        root_nh.getParam("shooter_calibration", shooter_calibration_param);
        shooter_calibration_ = new rm_common::CalibrationQueue(shooter_calibration_param, is_auto_mode_nh, controller_manager_);
    }catch (XmlRpc::XmlRpcException& e){
        ROS_ERROR("%s", e.getMessage().c_str());
    }
    controller_manager_.startStateControllers();
    controllerCalibrate();
    /*   sentry_init */

    dbus_sub_ = root_nh.subscribe<rm_msgs::DbusData>("/dbus_data", 10, &IsAutoMode::dbusCallback, this);

}

/*   sentry_init */
void IsAutoMode::controllerCalibrate() {
    controller_manager_.startMainControllers();
    gimbal_calibration_->reset();
    shooter_calibration_->reset();
    gimbal_calibration_->stopController();
    shooter_calibration_->stopController();
}

void IsAutoMode::controllerUpdate(){
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

void IsAutoMode::init() {
    controllerUpdate();
}
/*   sentry_init */

void IsAutoMode::dbusCallback(const rm_msgs::DbusData::ConstPtr& data){
    dbus_ = *data;
//    context_.dbusUpdate(*data);
//    chassis_cmd_sender_->updateRefereeStatus(referee_is_online_);
//    union_cmd_sender_->double_barrel_cmd_sender_->updateRefereeStatus(referee_is_online_);
}

BT::PortsList IsAutoMode::providedPorts(){
    BT::PortsList ports_list;
    return ports_list;
}

BT::NodeStatus IsAutoMode::tick(){
    init();
    if(!dbus_.s_l&&!dbus_.s_r){
        return BT::NodeStatus::SUCCESS;
    } else{
        return BT::NodeStatus::FAILURE;
    }
}

}// behavior_tree