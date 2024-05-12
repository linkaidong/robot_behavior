//
// Created by yawara on 23-12-1.
//

#ifndef SRC_NAVIGATION_H
#define SRC_NAVIGATION_H

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include "ros/ros.h"
#include <rm_msgs/TrackData.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <rm_common/decision/command_sender.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <realtime_tools/realtime_publisher.h>

namespace behavior_tree{
class Navigation : public BT::SyncActionNode {
public:
    Navigation(const std::string &name, const BT::NodeConfiguration &config, ros::NodeHandle &root_nh,
               const ros::NodeHandle &decision_nh)
               : BT::SyncActionNode(name, config) {
        mbf_client_ =
                std::make_unique<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>>("move_base", true);
        game_robot_status_sub_ = root_nh.subscribe<rm_msgs::GameRobotStatus>("/rm_referee/game_robot_status", 10, &Navigation::gameRobotStatusCallback, this);
        track_sub_ = root_nh.subscribe<rm_msgs::TrackData>("/track", 1, &Navigation::detectionCallback, this);
        timer_ = root_nh.createWallTimer(ros::WallDuration(5.0), &Navigation::timerCallback, this, false);

        ros::NodeHandle patrol_nh(decision_nh, "patrol");
        ros::NodeHandle chassis_nh(decision_nh, "chassis");

        chassis_command_sender_ = new rm_common::ChassisCommandSender(chassis_nh);

        XmlRpc::XmlRpcValue patrol_list;
        patrol_nh.getParam("goal_list", patrol_list);
        patrol_nh.getParam("search_list", search_list);
        patrol_nh.getParam("search_radius", search_radius);
        for (int i = 0; i < patrol_list.size(); ++i) {
            ROS_ASSERT(patrol_list[i].getType() == XmlRpc::XmlRpcValue::TypeArray);
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header.frame_id = "map";

            ROS_ASSERT(patrol_list[i][0].getType() == XmlRpc::XmlRpcValue::TypeDouble and
                       patrol_list[i][1].getType() == XmlRpc::XmlRpcValue::TypeDouble and
                       patrol_list[i][2].getType() == XmlRpc::XmlRpcValue::TypeDouble);

            pose_stamped.pose.position.x = static_cast<double>(patrol_list[i][0]);
            pose_stamped.pose.position.y = static_cast<double>(patrol_list[i][1]);
            tf2::Quaternion quat;
            quat.setRPY(0.0, 0.0, static_cast<double>(patrol_list[i][2]));
            pose_stamped.pose.orientation = toMsg(quat);
            patrol_list_.push_back(pose_stamped);
        }
    }

    static BT::PortsList providedPorts() {
        BT::PortsList ports_list;
        return ports_list;
    }

    BT::NodeStatus tick() override {
        Chassis_init();
        MoveChassisRAW();
        return BT::NodeStatus::SUCCESS;
    }

private:
    void detectionCallback(const rm_msgs::TrackData::ConstPtr& msg) { track_buffer_ = *msg; }
    void gameRobotStatusCallback(const rm_msgs::GameRobotStatus::ConstPtr& data) { game_robot_status_ = *data; }

    void timerCallback(const ros::WallTimerEvent&) {
        rand_index++;
        if(rand_index >= rand_index_size){
            rand_index = 0;
        }
        goal_reached = true;
    }

    void Chassis_init() {
        ros::Time time = ros::Time::now();
        chassis_command_sender_->setMode(rm_msgs::ChassisCmd::RAW);
        chassis_command_sender_->getMsg()->command_source_frame = "base_link";

        chassis_command_sender_->power_limit_->updateState(rm_common::PowerLimit::NORMAL);
        chassis_command_sender_->updateGameRobotStatus(game_robot_status_);
        chassis_command_sender_->sendChassisCommand(time, false);
    }

    void MoveChassisRAW() {
        if(track_buffer_.id != 0) { // vision success
//            track_buffer_.position ;
            searchNavigablePoints();

        }else {

        }

        if(goal_reached) {
            move_base_msgs::MoveBaseGoal mbf_goal;
            mbf_goal.target_pose.header.stamp = ros::Time::now();
            mbf_goal.target_pose = patrol_list_[rand_index];
            mbf_client_->sendGoal(mbf_goal);
            goal_reached = false;
        }

        // Ensure that the execution time of the callback function is shorter than the timer cycle
        timer_.start();
        if(mbf_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            timer_.stop();
            ROS_INFO("rand_index is %d",rand_index);
            ROS_INFO("The Goal achieved success !!!");
            rand_index++;
            if(rand_index >= rand_index_size){
                rand_index = 0;
            }
            goal_reached = true;
        }
    }

    void searchNavigablePoints() {
        for (int i = 0; i < search_list.size(); ++i) {
            geometry_msgs::PoseStamped pose_search;
            pose_search.header.frame_id = "map";
            pose_search.pose.position.x = static_cast<double>(search_list[i][0]) * search_radius;
            pose_search.pose.position.y = static_cast<double>(search_list[i][1]) * search_radius;

            tf2::Quaternion quat;
            quat.setRPY(0.0, 0.0, 1.57);
            pose_search.pose.orientation = toMsg(quat);
            search_list_.push_back(pose_search);
        }
    }

    XmlRpc::XmlRpcValue search_list;
    std::vector<geometry_msgs::PoseStamped> search_list_;
    std::vector<geometry_msgs::PoseStamped> patrol_list_;
    std::unique_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>> mbf_client_;

    rm_common::ChassisCommandSender* chassis_command_sender_;
    rm_msgs::GameRobotStatus game_robot_status_;
    rm_msgs::TrackData track_buffer_;

    ros::Subscriber game_robot_status_sub_;
    ros::Subscriber track_sub_;
    ros::WallTimer timer_;

    int rand_index = 0;
    int rand_index_size = 3;
    bool goal_reached{true};
    float search_radius{0.0};
};
}// behavior_tree

#endif //SRC_NAVIGATION_H
