//
// Created by yawara on 23-9-2.
//

#include "is_enemy_visible.h"

namespace behavior_tree{
IsEnemyVisible::IsEnemyVisible(const std::string &name, const BT::NodeConfiguration &config, ros::NodeHandle &root_nh,
                               const ros::NodeHandle &decision_nh)
                                : BT::SyncActionNode(name, config){

    ros::NodeHandle is_enemy_visible_nh(decision_nh, "is_enemy_visible");
    track_sub_ = root_nh.subscribe<rm_msgs::TrackData>("/track", 1, &IsEnemyVisible::detectionCallback, this);
}

void IsEnemyVisible::detectionCallback(const rm_msgs::TrackData::ConstPtr& msg){ track_buffer_ = *msg; }

BT::PortsList IsEnemyVisible::providedPorts(){
    BT::PortsList ports_list;
    return ports_list;
}

BT::NodeStatus IsEnemyVisible::tick(){
    if(track_buffer_.id != 0){
        return BT::NodeStatus::SUCCESS;
    } else{
        return BT::NodeStatus::FAILURE;
    }
}

}// behavior_tree