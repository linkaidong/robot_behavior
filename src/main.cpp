//
// Created by yawara on 23-4-17.
//
#include "patrol.h"
#include "is_enemy_visible.h"
#include "track_to_attack.h"
#include "choosemode.h"
#include "manual.h"
#include "navigation.h"

static const char* xml_text = R"(
<root main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
        <Sequence>
            <ChooseMode mode="{mode}"/>
            <Switch3 variable="{mode}"  case_1="manual" case_2="shutdown" case_3="auto">
                <Manual/>
                <ShutDown/>
                <SubTree ID="Auto"/>
                <ShutDown/>
            </Switch3>
        </Sequence>
    </BehaviorTree>

    <BehaviorTree ID="Auto">
        <Sequence>
            <Navigation/>
            <IfThenElse>
                <IsEnemyVisible/>
                <Sequence>
                    <TrackToAttack/>
                </Sequence>
                <Patrol/>
            </IfThenElse>
        </Sequence>   
    </BehaviorTree>

</root>
)";

int main(int argc, char** argv) {
    ros::init(argc, argv, "run_bt");
    ros::NodeHandle root_nh;
    ros::NodeHandle decision_nh("~");
    BT::BehaviorTreeFactory factory_;
    BT::NodeBuilder builder_;

    builder_ = [&root_nh, &decision_nh](const std::string& name, const BT::NodeConfiguration& config) {
        return std::make_unique<behavior_tree::Patrol>(name, config, root_nh, decision_nh);
    };
    factory_.registerBuilder<behavior_tree::Patrol>("Patrol", builder_);

    builder_ = [&root_nh, &decision_nh](const std::string& name, const BT::NodeConfiguration& config) {
        return std::make_unique<behavior_tree::IsEnemyVisible>(name, config, root_nh, decision_nh);
    };
    factory_.registerBuilder<behavior_tree::IsEnemyVisible>("IsEnemyVisible", builder_);

    builder_ = [&root_nh, &decision_nh](const std::string& name, const BT::NodeConfiguration& config) {
        return std::make_unique<behavior_tree::TrackToAttack>(name, config, root_nh, decision_nh);
    };
    factory_.registerBuilder<behavior_tree::TrackToAttack>("TrackToAttack", builder_);

    builder_ = [&root_nh, &decision_nh](const std::string& name, const BT::NodeConfiguration& config) {
        return std::make_unique<behavior_tree::ChooseMode>(name, config, root_nh, decision_nh);
    };
    factory_.registerBuilder<behavior_tree::ChooseMode>("ChooseMode", builder_);

    builder_ = [&root_nh, &decision_nh](const std::string& name, const BT::NodeConfiguration& config) {
        return std::make_unique<behavior_tree::Manual>(name, config, root_nh, decision_nh);
    };
    factory_.registerBuilder<behavior_tree::Manual>("Manual", builder_);

    builder_ = [&root_nh, &decision_nh](const std::string& name, const BT::NodeConfiguration& config) {
        return std::make_unique<behavior_tree::ShutDown>(name, config, root_nh, decision_nh);
    };
    factory_.registerBuilder<behavior_tree::ShutDown>("ShutDown", builder_);

    builder_ = [&root_nh, &decision_nh](const std::string& name, const BT::NodeConfiguration& config) {
        return std::make_unique<behavior_tree::Navigation>(name, config, root_nh, decision_nh);
    };
    factory_.registerBuilder<behavior_tree::Navigation>("Navigation", builder_);

    auto tree = factory_.createTreeFromText(xml_text);
    ros::Rate loop_rate(20);

    while(ros::ok()){
        tree.tickRoot();
        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
