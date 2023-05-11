#include "nav_bt.h"

GoToPose::GoToPose(const std::string &name,
                   const BT::NodeConfiguration &config,
                   rclcpp::Node::SharePtr node_ptr)
    : BT::StatefulActionNode(name, config), node_ptr_(node_ptr)
{
    action_client_ptr = rclcpp_action::create_client<NavigationToPose>(node_ptr_, "/navigate_to_pose");
    done_flag_ = false;
}

BT::PortsList GoToPose::providePorts()
{
    return (BT::InputPort<std::string>("loc"));
}

BT::NodeStatus GoToPose::onStart()
{
}

BT::NodeStatus GoToPose::onRunning()
{
}

void nav_to_pose_callback(const GoalHandleNav::WrapperResult &result)
{
}
