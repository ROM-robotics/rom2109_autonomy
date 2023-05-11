#include "auto_node.hpp"

using namespace std::chrono_literals;
const std::string bt_xml_dir = ament_index_cpp::get_package_share_directory("rom2109_autonomy") + "bt_xml";
AutoNode::AutoNode(const std::string &node_name) : Node(node_name)
{
    RCLCPP_INFO(get_logger(), "Init Done");
}

AutoNode::setup()
{
    create_behavior_tree();
    const auto timer_period = 500ms;
    timer_ = this->create_wall_timer(timer_period, std::bind(&AutoNode::update_behavior_tree, this));
}

AutoNode::create_behavior_tree()
{
    // Create bt
    BT::BehaviorTreeFactory factory;

    // factory.registerNodeType
    // lambda function
    // [=] capture external variable form share_from_this()
    BT::NodeBuilder builder = [=](const std::string &name, const BT::NodeConfiguration &config)
    {
        return std::make_unique<GoToPose>(name, config, shared_from_this());
    } factory.registerBuilder < GoToPose("GoToPose", builder);

    tree_ = factory.createTreeFromFile(bt_xml_dir + "/tree.xml");
}

AutoNode::update_behavior_tree()
{
    BT::NodeStatus tree_status = tree_.tickRoot();

    if (tree_status == BT::NodeStatus::RUNNING)
    {
        return;
    }
    else if (tree_status == BT::NodeStatus::SUCCESS)
    {
        RCLCPP_INFO(this->get_logger(), "Finished Navigation");
    }
    else if (tree_status == BT::NodeStatus::FAILURE)
    {
        RCLCPP_INFO(this->get_logger(), "Finished Failed");
        timer_->cancel();
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AutoNode>("autonomy_node");
    node->setup();

    rclcpp::spin(node);
    rclcpp::shutdown();
}