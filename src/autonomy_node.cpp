#include "autonomy_node.hpp"

using namespace std::chrono_literals;
const std::string xml_dir = ament_index_cpp::get_package_share_directory("rom2109_autonomy") + "bt_xml";

AutoNode::AutoNode(const std::string &node_name) : Node(node_name)
{
    this->declare_parameter("location_file", "none");
    RCLCPP_INFO(get_logger(), "Init Done");
}

void AutoNode::setup()
{
    // Initial BT setup
    create_behavior_tree();
    const auto timer_period = 500ms;
    timer_ = this->create_wall_timer(timer_period, std::bind(&AutoNode::update_behavior_tree, this));
}

void AutoNode::create_behavior_tree()
{
    // Create bt
    BT::BehaviorTreeFactory factory;

    // factory.registerNodeType
    // lambda function
    // [=] capture external variable form share_from_this()
    BT::NodeBuilder builder = [=](const std::string &name, const BT::NodeConfiguration &config)
    {
        return std::make_unique<GoToPose>(name, config, shared_from_this());
    }; 
    factory.registerBuilder<GoToPose>("GoToPose", builder);

    tree_ = factory.createTreeFromFile(xml_dir + "/tree.xml");
    RCLCPP_INFO(get_logger(), "3");
}

void AutoNode::update_behavior_tree()
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