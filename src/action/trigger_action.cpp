/******************************************************************
trigger behavior tree action plugin under ROS 2

Features:
- empty and bool trigger
- xxx

Written by Tanzeel, xxx@yyy.com
Refactored by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_nav2_bt_plugins/action/trigger_action.hpp"

namespace whi_nav2_bt_plugins
{
    Trigger::Trigger(const std::string& XmlTagName, const BT::NodeConfiguration& Config)
        : BT::SyncActionNode(XmlTagName, Config)
        , node_(rclcpp::Node::make_shared("trigger_bt"))
    {
        /// node version and copyright announcement
        std::cout << "\nWHI trigger bt node VERSION 00.01.1" << std::endl;
        std::cout << "Copyright © 2026-2027 Wheel Hub Intelligent Co.,Ltd. All rights reserved\n" << std::endl;
    }

    BT::NodeStatus Trigger::tick()
    {
        std::string topic("trigger");
        if (!getInput("trigger_topic", topic))
        {
            RCLCPP_WARN(node_->get_logger(), "Failed to get input port 'trigger_topic', using default trigger");
        }

        std::string type("empty");
        if (!getInput("trigger_type", type))
        {
            RCLCPP_WARN(node_->get_logger(), "Failed to get input port 'trigger_type', using default empty");
        }

        if (type == "empty")
        {
            pub_empty_ = node_->create_publisher<std_msgs::msg::Empty>(topic, 10);
            std_msgs::msg::Empty msg;
            pub_empty_->publish(msg);

            RCLCPP_INFO(node_->get_logger(), "Empty trigger published");
        }
        else
        {
            bool value = true; // default
            if (!getInput("trigger_value", value))
            {
                RCLCPP_WARN(node_->get_logger(), "Failed to get input port 'trigger_value', using default false");
            }

            pub_bool_ = node_->create_publisher<std_msgs::msg::Bool>(topic, 10);
            std_msgs::msg::Bool msg;
            msg.data = value;
            pub_bool_->publish(msg);

            RCLCPP_INFO(node_->get_logger(), "Trigger published: %s", value ? "true" : "false");
        }

        return BT::NodeStatus::SUCCESS;
    }
} // namespace whi_nav2_bt_plugins

// Register node
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<whi_nav2_bt_plugins::Trigger>("Trigger");
}
