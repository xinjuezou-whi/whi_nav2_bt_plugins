/******************************************************************
trigger behavior tree action plugin under ROS 2

Features:
- empty and bool trigger
- xxx

Written by Tanzeel, xxx@yyy.com
Refactored by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2026-03-20: Initial version
2025-xx-xx: xxx
******************************************************************/
#pragma once
#include <nav2_behavior_tree/bt_action_node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>

namespace whi_nav2_bt_plugins
{
    class Trigger : public BT::SyncActionNode
    {
    public:
        Trigger(const std::string& XmlTagName, const BT::NodeConfiguration& Config);

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<std::string>("trigger_topic", "Trigger", "Topic of trigger"),
                BT::InputPort<std::string>("trigger_type", "empty", "Type of trigger: empty or bool"),
                BT::InputPort<bool>("trigger_value", true, "Bool value to publish to trigger_topic"),
            };
        }

        BT::NodeStatus tick() override;

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_empty_{ nullptr };
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_bool_{ nullptr };
    };
} // namespace whi_nav2_bt_plugins
