/******************************************************************
synchronized trigger action behavior tree action plugin under ROS 2

Features:
- synchronized trigger by service call
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2026-03-30: Initial version
2025-xx-xx: xxx
******************************************************************/
#pragma once
#include <nav2_behavior_tree/bt_action_node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

namespace whi_nav2_bt_plugins
{
    class SyncTrigger : public BT::SyncActionNode
    {
    public:
        SyncTrigger(const std::string& XmlTagName, const BT::NodeConfiguration& Config);

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<std::string>("trigger_service", "sync_trigger", "Service name of sync trigger"),
                BT::InputPort<bool>("value", 0, "Value of trigger"),
            };
        }

        BT::NodeStatus tick() override;

    private:
        rclcpp::Node::SharedPtr node_;
		rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr trigger_client_{ nullptr };
		std::string trigger_service_{ "sync_trigger"};
        bool value_{ false };
    };
} // namespace whi_nav2_bt_plugins
