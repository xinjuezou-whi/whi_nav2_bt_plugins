/******************************************************************
io operation behavior tree action plugin under ROS 2

Features:
- read and write io ports
- xxx

Written by Tanzeel, xxx@yyy.com
Refactored by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2026-03-30: Initial version
2025-xx-xx: xxx
******************************************************************/
#pragma once
#include <nav2_behavior_tree/bt_action_node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <whi_interfaces/srv/whi_srv_io.hpp>

namespace whi_nav2_bt_plugins
{
    class IoRequest : public BT::SyncActionNode
    {
    public:
        IoRequest(const std::string& XmlTagName, const BT::NodeConfiguration& Config);

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<std::string>("io_service", "modbus_io_request", "Service name of io request"),
                BT::InputPort<std::string>("addr", "0", "Address of io port"),
                BT::InputPort<std::string>("operation", "0", "Operation of IO, 0 for read, 1 for write"),
                BT::InputPort<int>("value", 0, "Value to set to IO port"),
                BT::OutputPort<int>("level", "Read value of IO port"),
            };
        }

        BT::NodeStatus tick() override;

    private:
        rclcpp::Node::SharedPtr node_;
		rclcpp::Client<whi_interfaces::srv::WhiSrvIo>::SharedPtr io_client_{ nullptr };
		std::string io_service_{ "modbus_io_request"};
        int addr_{ 0 };
        int operation_{ 0 };
        int value_{ 0 };
    };
} // namespace whi_nav2_bt_plugins
