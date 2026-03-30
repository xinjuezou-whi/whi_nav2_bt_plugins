/******************************************************************
io operation behavior tree action plugin under ROS 2

Features:
- read and write io ports
- xxx

Written by Tanzeel, xxx@yyy.com
Refactored by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_nav2_bt_plugins/action/io_request_action.hpp"

namespace whi_nav2_bt_plugins
{
    IoRequest::IoRequest(const std::string& XmlTagName, const BT::NodeConfiguration& Config)
        : BT::SyncActionNode(XmlTagName, Config)
        , node_(rclcpp::Node::make_shared("io_bt"))
    {
        /// node version and copyright announcement
        std::cout << "\nWHI IO request bt node VERSION 00.01.1" << std::endl;
        std::cout << "Copyright © 2026-2027 Wheel Hub Intelligent Co.,Ltd. All rights reserved\n" << std::endl;

        getInput("io_service", io_service_);
        getInput("addr", addr_);
        getInput("operation", operation_);
        getInput("value", value_);

		auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
		io_client_ = node->create_client<whi_interfaces::srv::WhiSrvIo>(io_service_);
    }

    BT::NodeStatus IoRequest::tick()
    {
		if (io_client_)
		{
			auto request = std::make_shared<whi_interfaces::srv::WhiSrvIo::Request>();
            request->io.addr = addr_;
            request->io.operation = operation_;
            request->io.level = value_;
			auto resultFuture = io_client_->async_send_request(request);
			if (rclcpp::spin_until_future_complete(config().blackboard->get<rclcpp::Node::SharedPtr>("node"), resultFuture) ==
				rclcpp::FutureReturnCode::SUCCESS)
			{
				auto result = resultFuture.get();
				if (result->result)
				{
                    if (operation_ == 0) // read operation, set output port
                    {
                        setOutput("level", result->level);
                    }
                    return BT::NodeStatus::SUCCESS;
				}
                else
                {
                    return BT::NodeStatus::FAILURE;
                }
			}
			else
			{
				RCLCPP_ERROR(config().blackboard->get<rclcpp::Node::SharedPtr>("node")->get_logger(),
					"Failed to call service %s", io_service_.c_str());
				return BT::NodeStatus::FAILURE;
			}
		}
		else
		{
			RCLCPP_ERROR(config().blackboard->get<rclcpp::Node::SharedPtr>("node")->get_logger(),
				"Service client for %s is not initialized", io_service_.c_str());
			return BT::NodeStatus::FAILURE;
		}
    }
} // namespace whi_nav2_bt_plugins

// Register node
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<whi_nav2_bt_plugins::IoRequest>("IoRequest");
}
