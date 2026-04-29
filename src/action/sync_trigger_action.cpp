/******************************************************************
io operation behavior tree action plugin under ROS 2

Features:
- read and write io ports
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_nav2_bt_plugins/action/sync_trigger_action.hpp"

namespace whi_nav2_bt_plugins
{
    SyncTrigger::SyncTrigger(const std::string& XmlTagName, const BT::NodeConfiguration& Config)
        : BT::SyncActionNode(XmlTagName, Config)
        , node_(rclcpp::Node::make_shared("sync_trigger_bt"))
    {
        /// node version and copyright announcement
        std::cout << "\nWHI Sync Trigger bt node VERSION 00.01.1" << std::endl;
        std::cout << "Copyright © 2026-2027 Wheel Hub Intelligent Co.,Ltd. All rights reserved\n" << std::endl;

        getInput("trigger_service", trigger_service_);
        getInput("value", value_);

		auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
		trigger_client_ = node->create_client<std_srvs::srv::SetBool>(trigger_service_);
    }

    BT::NodeStatus SyncTrigger::tick()
    {
		if (trigger_client_)
		{
			auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
            request->data = value_;
			auto resultFuture = trigger_client_->async_send_request(request);
			if (rclcpp::spin_until_future_complete(config().blackboard->get<rclcpp::Node::SharedPtr>("node"), resultFuture) ==
				rclcpp::FutureReturnCode::SUCCESS)
			{
				auto result = resultFuture.get();
				if (result->success)
                {
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
					"Failed to call service %s", trigger_service_.c_str());
				return BT::NodeStatus::FAILURE;
			}
		}
		else
		{
			RCLCPP_ERROR(config().blackboard->get<rclcpp::Node::SharedPtr>("node")->get_logger(),
				"Service client for %s is not initialized", trigger_service_.c_str());
			return BT::NodeStatus::FAILURE;
		}
    }
} // namespace whi_nav2_bt_plugins

// Register node
#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<whi_nav2_bt_plugins::SyncTrigger>("SyncTrigger");
}
