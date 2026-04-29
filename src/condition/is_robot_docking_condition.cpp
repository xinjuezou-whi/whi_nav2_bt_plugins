/******************************************************************
is robot going to dock tree condition plugin under ROS 2

Features:
- check the docking state of robot
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_nav2_bt_plugins/condition/is_robot_docking_condition.hpp"

namespace whi_nav2_bt_plugins
{
	IsRobotDockingCondition::IsRobotDockingCondition(const std::string &XmlTagName, const BT::NodeConfiguration &Conf)
		: BT::ConditionNode(XmlTagName, Conf),
		  state_service_("battery_state")
	{
		/// node version and copyright announcement
		std::cout << "\nWHI is robot docking bt node VERSION 00.01.1" << std::endl;
		std::cout << "Copyright © 2026-2027 Wheel Hub Intelligent Co.,Ltd. All rights reserved\n" << std::endl;

		getInput("state_service", state_service_);

		auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
		battery_state_client_ = node->create_client<whi_interfaces::srv::WhiSrvBatteryState>(state_service_);
	}

	BT::NodeStatus IsRobotDockingCondition::tick()
	{
		if (battery_state_client_)
		{
			auto request = std::make_shared<whi_interfaces::srv::WhiSrvBatteryState::Request>();
			auto resultFuture = battery_state_client_->async_send_request(request);
			if (rclcpp::spin_until_future_complete(config().blackboard->get<rclcpp::Node::SharedPtr>("node"), resultFuture) ==
				rclcpp::FutureReturnCode::SUCCESS)
			{
				auto result = resultFuture.get();
				if (result->result)
				{
					if (result->state.state == whi_interfaces::msg::WhiBattery::STA_PRE_STAGING ||
						result->state.state == whi_interfaces::msg::WhiBattery::STA_DOCKING)
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
					return BT::NodeStatus::FAILURE;
				}
			}
			else
			{
				RCLCPP_ERROR(config().blackboard->get<rclcpp::Node::SharedPtr>("node")->get_logger(),
					"Failed to call service %s", state_service_.c_str());
				return BT::NodeStatus::FAILURE;
			}
		}
		else
		{
			RCLCPP_ERROR(config().blackboard->get<rclcpp::Node::SharedPtr>("node")->get_logger(),
				"Service client for %s is not initialized", state_service_.c_str());
			return BT::NodeStatus::FAILURE;
		}
	}
} // namespace whi_nav2_bt_plugins

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
	factory.registerNodeType<whi_nav2_bt_plugins::IsRobotDockingCondition>("IsRobotDocking");
}
