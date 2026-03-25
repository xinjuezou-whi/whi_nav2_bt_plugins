/******************************************************************
is robot idle behavior tree condition plugin under ROS 2

Features:
- check the idle state of robot
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_nav2_bt_plugins/condition/is_robot_idle_condition.hpp"

namespace whi_nav2_bt_plugins
{
	IsRobotIdleCondition::IsRobotIdleCondition(const std::string &XmlTagName, const BT::NodeConfiguration &Conf)
		: BT::ConditionNode(XmlTagName, Conf),
		  state_topic_("whi_state"), is_robot_idle_(false)
	{
		/// node version and copyright announcement
		std::cout << "\nWHI is robot idle bt node VERSION 00.01.1" << std::endl;
		std::cout << "Copyright © 2026-2027 Wheel Hub Intelligent Co.,Ltd. All rights reserved\n" << std::endl;

		getInput("state_topic", state_topic_);
		std::string deviceIds;
		getInput("device_ids", deviceIds);
		auto parts = BT::splitString(deviceIds, ';');
		for (const auto& it : parts)
		{
			device_ids_.emplace_back(it);  // string_view → string
		}

		auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
		callback_group_ = node->create_callback_group(
			rclcpp::CallbackGroupType::MutuallyExclusive, false);
		callback_group_executor_.add_callback_group(callback_group_, node->get_node_base_interface());

		rclcpp::SubscriptionOptions subOption;
		subOption.callback_group = callback_group_;
		state_sub_ = node->create_subscription<whi_interfaces::msg::WhiState>(
			state_topic_, rclcpp::SystemDefaultsQoS(),
			std::bind(&IsRobotIdleCondition::stateCallback, this, std::placeholders::_1),
			subOption);
		// leave for extending subscriptions
		// xxx_sub_ = node->create_subscription<sensor_msgs::msg::xxx>(
		// 	xxx_topic_, rclcpp::SystemDefaultsQoS(),
		// 	std::bind(&IsRobotIdleCondition::xxxCallback, this, std::placeholders::_1),
		// 	subOption);
	}

	BT::NodeStatus IsRobotIdleCondition::tick()
	{
		callback_group_executor_.spin_some();
		if (is_robot_idle_)
		{
			return BT::NodeStatus::SUCCESS;
		}
		return BT::NodeStatus::FAILURE;
	}

	void IsRobotIdleCondition::stateCallback(whi_interfaces::msg::WhiState::SharedPtr Msg)
	{
		for (const auto& id : device_ids_)
		{
			if (id == Msg->hardware_id)
			{
				for (const auto& it : Msg->values)
				{
					if (it.key == "state")
					{
						if (it.value == "standby" || it.value == "running" ||
							it.value == "turning_left" || it.value == "turning_right")
						{
							is_robot_idle_ = true;
						}
						else
						{
							is_robot_idle_ = false;
						}
						break;
					}
				}

				if (!is_robot_idle_)
				{
					break;
				}
			}
		}
	}

} // namespace whi_nav2_bt_plugins

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
	factory.registerNodeType<whi_nav2_bt_plugins::IsRobotIdleCondition>("IsRobotIdle");
}
