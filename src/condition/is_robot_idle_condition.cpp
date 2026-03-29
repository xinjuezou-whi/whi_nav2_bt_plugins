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
		  state_topic_("whi_state")
	{
		/// node version and copyright announcement
		std::cout << "\nWHI is robot idle bt node VERSION 00.01.2" << std::endl;
		std::cout << "Copyright © 2026-2027 Wheel Hub Intelligent Co.,Ltd. All rights reserved\n" << std::endl;

		getInput("state_topic", state_topic_);
		std::string deviceIds;
		getInput("device_ids", deviceIds);
		auto idParts = BT::splitString(deviceIds, ';');
		for (const auto& it : idParts)
		{
			device_ids_.emplace_back(it);  // string_view → string
			idles_map_[device_ids_.back()] = true;  // initialize as idle
		}
		std::string services;
		getInput("state_services", services);
		auto serviceParts = BT::splitString(services, ';');
		for (const auto& it : serviceParts)
		{
			state_services_.emplace_back(it);  // string_view → string
		}
		getInput("battery_state_service", battery_state_service_);

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

		// services
		for (const auto& service : state_services_)
		{
			state_clients_.emplace_back(node->create_client<whi_interfaces::srv::WhiSrvBinaryState>(service));
		}

		// battery state service
		battery_state_client_ = node->create_client<whi_interfaces::srv::WhiSrvBatteryState>(battery_state_service_);
	}

	BT::NodeStatus IsRobotIdleCondition::tick()
	{
		callback_group_executor_.spin_some();

		// message check
		for (const auto& it : idles_map_)
		{
			if (!it.second)
			{
				return BT::NodeStatus::FAILURE;
			}

		}
		// services check
		for (const auto& client : state_clients_)
		{
			auto request = std::make_shared<whi_interfaces::srv::WhiSrvBinaryState::Request>();
			auto resultFuture = client->async_send_request(request);
			if (rclcpp::spin_until_future_complete(config().blackboard->get<rclcpp::Node::SharedPtr>("node"), resultFuture) ==
				rclcpp::FutureReturnCode::SUCCESS)
			{
				auto result = resultFuture.get();
				if (!result->state)
				{
					return BT::NodeStatus::FAILURE;
				}
			}
			else
			{
				RCLCPP_ERROR(config().blackboard->get<rclcpp::Node::SharedPtr>("node")->get_logger(),
					"Failed to call service %s", client->get_service_name());
				return BT::NodeStatus::FAILURE;
			}
		}
		// battery state check
		if (battery_state_client_)
		{
			auto request = std::make_shared<whi_interfaces::srv::WhiSrvBatteryState::Request>();
			auto resultFuture = battery_state_client_->async_send_request(request);
			if (rclcpp::spin_until_future_complete(config().blackboard->get<rclcpp::Node::SharedPtr>("node"), resultFuture) ==
				rclcpp::FutureReturnCode::SUCCESS)
			{
				auto result = resultFuture.get();
				if (!result->result || result->state.state == whi_interfaces::msg::WhiBattery::STA_CHARGING)
				{
					return BT::NodeStatus::FAILURE;
				}	
			}
			else
			{
				RCLCPP_ERROR(config().blackboard->get<rclcpp::Node::SharedPtr>("node")->get_logger(),
					"Failed to call service %s", battery_state_service_.c_str());
				return BT::NodeStatus::FAILURE;
			}
		}
		return BT::NodeStatus::SUCCESS;
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
						if (it.value != "operating" && it.value != "estopped" &&
							it.value != "critical_collision" && it.value != "fault" &&
							it.value != "charging")
						{
							idles_map_.at(id) = true;
						}
						else
						{
							idles_map_.at(id) = false;
						}
						break;
					}
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
