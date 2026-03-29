/******************************************************************
is battery in charging and enough for task tree condition plugin under ROS 2

Features:
- check the battery charge level
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2026-03-29: Initial version
2026-xx-xx: xxx
******************************************************************/
#pragma once
#include <string>
#include <memory>
#include <mutex>

#include <whi_interfaces/msg/whi_state.hpp>
#include <whi_interfaces/srv/whi_srv_battery_state.hpp>

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/condition_node.h>

namespace whi_nav2_bt_plugins
{
	/**
	 * @brief A BT::ConditionNode that listens to a battery topic and
	 * returns SUCCESS when battery is charging and FAILURE otherwise
	 */
	class IsChargingEnoughCondition : public BT::ConditionNode
	{
	public:
		/**
		 * @brief A constructor for whi_nav2_bt_plugins::IsRobotIdleCondition
		 * @param XmlTagName Name for the XML tag for this node
		 * @param Conf BT node configuration
		 */
		IsChargingEnoughCondition(const std::string &XmlTagName, const BT::NodeConfiguration &Conf);

		IsChargingEnoughCondition() = delete;

		/**
		 * @brief The main override required by a BT action
		 * @return BT::NodeStatus Status of tick execution
		 */
		BT::NodeStatus tick() override;

		/**
		 * @brief Creates list of BT ports
		 * @return BT::PortsList Containing node-specific ports
		 */
		static BT::PortsList providedPorts()
		{
			return {
				BT::InputPort<std::string>("state_service", std::string("battery_state"), "State service"),
			};
		}

	private:
		rclcpp::Client<whi_interfaces::srv::WhiSrvBatteryState>::SharedPtr battery_state_client_{ nullptr };
		std::string state_service_{ "battery_state"};
	};

} // namespace whi_nav2_bt_plugins
