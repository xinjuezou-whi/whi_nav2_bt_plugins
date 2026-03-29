/******************************************************************
is robot idle behavior tree condition plugin under ROS 2

Features:
- check the idle state of robot
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2026-03-25: Initial version
2026-xx-xx: xxx
******************************************************************/
#pragma once
#include <string>
#include <memory>
#include <mutex>

#include <whi_interfaces/msg/whi_state.hpp>
#include <whi_interfaces/srv/whi_srv_binary_state.hpp>
#include <whi_interfaces/srv/whi_srv_battery_state.hpp>

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/condition_node.h>

namespace whi_nav2_bt_plugins
{
	/**
	 * @brief A BT::ConditionNode that listens to a battery topic and
	 * returns SUCCESS when battery is charging and FAILURE otherwise
	 */
	class IsRobotIdleCondition : public BT::ConditionNode
	{
	public:
		/**
		 * @brief A constructor for whi_nav2_bt_plugins::IsRobotIdleCondition
		 * @param XmlTagName Name for the XML tag for this node
		 * @param Conf BT node configuration
		 */
		IsRobotIdleCondition(
			const std::string &XmlTagName,
			const BT::NodeConfiguration &Conf);

		IsRobotIdleCondition() = delete;

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
				BT::InputPort<std::string>("state_topic", std::string("whi_state"), "State topic"),
				BT::InputPort<std::string>("device_ids", "Array of device ids"),
				BT::InputPort<std::string>("state_services", "Array of state services"),
				BT::InputPort<std::string>("battery_state_service", std::string("battery_state"), "Battery state service"),
			};
		}

	private:
		/**
		 * @brief Callback function for battery topic
		 * @param msg Shared pointer to sensor_msgs::msg::BatteryState message
		 */
		void stateCallback(whi_interfaces::msg::WhiState::SharedPtr Msg);

		rclcpp::CallbackGroup::SharedPtr callback_group_;
		rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
		rclcpp::Subscription<whi_interfaces::msg::WhiState>::SharedPtr state_sub_;
		std::string state_topic_;
		std::vector<std::string> device_ids_;
		std::map<std::string, bool> idles_map_;
		std::vector<std::string> state_services_;
		std::string battery_state_service_;
		std::vector<rclcpp::Client<whi_interfaces::srv::WhiSrvBinaryState>::SharedPtr> state_clients_;
		rclcpp::Client<whi_interfaces::srv::WhiSrvBatteryState>::SharedPtr battery_state_client_{ nullptr };
	};

} // namespace whi_nav2_bt_plugins
