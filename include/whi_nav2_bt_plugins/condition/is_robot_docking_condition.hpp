/******************************************************************
is robot going to dock tree condition plugin under ROS 2

Features:
- check the docking state of robot
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2026-03-28: Initial version
2026-xx-xx: xxx
******************************************************************/
#pragma once
#include <string>
#include <memory>
#include <mutex>

#include <whi_interfaces/msg/whi_state.hpp>

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/condition_node.h>

namespace whi_nav2_bt_plugins
{
	/**
	 * @brief A BT::ConditionNode that listens to a battery topic and
	 * returns SUCCESS when battery is charging and FAILURE otherwise
	 */
	class IsRobotDockingCondition : public BT::ConditionNode
	{
	public:
		/**
		 * @brief A constructor for whi_nav2_bt_plugins::IsRobotIdleCondition
		 * @param XmlTagName Name for the XML tag for this node
		 * @param Conf BT node configuration
		 */
		IsRobotDockingCondition(const std::string &XmlTagName, const BT::NodeConfiguration &Conf);

		IsRobotDockingCondition() = delete;

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
		bool is_robot_docking_;
	};

} // namespace whi_nav2_bt_plugins
