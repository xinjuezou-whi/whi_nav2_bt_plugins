/******************************************************************
pose registration behavior tree action plugin under ROS 2

Features:
- action of pose registration
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2025-08-12: Initial version
2025-xx-xx: xxx
******************************************************************/
#pragma once
#include "whi_interfaces/action/pose_registration.hpp"

#include <nav2_behavior_tree/bt_action_node.hpp>

#include <string>

namespace whi_nav2_bt_plugins
{
	class PoseRegistrationAction : public nav2_behavior_tree::BtActionNode<whi_interfaces::action::PoseRegistration>
	{
		using Action = whi_interfaces::action::PoseRegistration;
		using ActionResult = Action::Result;

	public:
		PoseRegistrationAction(const std::string& XmlTagName, const std::string& ActionName,
			const BT::NodeConfiguration& Conf);

		// Function to perform some user-defined operation on tick
		void on_tick() override;

		// Function to perform some user-defined operation after a timeout
		// waiting for a result that hasn't been received yet
		void on_wait_for_result(std::shared_ptr<const Action::Feedback> Feedback) override;

		static BT::PortsList providedPorts()
		{
			return providedBasicPorts(
				{
					BT::InputPort<std::string>("controller_id", ""),
					BT::InputPort<geometry_msgs::msg::PoseStamped>("pose", "Pose to registrate")
				});
		}
	};

} // namespace whi_nav2_bt_plugins
