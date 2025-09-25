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
#include "whi_nav2_bt_plugins/action/pose_registration_action.hpp"

#include <memory>
#include <string>

namespace whi_nav2_bt_plugins
{

	PoseRegistrationAction::PoseRegistrationAction(const std::string& XmlTagName, const std::string& ActionName,
			const BT::NodeConfiguration& Conf)
		: nav2_behavior_tree::BtActionNode<whi_interfaces::action::PoseRegistration>(XmlTagName, ActionName, Conf)
	{
		/// node version and copyright announcement
		std::cout << "\nWHI pose registration bt node VERSION 00.01.3" << std::endl;
		std::cout << "Copyright © 2025-2026 Wheel Hub Intelligent Co.,Ltd. All rights reserved\n" << std::endl;
	}

	void PoseRegistrationAction::on_tick()
	{
		getInput("pose", goal_.target_pose);
		getInput("controller_id", goal_.controller_id);
	}

	void PoseRegistrationAction::on_wait_for_result(std::shared_ptr<const Action::Feedback> /*Feedback*/)
	{
		// TODO
	}
} // namespace whi_nav2_bt_plugins

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
	BT::NodeBuilder builder =
		[](const std::string& Name, const BT::NodeConfiguration& Config)
	{
		return std::make_unique<whi_nav2_bt_plugins::PoseRegistrationAction>(
			Name, "pose_registration", Config); // action name
	};

	factory.registerBuilder<whi_nav2_bt_plugins::PoseRegistrationAction>(
		"PoseRegistration", builder); // name of bt action
}
