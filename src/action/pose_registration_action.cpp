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
		std::cout << "\nWHI pose registration bt node VERSION 00.01.2" << std::endl;
		std::cout << "Copyright © 2025-2026 Wheel Hub Intelligent Co.,Ltd. All rights reserved\n" << std::endl;
	}

	void PoseRegistrationAction::on_tick()
	{
		getInput("pose", goal_.target_pose);
	}

	void PoseRegistrationAction::on_timeout()
	{
		// setOutput("error_code_id", ActionResult::TIMEOUT);
		// setOutput("error_msg", "Behavior Tree action client timed out waiting.");
	}

	BT::NodeStatus PoseRegistrationAction::on_success()
	{
		// setOutput("error_code_id", ActionResult::NONE);
		// setOutput("error_msg", "");
		return BT::NodeStatus::SUCCESS;
	}

	BT::NodeStatus PoseRegistrationAction::on_aborted()
	{
		// setOutput("error_code_id", result_.result->error_code);
		// setOutput("error_msg", result_.result->error_msg);
		return BT::NodeStatus::FAILURE;
	}

	BT::NodeStatus PoseRegistrationAction::on_cancelled()
	{
		// setOutput("error_code_id", ActionResult::NONE);
		// setOutput("error_msg", "");
		return BT::NodeStatus::SUCCESS;
	}
} // namespace whi_nav2_bt_plugins

#include "behaviortree_cpp/bt_factory.h"
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
