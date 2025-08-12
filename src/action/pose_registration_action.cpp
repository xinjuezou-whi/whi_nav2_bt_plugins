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

	PoseRegistration::PoseRegistration(const std::string& XmlTagName, const std::string& ActionName,
			const BT::NodeConfiguration& Conf)
		: nav2_behavior_tree::BtActionNode<whi_interfaces::action::PoseRegistration>(XmlTagName, ActionName, Conf)
	{
		config().blackboard->set("pose_updated", false);
	}

	void PoseRegistration::on_tick()
	{
		getInput("pose", goal_.target_pose);
		getInput("controller_id", goal_.controller_id);
	}

	void PoseRegistration::on_wait_for_result()
	{
		// Check if the goal has been updated
		if (config().blackboard->get<bool>("pose_updated"))
		{
			// Reset the flag in the blackboard
			config().blackboard->set("pose_updated", false);

			// Grab the new goal and set the flag so that we send the new goal to
			// the action server on the next loop iteration
			getInput("pose", goal_.target_pose);
			goal_updated_ = true;
		}
	}

} // namespace whi_nav2_bt_plugins

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
	BT::NodeBuilder builder =
		[](const std::string& Name, const BT::NodeConfiguration& Config)
	{
		return std::make_unique<whi_nav2_bt_plugins::PoseRegistration>(
			Name, "pose_registration", Config);
	};

	factory.registerBuilder<whi_nav2_bt_plugins::PoseRegistration>(
		"PoseRegistration", builder);
}
