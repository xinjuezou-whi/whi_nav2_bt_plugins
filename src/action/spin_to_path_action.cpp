/******************************************************************
spin to align with path behavior tree action plugin under ROS 2

Features:
- spin to align the direction of planned path
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_nav2_bt_plugins/action/spin_to_path_action.hpp"

#include <memory>
#include <string>

namespace whi_nav2_bt_plugins
{
	SpinToPathAction::SpinToPathAction(const std::string& XmlTagName, const std::string& ActionName,
			const BT::NodeConfiguration& Conf)
		: nav2_behavior_tree::BtActionNode<whi_interfaces::action::SpinToPath>(XmlTagName, ActionName, Conf)
	{
		/// node version and copyright announcement
		std::cout << "\nWHI spin to path bt node VERSION 00.01.4" << std::endl;
		std::cout << "Copyright © 2025-2026 Wheel Hub Intelligent Co.,Ltd. All rights reserved\n" << std::endl;
	}

	void SpinToPathAction::on_tick()
	{
		getInput("path", goal_.path);
		getInput("lookahead_distance", goal_.lookahead_distance);
	}
} // namespace whi_nav2_bt_plugins

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
	BT::NodeBuilder builder =
		[](const std::string& Name, const BT::NodeConfiguration& Config)
	{
		return std::make_unique<whi_nav2_bt_plugins::SpinToPathAction>(
			Name, "spin_to_path", Config); // action name
	};

	factory.registerBuilder<whi_nav2_bt_plugins::SpinToPathAction>(
		"SpinToPath", builder); // name of bt action
}
