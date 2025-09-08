/******************************************************************
spin to align with path behavior tree action plugin under ROS 2

Features:
- spin to align the direction of planned path
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2025-09-06: Initial version
2025-xx-xx: xxx
******************************************************************/
#pragma once
#include "whi_interfaces/action/spin_to_path.hpp"

#include <nav2_behavior_tree/bt_action_node.hpp>

#include <string>

namespace whi_nav2_bt_plugins
{
	class SpinToPathAction : public nav2_behavior_tree::BtActionNode<whi_interfaces::action::SpinToPath>
	{
	public:
		SpinToPathAction(const std::string& XmlTagName, const std::string& ActionName,
			const BT::NodeConfiguration& Conf);

		void on_tick() override;

		void on_wait_for_result() override;

		static BT::PortsList providedPorts()
		{
			return providedBasicPorts(
				{
					BT::InputPort<std::string>("controller_id", ""),
					BT::InputPort<nav_msgs::msg::Path>("path", "Planned path"),
					BT::InputPort<double>("lookahead_distance", 0.5, "lookahead distance")
				});
		}
	};
} // namespace whi_nav2_bt_plugins
