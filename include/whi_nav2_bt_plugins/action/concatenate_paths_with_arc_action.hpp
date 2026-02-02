/******************************************************************
concatenate paths with arc behavior tree action plugin under ROS 2

Features:
- find the tangent arc between paths
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2026-02-02: Initial version
2025-xx-xx: xxx
******************************************************************/
#pragma once
#include <nav_msgs/msg/path.hpp>
#include <nav2_behavior_tree/bt_action_node.hpp>

#include <string>

namespace whi_nav2_bt_plugins
{
	class ConcatenatePathsWithArc : public BT::ActionNodeBase
	{
	public:
		ConcatenatePathsWithArc(const std::string& XmlTagName, const BT::NodeConfiguration& Conf);

		static BT::PortsList providedPorts()
		{
            return {
                BT::InputPort<nav_msgs::msg::Path>("input_path1", "Input Path 1 to cancatenate"),
                BT::InputPort<nav_msgs::msg::Path>("input_path2", "Input Path 2 to cancatenate"),
                BT::InputPort<double>("density", 0.05, "Density of arc, should equal to the one in route"),
                BT::InputPort<double>("arc_radius", 1.0, "Radius of tangent arc"),
                BT::OutputPort<nav_msgs::msg::Path>("output_path", "Paths concatenated"),
            };
		}

    private:
        void halt() override {}
        BT::NodeStatus tick() override;
	};
} // namespace whi_nav2_bt_plugins
