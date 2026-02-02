/******************************************************************
concatenate paths with arc behavior tree action plugin under ROS 2

Features:
- find the tangent arc between paths
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_nav2_bt_plugins/action/concatenate_paths_with_arc_action.hpp"
#include "whi_nav2_bt_plugins/corner_smoothing.hpp"

#include <memory>
#include <string>

namespace whi_nav2_bt_plugins
{
	ConcatenatePathsWithArc::ConcatenatePathsWithArc(const std::string& XmlTagName, const BT::NodeConfiguration& Conf)
		: BT::ActionNodeBase(XmlTagName, Conf)
	{
		/// node version and copyright announcement
		std::cout << "\nWHI concatenate paths with arc bt node VERSION 00.01.1" << std::endl;
		std::cout << "Copyright © 2026-2027 Wheel Hub Intelligent Co.,Ltd. All rights reserved\n" << std::endl;
	}

    inline BT::NodeStatus ConcatenatePathsWithArc::tick()
    {
        setStatus(BT::NodeStatus::RUNNING);

        nav_msgs::msg::Path path1, path2;
        getInput("input_path1", path1);
        getInput("input_path2", path2);

        if (path1.poses.empty() && path2.poses.empty())
        {
            RCLCPP_ERROR(config().blackboard->get<rclcpp::Node::SharedPtr>("node")->get_logger(),
                "No input paths provided to concatenate. Both paths are empty");

            return BT::NodeStatus::FAILURE;
        }

        nav_msgs::msg::Path outputPath;
        outputPath = path1;
        if (path1.header != std_msgs::msg::Header())
        {
            outputPath.header = path1.header;
        }
        else if (path2.header != std_msgs::msg::Header())
        {
            outputPath.header = path2.header;
        }

        double radius = 1.0;
        getInput("arc_radius", radius);

        CornerArc::Coordinates start;
        CornerArc::Coordinates arcCenter(outputPath.poses.back().pose.position.x, outputPath.poses.back().pose.position.y);
        for (int i = outputPath.poses.size() - 2; i >= 0; --i)
        {
            if (hypotf(outputPath.poses[i].pose.position.x - arcCenter.x, outputPath.poses[i].pose.position.y - arcCenter.y) > radius)
            {
                start.x = outputPath.poses[i].pose.position.x;
                start.y = outputPath.poses[i].pose.position.y;

                outputPath.poses.erase(outputPath.poses.begin() + i, outputPath.poses.end());

                break;
            }
        }
        if (outputPath.poses.size() == path1.poses.size())
        {
            // means the arc radius is larger than the path
            start.x = outputPath.poses.front().pose.position.x;
            start.y = outputPath.poses.front().pose.position.y;
            radius = hypotf(start.x - arcCenter.x, start.y - arcCenter.y);
        }

        CornerArc::Coordinates end(NAN, NAN);
        for (std::size_t i = 1; i < path2.poses.size(); ++i)
        {
            if (hypotf(path2.poses[i].pose.position.x - path2.poses.front().pose.position.x,
                path2.poses[i].pose.position.y - path2.poses.front().pose.position.y) > radius)
            {
                end.x = path2.poses[i].pose.position.x;
                end.y = path2.poses[i].pose.position.y;

                path2.poses.erase(path2.poses.begin(), path2.poses.begin() + i);

                break;
            }
        }
        if (std::isnan(end.x) || std::isnan(end.y))
        {
            // means the arc radius is larger than the path
            radius = hypotf(path2.poses.front().pose.position.x - path2.poses.back().pose.position.x,
                path2.poses.front().pose.position.y - path2.poses.back().pose.position.y);
            end.x = path2.poses.back().pose.position.x;
            end.y = path2.poses.back().pose.position.y;
        }

        double density = 0.05;
        getInput("density", density);

        nav_msgs::msg::Path arcPath;
        CornerArc arc(start, arcCenter, end, radius);
        if (arc.isCornerValid())
        {
            arc.interpolateArc(density / radius, arcPath.poses);

            // Set path poses orientations for each point
            for (std::size_t i = 0; i < arcPath.poses.size() - 1; ++i)
            {
                const auto& pose = arcPath.poses[i];
                const auto& nextPose = arcPath.poses[i + 1];
                const double dx = nextPose.pose.position.x - pose.pose.position.x;
                const double dy = nextPose.pose.position.y - pose.pose.position.y;
                const double yaw = atan2(dy, dx);
                arcPath.poses[i].pose.orientation = CornerArc::orientationAroundZAxis(yaw);
            }

            outputPath.poses.insert(outputPath.poses.end(), arcPath.poses.begin(), arcPath.poses.end());
        }
        
        outputPath.poses.insert(outputPath.poses.end(), path2.poses.begin(), path2.poses.end());

        setOutput("output_path", outputPath);
        return BT::NodeStatus::SUCCESS;
    }
} // namespace whi_nav2_bt_plugins

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<whi_nav2_bt_plugins::ConcatenatePathsWithArc>("ConcatenatePathsWithArc");
}
