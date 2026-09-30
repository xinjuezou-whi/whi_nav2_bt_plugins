/******************************************************************
compute route action plugin under ROS 2

Features:
- create virtual node if the first edge is blocked
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2026-09-30: Initial version
2025-xx-xx: xxx
******************************************************************/
#pragma once
#include <string>

#include <nav2_msgs/action/compute_route.hpp>
#include <nav_msgs/msg/path.h>
#include <nav2_behavior_tree/bt_action_node.hpp>

namespace whi_nav2_bt_plugins
{

/**
 * @brief A whi_nav2_bt_plugins::BtActionNode class that wraps nav2_msgs::action::ComputeRoute
 */
class ComputeRouteAction : public nav2_behavior_tree::BtActionNode<nav2_msgs::action::ComputeRoute>
{
  using Action = nav2_msgs::action::ComputeRoute;
  using ActionResult = Action::Result;

public:
  /**
   * @brief A constructor for whi_nav2_bt_plugins::ComputeRoute
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  ComputeRouteAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Function to perform some user-defined operation on tick
   */
  void on_tick() override;

  /**
   * @brief Function to perform some user-defined operation upon successful completion of the action
   */
  BT::NodeStatus on_success() override;

  /**
   * @brief Function to perform some user-defined operation upon abortion of the action
   */
  BT::NodeStatus on_aborted() override;

  /**
   * @brief Function to perform some user-defined operation upon cancellation of the action
   */
  BT::NodeStatus on_cancelled() override;

  /**
   * @brief Function to perform work in a BT Node when the action server times out
   * Such as setting the error code ID status to timed out for action clients.
   */
  void on_timeout() override;

  /**
   * \brief Override required by the a BT action. Cancel the action and set the path output
   */
  void halt() override;

  /**
   * @brief Reset output port values on failure
   */
  void resetPorts();

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<unsigned int>("start_id", "ID of the start node"),
        BT::InputPort<unsigned int>("goal_id", "ID of the goal node"),
        BT::InputPort<geometry_msgs::msg::PoseStamped>(
          "start",
          "Start pose of the path if overriding current robot pose and using poses over IDs"),
        BT::InputPort<geometry_msgs::msg::PoseStamped>(
          "goal", "Goal pose of the path if using poses over IDs"),
        BT::InputPort<bool>(
          "use_start", false,
          "Whether to use the start pose or the robot's current pose"),
        BT::InputPort<bool>(
          "use_poses", false, "Whether to use poses or IDs for start and goal"),
        BT::OutputPort<ActionResult::_route_type>(
          "route", "The route computed by ComputeRoute node"),
        BT::OutputPort<builtin_interfaces::msg::Duration>("planning_time",
          "Time taken to compute route"),
        BT::OutputPort<nav_msgs::msg::Path>("path", "Path created by ComputeRoute node"),
        BT::OutputPort<ActionResult::_error_code_type>(
          "error_code_id", "The compute route error code"),
        BT::OutputPort<std::string>(
          "error_msg", "The compute route error msg"),
      });
  }
};

}  // namespace whi_nav2_bt_plugins
