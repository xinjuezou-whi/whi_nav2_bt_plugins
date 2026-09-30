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
#include "whi_nav2_bt_plugins/action/compute_route_action.hpp"

#include <memory>
#include <string>

namespace whi_nav2_bt_plugins
{

ComputeRouteAction::ComputeRouteAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<Action>(xml_tag_name, action_name, conf)
{
}

void ComputeRouteAction::on_tick()
{
  bool use_poses = false, use_start = false;
  getInput("use_poses", use_poses);
  if (use_poses) {
    goal_.use_poses = true;
    getInput("goal", goal_.goal);

    goal_.use_start = false;
    getInput("use_start", use_start);
    if (use_start) {
      getInput("start", goal_.start);
      goal_.use_start = true;
    }
  } else {
    getInput("start_id", goal_.start_id);
    getInput("goal_id", goal_.goal_id);
    goal_.use_start = false;
    goal_.use_poses = false;
  }
}

BT::NodeStatus ComputeRouteAction::on_success()
{
  setOutput("path", result_.result->path);
  setOutput("route", result_.result->route);
  setOutput("planning_time", result_.result->planning_time);
  // Set empty error code, action was successful
  setOutput("error_code_id", ActionResult::NONE);
  setOutput("error_msg", "");
  return BT::NodeStatus::SUCCESS;
}

void ComputeRouteAction::resetPorts()
{
  nav_msgs::msg::Path empty_path;
  setOutput("path", empty_path);
  nav2_msgs::msg::Route empty_route;
  setOutput("route", empty_route);
  setOutput("planning_time", builtin_interfaces::msg::Duration());
}

BT::NodeStatus ComputeRouteAction::on_aborted()
{
  resetPorts();
  setOutput("error_code_id", result_.result->error_code);
  setOutput("error_msg", result_.result->error_msg);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ComputeRouteAction::on_cancelled()
{
  resetPorts();
  // Set empty error code, action was cancelled
  setOutput("error_code_id", ActionResult::NONE);
  setOutput("error_msg", "");
  return BT::NodeStatus::SUCCESS;
}

void ComputeRouteAction::on_timeout()
{
  setOutput("error_code_id", ActionResult::TIMEOUT);
  setOutput("error_msg", "Behavior Tree action client timed out waiting.");
}

void ComputeRouteAction::halt()
{
  resetPorts();
  // DO NOT reset "error_code_id" output port, we want to read it later
  // DO NOT reset "error_msg" output port, we want to read it later
  BtActionNode::halt();
}

}  // namespace whi_nav2_bt_plugins

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<whi_nav2_bt_plugins::ComputeRouteAction>(
        name, "whi_compute_route", config); // advertised action name
    };

  factory.registerBuilder<whi_nav2_bt_plugins::ComputeRouteAction>(
    "ComputeRouteWhi", builder); // plugin name
}
