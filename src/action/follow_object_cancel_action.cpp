// Copyright (c) 2024 Open Navigation LLC
// Copyright (c) 2024 Alberto J. Tudela Roldán
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <string>
#include <memory>

#include "whi_nav2_bt_plugins/action/follow_object_cancel_action.hpp"

namespace whi_nav2_bt_plugins
{
	FollowObjectCancel::FollowObjectCancel(
		const std::string &xml_tag_name,
		const std::string &action_name,
		const BT::NodeConfiguration &conf)
		: nav2_behavior_tree::BtCancelActionNode<whi_interfaces::action::FollowObject>(
			  xml_tag_name, action_name, conf)
	{
	}

} // namespace whi_nav2_bt_plugins

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
	BT::NodeBuilder builder =
		[](const std::string &name, const BT::NodeConfiguration &config)
	{
		return std::make_unique<whi_nav2_bt_plugins::FollowObjectCancel>(
			name, "follow_object_ext", config);
	};

	factory.registerBuilder<whi_nav2_bt_plugins::FollowObjectCancel>(
		"CancelFollowObjectExt", builder);
}
