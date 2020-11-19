// Copyright 2015-2020 Autoware Foundation. All rights reserved.
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

#include <awapi_awauto_adapter/autoware_auto_adapter.hpp>
#include <rclcpp_components/register_node_macro.hpp>
using namespace std::chrono_literals;
namespace autoware_api
{
AutowareAutoAdapter::AutowareAutoAdapter(const rclcpp::NodeOptions & options)
: rclcpp::Node("autoware_auto_adapter", options)
{
  pub_autoware_enage_ = this->create_publisher<AutowareEngage>("/awapi/autoware/put/engage", 1);
  timer_engage_ =
    this->create_wall_timer(500ms, std::bind(&AutowareAutoAdapter::dummy_engage_autoware, this));
}
void AutowareAutoAdapter::dummy_engage_autoware()
{
  AutowareEngage engage;
  engage.data = true;
  RCLCPP_INFO(this->get_logger(), "Publishing: '%i'", engage.data);
  pub_autoware_enage_->publish(engage);
}
}  // namespace autoware_api
RCLCPP_COMPONENTS_REGISTER_NODE(autoware_api::AutowareAutoAdapter)
