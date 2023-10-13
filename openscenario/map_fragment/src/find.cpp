// Copyright 2015 TIER IV, Inc. All rights reserved.
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

#include <map_fragment/map_fragment.hpp>
#include <rclcpp/rclcpp.hpp>

auto main(const int argc, char const * const * const argv) -> int
try {
  using namespace map_fragment;

  rclcpp::init(argc, argv);

  auto node = rclcpp::Node(std::filesystem::path(argv[0]).stem());

  const auto input_directory = [&]() {
    node.declare_parameter("input_directory", default_value::directory());
    return std::filesystem::path(node.get_parameter("input_directory").as_string());
  }();

  const auto lanelet2_map = [&]() {
    node.declare_parameter("lanelet2_map", input_directory / "lanelet2_map.osm");
    return node.get_parameter("lanelet2_map").as_string();
  }();

  const auto type = [&]() {
    node.declare_parameter("type", "lanelet");
    return node.get_parameter("type").as_string();
  }();

  const auto subtype = [&]() {
    node.declare_parameter("subtype", "road");
    return node.get_parameter("subtype").as_string();
  }();

  auto satisfies_lower_bound_id = [&]() {
    node.declare_parameter("greater_than", lanelet::Id());
    return [lower_bound = node.get_parameter("greater_than").as_int()](const auto & lanelet) {
      return lower_bound < lanelet.id();
    };
  }();

  auto satisfies_upper_bound_id = [&]() {
    node.declare_parameter("less_than", std::numeric_limits<lanelet::Id>::max());
    return [upper_bound = node.get_parameter("less_than").as_int()](const auto & lanelet) {
      return lanelet.id() < upper_bound;
    };
  }();

  const auto map = lanelet::load(lanelet2_map, map_fragment::projector());

  auto finder = [&](const auto & lanelet) {
    auto matches = [&](const auto & attribute, const auto & value) {
      auto iterator = lanelet.attributes().find(attribute);
      return iterator != lanelet.attributes().end() and iterator->second == value;
    };

    return matches("type", type) and              //
           matches("subtype", subtype) and        //
           satisfies_lower_bound_id(lanelet) and  //
           satisfies_upper_bound_id(lanelet);
  };

  std::cerr << "count = "
            << std::count_if(map->laneletLayer.begin(), map->laneletLayer.end(), finder)
            << std::endl;

  if (auto iterator = std::find_if(map->laneletLayer.begin(), map->laneletLayer.end(), finder);
      iterator != map->laneletLayer.end()) {
    std::cout << iterator->id() << std::endl;
    return EXIT_SUCCESS;
  } else {
    throw std::runtime_error("not found");
  }
} catch (const std::exception & exception) {
  std::cerr << exception.what() << std::endl;
  return EXIT_FAILURE;
}
