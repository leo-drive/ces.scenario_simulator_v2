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

  const auto map = lanelet::load(lanelet2_map, map_fragment::projector());

  const auto routing_graph = lanelet::routing::RoutingGraph::build(*map, vehicleTrafficRules());

  const std::list<std::function<bool(const lanelet::Lanelet &)>> constraints{
    [&]() {
      node.declare_parameter("type", "lanelet");
      const auto type = node.get_parameter("type").as_string();
      return [type](const auto & lanelet) {
        return lanelet.attribute("type") == type or [&]() {
          std::cerr << "[" << lanelet.id() << "] discarded: The type is not " << type << "."
                    << std::endl;
          return false;
        }();
      };
    }(),

    [&]() {
      node.declare_parameter("subtype", "road");
      const auto subtype = node.get_parameter("subtype").as_string();
      return [subtype](const auto & lanelet) {
        return lanelet.attribute("subtype") == subtype or [&]() {
          std::cerr << "[" << lanelet.id() << "] discarded: The subtype is not " << subtype << "."
                    << std::endl;
          return false;
        }();
      };
    }(),

    [&]() {
      node.declare_parameter("id_greater_than", lanelet::Id());
      const auto lower_bound = node.get_parameter("id_greater_than").as_int();
      return [lower_bound](const auto & lanelet) {
        return lower_bound < lanelet.id() or [&]() {
          std::cerr << "[" << lanelet.id() << "] discarded: The id is not greater than "
                    << lower_bound << "." << std::endl;
          return false;
        }();
      };
    }(),

    [&]() {
      node.declare_parameter("id_less_than", std::numeric_limits<lanelet::Id>::max());
      const auto upper_bound = node.get_parameter("id_less_than").as_int();
      return [upper_bound](const auto & lanelet) {
        return lanelet.id() < upper_bound or [&]() {
          std::cerr << "[" << lanelet.id() << "] discarded: The id is not less than " << upper_bound
                    << "." << std::endl;
          return false;
        }();
      };
    }(),

    [&]() {
      node.declare_parameter("route_length_greater_than", 0.0);
      const auto lower_bound = node.get_parameter("route_length_greater_than").as_double();
      return [&, lower_bound](const auto & lanelet) {
        return 0 < routing_graph->possiblePaths(lanelet, lower_bound).size() or [&]() {
          std::cerr
            << "[" << lanelet.id()
            << "] discarded: The length of the path routable from the lanelet is not greater than "
            << lower_bound << std::endl;
          return false;
        }();
      };
    }(),
  };

  auto satisfy = [&](const auto & lanelet) {
    return std::all_of(
      constraints.begin(), constraints.end(), [&](const auto satisfy) { return satisfy(lanelet); });
  };

  if (auto first = std::find_if(map->laneletLayer.begin(), map->laneletLayer.end(), satisfy);
      first != map->laneletLayer.end()) {
    if (auto second = std::find_if(std::next(first), map->laneletLayer.end(), satisfy);
        second != map->laneletLayer.end()) {
      std::stringstream what;
      what << "There are " << std::count_if(std::next(second), map->laneletLayer.end(), satisfy) + 2
           << " lanelets that satisfy the constraints.";
      throw std::runtime_error(what.str());
    } else {
      std::cout << first->id() << std::endl;
      return EXIT_SUCCESS;
    }
  } else {
    throw std::runtime_error("There is no lanelet that satisfies the constraints.");
  }
} catch (const std::exception & exception) {
  std::cerr << exception.what() << std::endl;
  return EXIT_FAILURE;
}
