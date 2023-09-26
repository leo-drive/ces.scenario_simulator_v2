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
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node(std::filesystem::path(argv[0]).stem());

  const auto width = [&]() {
    node.declare_parameter("width", 10.0);
    return node.get_parameter("width").as_double();
  }();

  const auto length = [&]() {
    node.declare_parameter("length_at_least", 100.0);
    const auto length_at_least = node.get_parameter("length_at_least").as_double();
    node.declare_parameter("length", length_at_least * 1.1);
    return std::max(length_at_least, node.get_parameter("length").as_double());
  }();

  const auto curvature = [&]() {
    node.declare_parameter("curvature", 0.0);
    return node.get_parameter("curvature").as_double();
  }();

  const auto resolution = [&]() {
    node.declare_parameter("resolution", 100);
    return node.get_parameter("resolution").as_int();
  }();

  const auto output_directory = [&]() {
    node.declare_parameter("output_directory", map_fragment::directory());
    return std::filesystem::path(node.get_parameter("output_directory").as_string());
  }();

  auto lanelets = lanelet::Lanelets();

  using map_fragment::makeCurvature;
  using map_fragment::makeLanelet;
  using map_fragment::makeLaneletLeft;
  using map_fragment::makeLaneletRight;

  lanelets.push_back(makeLanelet(width, length, curvature, resolution));

  lanelets.push_back(makeLanelet(lanelets[0], length, makeCurvature(length, +90), resolution));
  lanelets.push_back(makeLanelet(lanelets[0], length, makeCurvature(length, -90), resolution));

  lanelets.push_back(makeLanelet(lanelets[1], length, 0, resolution));
  lanelets.push_back(makeLanelet(lanelets[1], length, makeCurvature(length, +180), resolution));
  lanelets.push_back(makeLanelet(lanelets[1], length, makeCurvature(length, -180), resolution));

  lanelets.push_back(makeLanelet(lanelets[4], 200, makeCurvature(200, -270), resolution));
  lanelets.push_back(makeLanelet(lanelets.back(), 200, makeCurvature(200, +270), resolution));

  lanelets.push_back(makeLanelet(lanelets[5], 100, makeCurvature(100, 45), resolution));
  lanelets.push_back(makeLanelet(lanelets.back(), 50, makeCurvature(50, -90), resolution));
  lanelets.push_back(makeLanelet(lanelets.back(), 10, 0, resolution));

  lanelets.push_back(makeLaneletLeft(lanelets[0], resolution));
  lanelets.push_back(makeLaneletRight(lanelets[0], resolution));

  lanelets.push_back(makeLaneletLeft(lanelets[1], resolution));
  lanelets.push_back(makeLaneletRight(lanelets[1], resolution));

  lanelets.push_back(makeLaneletLeft(lanelets[2], resolution));
  lanelets.push_back(makeLaneletRight(lanelets[2], resolution));

  const auto map = lanelet::utils::createMap(lanelets);

  map_fragment::write(*map, output_directory);

  std::cout << output_directory.c_str() << std::endl;

  return EXIT_SUCCESS;
} catch (const std::exception & exception) {
  std::cerr << exception.what() << std::endl;
  return EXIT_FAILURE;
}
