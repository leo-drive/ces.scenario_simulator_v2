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

#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_io/Io.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <map_fragment/map_fragment.hpp>
#include <rclcpp/rclcpp.hpp>

#define PRINT(...) std::cout << #__VA_ARGS__ " = " << std::boolalpha << (__VA_ARGS__) << std::endl

auto makePoint3d(double x, double y, double z, double elevation = 0.0)
{
  static lanelet::Id id = 0;
  auto point = lanelet::Point3d(++id, x, y, z);
  point.attributes()["ele"] = elevation;
  return point;
}

auto makeLineString3d(
  const lanelet::Point3d & origin, double length, double radius, std::size_t resolution)
{
  static lanelet::Id id = 0;

  if (std::isinf(radius)) {
    return lanelet::LineString3d(
      ++id, {origin, makePoint3d(origin.x() + length, origin.y(), origin.z())});
  } else {
    auto line = lanelet::LineString3d(++id);

    if (M_PI * 2 < length / radius) {
      PRINT(length / radius);
      std::exit(EXIT_FAILURE);
    }

    const auto radian_step = length / radius / resolution;

    auto total_length = 0.0;

    for (auto radian = 0.0; 0 < resolution; radian += radian_step, --resolution) {
      auto x = origin.x() + radius * std::sin(radian);
      auto y = origin.y() + radius * std::cos(radian) - radius;
      auto z = origin.z();
      line.push_back(makePoint3d(x, y, z));
      total_length += radius * radian_step;
    }

    auto x = origin.x() + radius * std::sin(length / radius);
    auto y = origin.y() + radius * std::cos(length / radius) - radius;
    auto z = origin.z();
    line.push_back(makePoint3d(x, y, z));

    return line;
  }
}

auto makeLanelet(double length, double width, double curvature, double resolution)
{
  const auto x = 0.0;
  const auto y = 0.0;
  const auto z = 0.0;

  const auto p1 = makePoint3d(x, y - width / 2, z);
  const auto p2 = makePoint3d(x, y + width / 2, z);

  const auto radius = curvature == 0 ? std::numeric_limits<double>::infinity() : 1 / curvature;

  const auto p1_radius = radius - width / 2;
  const auto p2_radius = radius + width / 2;

  const auto p1_offset = std::isinf(radius) or std::isinf(p1_radius)
                           ? 0.0
                           : p1_radius * (length / radius - length / p1_radius);

  const auto p2_offset = std::isinf(radius) or std::isinf(p2_radius)
                           ? 0.0
                           : p2_radius * (length / radius - length / p2_radius);

  static lanelet::Id id = 0;
  auto lane = lanelet::Lanelet(
    ++id, makeLineString3d(p1, length + p1_offset, p1_radius, resolution),
    makeLineString3d(p2, length + p2_offset, p2_radius, resolution));
  lane.attributes()["subtype"] = "road";
  return lane;
}

auto main(const int argc, char const * const * const argv) -> int
try {
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node(std::filesystem::path(argv[0]).stem());

  const auto length_at_least = [&]() {
    node.declare_parameter("length_at_least", 100.0);
    return node.get_parameter("length_at_least").as_double();
  }();

  const auto length = [&]() {
    node.declare_parameter("length", length_at_least * 1.1);
    return std::max(length_at_least, node.get_parameter("length").as_double());
  }();

  const auto width = [&]() {
    node.declare_parameter("width", 10.0);
    return node.get_parameter("width").as_double();
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

  auto map = lanelet::LaneletMap();

  map.add(makeLanelet(length, width, curvature, resolution));

  try {
    if (std::filesystem::remove_all(output_directory);
        not std::filesystem::create_directories(output_directory)) {
      RCLCPP_ERROR_STREAM(node.get_logger(), "failed to create directory " << output_directory);
    }
  } catch (const std::exception & exception) {
    RCLCPP_ERROR_STREAM(node.get_logger(), exception.what());
    return EXIT_FAILURE;
  }

  lanelet::write(output_directory / "lanelet2_map.osm", map, map_fragment::projector());

  try {
    std::filesystem::create_symlink(
      std::filesystem::canonical(
        std::filesystem::path(ament_index_cpp::get_package_share_directory("kashiwanoha_map")) /
        "map/pointcloud_map.pcd"),
      output_directory / "pointcloud_map.pcd");
  } catch (const std::exception & exception) {
    RCLCPP_ERROR_STREAM(node.get_logger(), exception.what());
    return EXIT_FAILURE;
  }

  std::cout << output_directory.c_str() << std::endl;

  return EXIT_SUCCESS;
} catch (const std::exception & exception) {
  std::cerr << exception.what() << std::endl;
  return EXIT_FAILURE;
}
