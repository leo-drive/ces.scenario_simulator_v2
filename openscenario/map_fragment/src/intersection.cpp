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

#include <map_fragment/road_segment.hpp>
// TODO: Add header guards so there is no redefinition
// #include <map_fragment/map_fragment.hpp>
#include <rclcpp/rclcpp.hpp>

// Structure of the intersection
// TODO: Make this configurable
const uint64_t NUMBER_OF_LANES_LEFT = 0;
const uint64_t NUMBER_OF_LANES_LEFT_OR_STRAIGHT = 0;
const uint64_t NUMBER_OF_LANES_STRAIGHT = 0;
const uint64_t NUMBER_OF_LANES_ANY_DIRECTION = 2;
const uint64_t NUMBER_OF_LANES_STRAIGHT_OR_RIGHT = 0;
const uint64_t NUMBER_OF_LANES_RIGHT = 0;

auto main(const int argc, char const * const * const argv) -> int
try {
  using namespace map_fragment;

  rclcpp::init(argc, argv);

  auto node = rclcpp::Node(std::filesystem::path(argv[0]).stem());

  const auto lane_width = [&]() {
    node.declare_parameter("lane_width", default_value::width);
    return node.get_parameter("lane_width").as_double();
  }();

  const auto leg_length = [&]() {
    node.declare_parameter("leg_length", default_value::length);
    return node.get_parameter("leg_length").as_double();
  }();

  const auto turn_radius = [&]() {
    node.declare_parameter("turn_radius", default_value::turn_radius);
    return node.get_parameter("turn_radius").as_double();
  }();

  const auto resolution = [&]() {
    node.declare_parameter("resolution", default_value::resolution);
    return node.get_parameter("resolution").as_int();
  }();

  const auto output_directory = [&]() {
    node.declare_parameter("output_directory", default_value::directory());
    return std::filesystem::path(node.get_parameter("output_directory").as_string());
  }();

  std::vector<RoadSegment> road_segments;
  auto total_number_of_lanes_per_leg \
    = NUMBER_OF_LANES_LEFT \
    + NUMBER_OF_LANES_LEFT_OR_STRAIGHT \
    + NUMBER_OF_LANES_STRAIGHT \
    + NUMBER_OF_LANES_ANY_DIRECTION \
    + NUMBER_OF_LANES_STRAIGHT_OR_RIGHT \
    + NUMBER_OF_LANES_RIGHT;

  Transformation2d leg_origin_transformation;
  leg_origin_transformation.translation.x = -turn_radius;

  Transformation2d transformation_between_subsequent_legs;
  transformation_between_subsequent_legs.rotation = M_PI / 2;

  /*
   * For each intersection leg...
   */
  for (auto i = 0; i < 4; i++)
  {
    RoadCrossSectionDescription leg_cross_section_description = {
      2 * total_number_of_lanes_per_leg,
      lane_width
    };

    uint64_t first_lane_index_for_direction;
    uint64_t number_of_lanes_for_direction;
    RoadCrossSectionDescription cross_section_description_for_direction;
    double lateral_offset_for_direction;
    ParametricCurve::Ptr guide_curve_for_direction;

    /*
     * 1) Create road segment for the leg (adjacent to the intersection)
     */

    Transformation2d leg_transformation;
    leg_transformation.translation.x = -leg_length;

    ParametricCurve::Ptr leg_guide_curve = std::make_shared<Straight>(leg_length);
    
    transformCurve(leg_guide_curve,
                   leg_transformation,
                   leg_guide_curve);
    
    transformCurve(leg_guide_curve,
                   leg_origin_transformation,
                   leg_guide_curve);
    
    road_segments.emplace_back(leg_guide_curve, leg_cross_section_description);

    /*
     * 2) Create road segment for left-turning lanelets
     */

    first_lane_index_for_direction = 0;

    number_of_lanes_for_direction \
      = NUMBER_OF_LANES_LEFT \
      + NUMBER_OF_LANES_LEFT_OR_STRAIGHT
      + NUMBER_OF_LANES_ANY_DIRECTION;

    generateSliceOfCrossSection(leg_cross_section_description,
                                first_lane_index_for_direction,
                                number_of_lanes_for_direction,
                                cross_section_description_for_direction,
                                lateral_offset_for_direction);

    guide_curve_for_direction = std::make_shared<Arc>(turn_radius, M_PI / 2);

    shiftCurveLaterally(guide_curve_for_direction,
                        lateral_offset_for_direction,
                        guide_curve_for_direction);

    transformCurve(guide_curve_for_direction,
                   leg_origin_transformation,
                   guide_curve_for_direction);
    
    road_segments.emplace_back(guide_curve_for_direction, cross_section_description_for_direction);

    /*
     * 3) Create road segment for straight lanelets
     */

    first_lane_index_for_direction += NUMBER_OF_LANES_LEFT;

    number_of_lanes_for_direction \
      = NUMBER_OF_LANES_LEFT_OR_STRAIGHT \
      + NUMBER_OF_LANES_STRAIGHT \
      + NUMBER_OF_LANES_ANY_DIRECTION \
      + NUMBER_OF_LANES_STRAIGHT_OR_RIGHT;

    generateSliceOfCrossSection(leg_cross_section_description,
                                first_lane_index_for_direction,
                                number_of_lanes_for_direction,
                                cross_section_description_for_direction,
                                lateral_offset_for_direction);
    
    guide_curve_for_direction = std::make_shared<Straight>(2 * turn_radius);
    
    shiftCurveLaterally(guide_curve_for_direction,
                        lateral_offset_for_direction,
                        guide_curve_for_direction);

    transformCurve(guide_curve_for_direction,
                   leg_origin_transformation,
                   guide_curve_for_direction);
    
    road_segments.emplace_back(guide_curve_for_direction, cross_section_description_for_direction);

    /*
     * 4) Create road segment for right-turning lanelets
     */

    first_lane_index_for_direction \
      += NUMBER_OF_LANES_LEFT_OR_STRAIGHT \
      + NUMBER_OF_LANES_STRAIGHT;

    number_of_lanes_for_direction \
      = NUMBER_OF_LANES_ANY_DIRECTION \
      + NUMBER_OF_LANES_STRAIGHT_OR_RIGHT \
      + NUMBER_OF_LANES_RIGHT;

    generateSliceOfCrossSection(leg_cross_section_description,
                                first_lane_index_for_direction,
                                number_of_lanes_for_direction,
                                cross_section_description_for_direction,
                                lateral_offset_for_direction);

    guide_curve_for_direction = std::make_shared<Arc>(turn_radius, -M_PI / 2);
    
    shiftCurveLaterally(guide_curve_for_direction,
                        lateral_offset_for_direction,
                        guide_curve_for_direction);

    transformCurve(guide_curve_for_direction,
                   leg_origin_transformation,
                   guide_curve_for_direction);
    
    road_segments.emplace_back(guide_curve_for_direction, cross_section_description_for_direction);

    leg_origin_transformation = chainTransformations(
      transformation_between_subsequent_legs,
      leg_origin_transformation
    );
  }

  lanelet::Lanelets lanelets;
  for (auto& road_segment : road_segments)
  {
    auto road_segment_lanelets = road_segment.getLanelets(resolution);
   
    lanelets.insert(lanelets.end(),
                    road_segment_lanelets.begin(),
                    road_segment_lanelets.end());
  }

  const auto map = lanelet::utils::createMap(lanelets);

  map_fragment::write(*map, output_directory);

  std::cout << output_directory.c_str() << std::endl;

  return EXIT_SUCCESS;
} catch (const std::exception & exception) {
  std::cerr << exception.what() << std::endl;
  return EXIT_FAILURE;
}
