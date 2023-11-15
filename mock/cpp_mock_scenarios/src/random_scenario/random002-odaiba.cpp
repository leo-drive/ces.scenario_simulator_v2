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

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cpp_mock_scenarios/catalogs.hpp>
#include <cpp_mock_scenarios/cpp_scenario_node.hpp>
#include <random001_parameters.hpp>
#include <rclcpp/rclcpp.hpp>
#include <traffic_simulator/api/api.hpp>

#include <traffic_simulator_msgs/msg/behavior_parameter.hpp>

#include <quaternion_operation/quaternion_operation.h>

// headers in STL
#include <memory>
#include <random>
#include <string>
#include <vector>
#include <cstdlib>
#include <ctime>

using traffic_simulator::LaneletPose;
using traffic_simulator::helper::constructLaneletPose;
using traffic_simulator::lane_change::Direction;

enum class DIRECTION {
  CENTER,
  LEFT,
  RIGHT,
};

namespace
{
uint8_t get_entity_subtype(const std::string & entity_type)
{
  using traffic_simulator_msgs::msg::EntitySubtype;
  if (entity_type == "car") {
    return EntitySubtype::CAR;
  } else if (entity_type == "truck") {
    return EntitySubtype::TRUCK;
  } else if (entity_type == "bus") {
    return EntitySubtype::BUS;
  } else if (entity_type == "trailer") {
    return EntitySubtype::TRAILER;
  }
  return EntitySubtype::CAR;
}

geometry_msgs::msg::Pose createPose(const double x, const double y, const double z = 0.0)
{
  return geometry_msgs::build<geometry_msgs::msg::Pose>()
    .position(geometry_msgs::build<geometry_msgs::msg::Point>().x(x).y(y).z(z))
    .orientation(geometry_msgs::msg::Quaternion());
}

// Function to generate a random integer between min and max
int randomInt(int min, int max)
{
  if (min == max) return min;
  return min + rand() % (max - min + 1);
}

// Function to generate a random double between min and max
double randomDouble(double min, double max)
{
  double range = max - min;
  if (std::abs(range) < 0.0001) return min;
  double div = RAND_MAX / range;
  return min + (rand() / div);
}
}  // namespace

class RandomScenario : public cpp_mock_scenarios::CppScenarioNode
{
public:
  explicit RandomScenario(const rclcpp::NodeOptions & option)
  : cpp_mock_scenarios::CppScenarioNode(
      "lanechange_left", /* ament_index_cpp::get_package_share_directory("kashiwanoha_map") + "/map" */
      "/home/horibe/workspace/map/odaiba_reference",
      "lanelet2_map.osm", __FILE__, false, option),
    param_listener_(std::make_shared<random001::ParamListener>(get_node_parameters_interface())),
    engine_(seed_gen_())
  {
    start();
  }

private:
  std::shared_ptr<random001::ParamListener> param_listener_;
  random001::Params params_;
  std::random_device seed_gen_;
  std::mt19937 engine_;
  double lane_change_position = 0.0;
  bool lane_change_requested = false;

  void spawnAndCrossPedestrian(
    const int entity_index, const lanelet::Id & spawn_lane_id, const lanelet::Id & goal_lane_id)
  {
    const auto & p = params_.random_parameters.crossing_pedestrian;
    std::normal_distribution<> offset_distribution(0.0, p.offset_variance);
    std::uniform_real_distribution<> speed_distribution(p.min_speed, p.max_speed);
    const auto spawn_pose = constructLaneletPose(spawn_lane_id, 0.0, offset_distribution(engine_));
    const auto goal_pose = constructLaneletPose(goal_lane_id, 25.0);

    std::string entity_name = "pedestrian_" + std::to_string(spawn_lane_id) + "_" +
                              std::to_string(goal_lane_id) + "_" + std::to_string(entity_index);
    constexpr double reach_tolerance = 5.0;
    if (
      !api_.entityExists(entity_name) &&
      !api_.reachPosition("ego", api_.canonicalize(goal_pose), reach_tolerance)) {
      std::normal_distribution<> offset_distribution(0.0, p.offset_variance);
      std::uniform_real_distribution<> speed_distribution(p.min_speed, p.max_speed);
      api_.spawn(entity_name, api_.canonicalize(spawn_pose), getPedestrianParameters());
      const auto speed = speed_distribution(engine_);
      api_.requestSpeedChange(entity_name, speed, true);
      api_.setLinearVelocity(entity_name, speed);
    }
    if (api_.entityExists(entity_name) && api_.getStandStillDuration(entity_name) >= 0.5) {
      api_.despawn(entity_name);
    }
  }

  void spawnAndChangeLane(
    const std::string & entity_name, const LaneletPose & spawn_pose, const lanelet::Id & lane_change_id,
    const Direction & lane_change_direction)
  {
    const auto & p = params_.random_parameters.lane_following_vehicle;
    if (!api_.entityExists(entity_name)) {
      api_.spawn(entity_name, api_.canonicalize(spawn_pose), getVehicleParameters());
      std::uniform_real_distribution<> speed_distribution(p.min_speed, p.max_speed);
      const auto speed = speed_distribution(engine_);
      api_.requestSpeedChange(entity_name, speed, true);
      api_.setLinearVelocity(entity_name, speed);
      std::uniform_real_distribution<> lane_change_position_distribution(
        0.0, api_.getLaneletLength(lane_change_id));
      lane_change_position = lane_change_position_distribution(engine_);
      lane_change_requested = false;
    }
    const auto lanelet_pose = api_.getLaneletPose("ego");
    /// Checking the ego entity overs the lane change position.
    if (
      lanelet_pose && static_cast<LaneletPose>(lanelet_pose.value()).lanelet_id == lane_change_id &&
      std::abs(static_cast<LaneletPose>(lanelet_pose.value()).s) >= lane_change_position) {
      api_.requestLaneChange(entity_name, lane_change_direction);
      lane_change_requested = true;
    }
  }

  void spawnAndMoveToGoal(const lanelet::Id & spawn_lane_id, const lanelet::Id & goal_lane_id)
  {
    const auto & p = params_.random_parameters.lane_following_vehicle;
    const auto spawn_pose = constructLaneletPose(spawn_lane_id, 0.0);
    const auto goal_pose = constructLaneletPose(goal_lane_id, 0.0);

    const auto entity_name =
      "vehicle_move_to_goal_" + std::to_string(spawn_lane_id) + "_" + std::to_string(goal_lane_id);
    if (!api_.entityExists(entity_name)) {
      api_.spawn(entity_name, api_.canonicalize(spawn_pose), getVehicleParameters());
      std::uniform_real_distribution<> speed_distribution(p.min_speed, p.max_speed);
      const auto speed = speed_distribution(engine_);
      api_.requestSpeedChange(entity_name, speed, true);
      api_.setLinearVelocity(entity_name, speed);
    }

    constexpr double reach_tolerance = 2.0;
    if (api_.reachPosition(entity_name, api_.canonicalize(goal_pose), reach_tolerance)) {
        api_.despawn(entity_name);
    }
  }

  void spawnRoadParkingVehicles(const lanelet::Id & spawn_lanelet_id, const size_t number_of_vehicles, const DIRECTION direction)
  {
    const auto & p = params_.random_parameters.road_parking_vehicle;
    std::normal_distribution<> normal_dist(0.0, p.s_variance);

    const auto spawn_road_parking_vehicle = [&](const auto & entity_index, const auto offset) {
      std::string entity_name =
        "road_parking_" + std::to_string(spawn_lanelet_id) + "_" + std::to_string(entity_index);
      if (api_.entityExists(entity_name)) {
        return;
      }
      const auto space = static_cast<double>(entity_index) / number_of_vehicles;
      const auto spawn_position =
        space * api_.getLaneletLength(spawn_lanelet_id) + normal_dist(engine_);
      const auto spawn_pose = constructLaneletPose(spawn_lanelet_id, spawn_position, offset, 0, 0);
      const auto vehicle_param = getVehicleParameters(get_entity_subtype(p.entity_type));
      api_.spawn(entity_name, api_.canonicalize(spawn_pose), vehicle_param);
      api_.requestSpeedChange(entity_name, 0, true);
    };

    const auto [min_offset, max_offset] = [&]() -> std::pair<double, double> {
      if (direction == DIRECTION::CENTER) {
        return {-0.5, 0.5};
      } else if (direction == DIRECTION::LEFT) {
        return {3.0, 1.0};
      } else {
        return {-1.0, -3.0};
      }
    }();
    std::uniform_real_distribution<> dist(min_offset, max_offset);
    for (size_t i = 0; i < number_of_vehicles; i++) {
      spawn_road_parking_vehicle(i, dist(engine_));
    }
  }

  /// Despawn parking entity before replacing parking entity.
  void despawnRoadParkingVehicles()
  {
    for (int i = 0; i < params_.random_parameters.road_parking_vehicle.number_of_vehicle; i++) {
      api_.despawn("road_parking_" + std::to_string(i));
    }
  }

  void despawnCrossingPedestrians()
  {
    for (int i = 0; i < params_.random_parameters.crossing_pedestrian.number_of_pedestrian; i++) {
      std::string entity_name = "pedestrian" + std::to_string(i);
      if (api_.entityExists(entity_name)) {
        api_.despawn(entity_name);
      }
    }
  }

  void spawnAndDespawnRelativeFromEgoInRange(
    const lanelet::Id & trigger_lane_id, const double trigger_lane_s, const double trigger_range,
    const double rel_x, const double rel_y)
  {
    const auto trigger_position =
      api_.canonicalize(constructLaneletPose(trigger_lane_id, trigger_lane_s));
    const auto entity_name = "spawn_nearby_ego";
    if (
      api_.reachPosition("ego", trigger_position, trigger_range) &&
      !api_.entityExists(entity_name)) {
      api_.spawn(
        entity_name, api_.getMapPoseFromRelativePose("ego", createPose(rel_x, rel_y)),
        getVehicleParameters(),
        traffic_simulator::entity::VehicleEntity::BuiltinBehavior::doNothing());
    }
    if (
      !api_.reachPosition("ego", trigger_position, trigger_range) &&
      api_.entityExists(entity_name)) {
      api_.despawn(entity_name);
    }
  }



  void onUpdate() override
  {
#if 0    
    // パラメータが更新されたら、全車両を更新
    if (param_listener_->is_old(params_)) {
      /// When the parameter was updated, clear entity before re-spawing entity.
      despawnRoadParkingVehicles();
      despawnCrossingPedestrians();
      param_listener_->refresh_dynamic_parameters();
      params_ = param_listener_->get_params();

      /// Re-spawn road parking vehicle.
      spawnRoadParkingVehicles(34705);
    }

    // Spawn lane-changing vehicles
    if (api_.isInLanelet("ego", 34684, 0.1)) {
      spawnAndChangeLane(
        "lane_following_0", constructLaneletPose(34513, 0.0), 34684, Direction::RIGHT);
    }

    /// Spawn and cross pedestrian if it does not exist and ego entity does not exists on lane
    /// "34576"
    for (int i = 0; i < params_.random_parameters.crossing_pedestrian.number_of_pedestrian; i++) {
      spawnAndCrossPedestrian(i, 34392, 34576);
    }

    spawnAndDespawnRelativeFromEgoInRange(34621, 10.0, 20.0, 10.0, -5.0);
#endif


    // Initialize random seed
    srand(time(0));

    // やりたいこと
    // 特定のlane_idの200m以内になったら、回避対象をspawn（位置はランダム、数もランダム）
    // spawnRoadParkingVehicles(176148, randomInt(3, 4), DIRECTION::LEFT);
    // spawnRoadParkingVehicles(176193, randomInt(3, 4), DIRECTION::LEFT);
    // spawnRoadParkingVehicles(1501, randomInt(3, 4), DIRECTION::RIGHT);
    // spawnRoadParkingVehicles(1500, randomInt(3, 4), DIRECTION::CENTER);
    spawnAndMoveToGoal(176261, 176187);

    // 特定のlane_idの200m以内になったら、横断歩道歩行者をspawn（速度は毎回ランダム、人数はspawnで抽選、数秒ごとにspawnを止める）

    // 特定のlane_idの200m以内になったら、交差点の奥から車両が来る。（速度は毎回ランダム、数秒ごとにspawnを止める）


  }

  void onInitialize() override
  {
    // api_.setVerbose(true);
    params_ = param_listener_->get_params();

    /// Spawn road parking vehicle with initial parameters.
    // spawnRoadParkingVehicles(176148, 4, DIRECTION::LEFT);
    spawnRoadParkingVehicles(176193, randomInt(3, 4), DIRECTION::LEFT);
    spawnRoadParkingVehicles(1501, randomInt(3, 4), DIRECTION::RIGHT);
    spawnRoadParkingVehicles(1500, randomInt(3, 4), DIRECTION::CENTER);
    spawnAndMoveToGoal(176261, 176187);

    const auto spawn_pose = api_.canonicalize(constructLaneletPose(176126, 10, 0, 0, 0, 0));
    const auto goal_pose = api_.canonicalize(constructLaneletPose(108, 0, 0, 0, 0, 0));
    spawnEgoEntity(spawn_pose, {goal_pose}, getVehicleParameters());

    // api_.spawn(
    //   "parking_outside", api_.getMapPoseFromRelativePose("ego", createPose(10.0, 15.0)),
    //   getVehicleParameters(),
    //   traffic_simulator::entity::VehicleEntity::BuiltinBehavior::doNothing());
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<RandomScenario>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}
