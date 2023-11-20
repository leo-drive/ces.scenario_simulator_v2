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
#include <traffic_simulator/helper/stop_watch.hpp>

#include <quaternion_operation/quaternion_operation.h>
#include "./random_util.hpp"

// headers in STL
#include <memory>
#include <random>
#include <string>
#include <vector>
#include <cstdlib>
#include <ctime>

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

    // sub_initial_pose_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    //   "/initialpose3d", rclcpp::QoS(1),
    //   std::bind(&RandomScenario::onInitialPose, this, std::placeholders::_1));
  }

private:
  std::shared_ptr<random001::ParamListener> param_listener_;
  random001::Params params_;
  std::random_device seed_gen_;
  std::mt19937 engine_;
  double lane_change_position = 0.0;
  bool lane_change_requested = false;

  const size_t MAX_SPAWN_NUMBER = 10;
  bool ego_is_in_stuck_ = false;

  MyStopWatch<> sw_ego_stuck_;

  StateManager<std::string> tl_state_manager_{{"red", "amber", "green"}, {10.0, 3.0, 10.0}};

  // If ego is far from lane_id, remove all entities.
  // Return if the ego is close to the lane_id.
  bool removeFarNPCsAndCheckIsInTriggerDistance(
    const std::string & entity_name_prefix, const lanelet::Id & lane_id)
  {
    const auto removeEntities = [&]() {
      for (int i = 0; i < MAX_SPAWN_NUMBER; i++) {
        const std::string name = entity_name_prefix + "_" + std::to_string(i);
        if (api_.entityExists(name)) {
          api_.despawn(name);
        }
      }
    };

    constexpr auto untrigger_distance = 22000.0;  // must be longer than trigger_distance
    constexpr auto trigger_distance = 20000.0;  // must be shorter than untrigger_distance
    const auto target_lane = api_.canonicalize(constructLaneletPose(lane_id, 0.0));

    const bool already_exist = api_.entityExists(entity_name_prefix + "_0");

    if (already_exist) {
      if (!api_.reachPosition("ego", target_lane, untrigger_distance)) {
        removeEntities();
      }
      return false;  // no need to spawn vehicles
    }

    if (!api_.reachPosition("ego", target_lane, trigger_distance)) {
      return false;  // no need to spawn vehicles
    }

    return true;  // need to spawn vehicles
  }

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

  void spawnAndMoveToGoal(
    const lanelet::Id & spawn_lane_id, const lanelet::Id & goal_lane_id, const double min_v = 3.0,
    const double max_v = 18.0)
  {
    const std::string entity_name_prefix =
      "vehicle_move_to_goal_" + std::to_string(spawn_lane_id) + "_" + std::to_string(goal_lane_id);
    if (!removeFarNPCsAndCheckIsInTriggerDistance(entity_name_prefix, spawn_lane_id)) {
      return;
    }

    const auto spawn_pose = constructLaneletPose(spawn_lane_id, 0.0);
    const auto goal_pose = constructLaneletPose(goal_lane_id, 0.0);

    const auto entity_name = entity_name_prefix;
    if (!api_.entityExists(entity_name)) {
      api_.spawn(entity_name, api_.canonicalize(spawn_pose), getVehicleParameters());
      std::uniform_real_distribution<> speed_distribution(min_v, max_v);
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
    const std::string entity_name_prefix = "road_parking_" + std::to_string(spawn_lanelet_id);
    if (!removeFarNPCsAndCheckIsInTriggerDistance(entity_name_prefix, spawn_lanelet_id)) {
      return;
    }

    const auto & p = params_.random_parameters.road_parking_vehicle;
    std::normal_distribution<> normal_dist(0.0, p.s_variance);

    const auto spawn_road_parking_vehicle = [&](const auto & entity_index, const auto offset) {
      const std::string entity_name = entity_name_prefix + "_" + std::to_string(entity_index);
      const auto space = static_cast<double>(entity_index) / number_of_vehicles;
      const auto spawn_position =
        space * api_.getLaneletLength(spawn_lanelet_id) + normal_dist(engine_);
      const auto spawn_pose = constructLaneletPose(spawn_lanelet_id, spawn_position, offset, 0, 0);
      const auto vehicle_param = getVehicleParameters(get_entity_subtype(p.entity_type));
      if (!api_.entityExists(entity_name)) {
        api_.spawn(entity_name, api_.canonicalize(spawn_pose), vehicle_param);
      }
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

  // Set color for the traffic_right_id. Set opposite color (green <-> red) to the opposite_traffic_right_id
  void updateRandomTrafficLightColor(
    const std::vector<int> & traffic_right_ids, const std::vector<int> & opposite_traffic_right_ids,
    const std::string & tl_color)
  {
    const auto setTlColor = [&](const auto & ids, const std::string & color) {
      for (const auto id : ids) {
        for (traffic_simulator::TrafficLight & traffic_light :
             api_.getConventionalTrafficLights(id)) {
          traffic_light.clear();
          traffic_light.set(color + " solidOn circle");
        };
      }
    };

    setTlColor(traffic_right_ids, tl_color);
    setTlColor(opposite_traffic_right_ids, getOppositeTlColor(tl_color));
  }

  bool processForEgoStuck() {
    constexpr auto STUCK_TIME_THRESHOLD = 10.0;
    const auto stuck_time = api_.getStandStillDuration("ego");
    // std::cerr << "stuck_time = " << stuck_time << std::endl;

    // 10秒以上スタックしたら、NPC全消ししてflag true、タイマースタート。
    if (stuck_time > STUCK_TIME_THRESHOLD && !ego_is_in_stuck_) {
      std::cerr << "\n\nego is in stuck. remove all vehicles!!!\n\n" << std::endl;
      // api_.despawnEntities();
      auto entities = api_.getEntityNames();
      std::all_of(
        entities.begin(), entities.end(), [&](const auto & e) { return e == api_.getEgoName() ? false :  api_.despawn(e); });
      ego_is_in_stuck_ = true;
      sw_ego_stuck_.tic("ego_stuck");
    }

    // flagがtrueならstuck時の処理を確認。5秒経つまではNPC処理はしない。5秒立ったらフラグを下げる。
    // →　これは、egoの近傍過ぎるやつはspawnしないという処理を入れることで消せる。
    if (ego_is_in_stuck_) {
      if (sw_ego_stuck_.toc("ego_stuck") < 5.0) {
        return true;
      } else {
        ego_is_in_stuck_ = false;
        return false;
      }
    }
    return false;
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



    // const auto stuck_time = api_.getStandStillDuration("ego");
    // std::cerr << "stuck_time = " << stuck_time << std::endl;

    // // 10秒以上スタックしたら、NPC全消ししてflag true、タイマースタート。
    // if (stuck_time > 10.0 && !ego_is_in_stuck_) {
    //   std::cerr << "\n\nego is in stuck. remove all vehicles!!!\n\n" << std::endl;
    //   // api_.despawnEntities();
    //   auto entities = api_.getEntityNames();
    //   std::all_of(
    //     entities.begin(), entities.end(), [&](const auto & e) { return e == api_.getEgoName() ? false :  api_.despawn(e); });
    //   ego_is_in_stuck_ = true;
    //   sw_ego_stuck_.tic("ego_stuck");
    // }

    // // // flagがtrueならstuck時の処理を確認。5秒経つまではNPC処理はしない。5秒立ったらフラグを下げる。
    // if (ego_is_in_stuck_) {
    //   if (sw_ego_stuck_.toc("ego_stuck") < 5.0) {
    //     return;
    //   } else {
    //     ego_is_in_stuck_ = false;
    //   }
    // }

    if (processForEgoStuck()) {
      return;
    }

    // やりたいこと
    // 特定のlane_idの200m以内になったら、回避対象をspawn（位置はランダム、数もランダム）
    // spawnAndMoveToGoal(176261, 176175, MIN_VEL, MAX_VEL);
    spawnRoadParkingVehicles(176148, randomInt(0, 4), DIRECTION::LEFT);  // unstable
    spawnRoadParkingVehicles(176193, randomInt(0, 4), DIRECTION::LEFT);
    spawnRoadParkingVehicles(1501, randomInt(0, 4), DIRECTION::RIGHT);
    // spawnRoadParkingVehicles(174069, randomInt(1, 4), DIRECTION::CENTER);  // 路肩は非対応
    // spawnRoadParkingVehicles(1262, randomInt(1, 4), DIRECTION::RIGHT);  // 右駐車は避けれん
    spawnAndMoveToGoal(350, 163, MIN_VEL, MAX_VEL);
    spawnAndMoveToGoal(350, 1506, MIN_VEL, MAX_VEL);
    spawnAndMoveToGoal(1482, 38, MIN_VEL, MAX_VEL);
    spawnAndMoveToGoal(1483, 38, MIN_VEL, MAX_VEL);
    spawnAndMoveToGoal(1484, 39, MIN_VEL, MAX_VEL);
    spawnAndMoveToGoal(1501, 40, MIN_VEL, MAX_VEL);
    spawnAndMoveToGoal(32, 38, MIN_VEL, MAX_VEL);
    spawnAndMoveToGoal(33, 39, MIN_VEL, MAX_VEL);
    spawnAndMoveToGoal(34, 40, MIN_VEL, MAX_VEL);
    spawnAndMoveToGoal(1314, 41, MIN_VEL, MAX_VEL);
    spawnAndMoveToGoal(94, 41, MIN_VEL, MAX_VEL);

    spawnRoadParkingVehicles(1265, randomInt(1, 4), DIRECTION::LEFT);
    spawnAndMoveToGoal(175378, 174994, MIN_VEL, MAX_VEL);
    spawnAndMoveToGoal(1263, 106, MIN_VEL, MAX_VEL);
    spawnAndMoveToGoal(1265, 178001, MIN_VEL, MAX_VEL);
    spawnAndMoveToGoal(1153, 94, MIN_VEL, MAX_VEL);
    spawnAndMoveToGoal(178233, 179475, MIN_VEL, MAX_VEL);

    spawnAndMoveToGoal(74, 84, MIN_VEL, MAX_VEL);
    spawnAndMoveToGoal(75, 83, MIN_VEL, MAX_VEL);
    spawnAndMoveToGoal(75, 178573, MIN_VEL, MAX_VEL);
    spawnAndMoveToGoal(1483, 1500, MIN_VEL, MAX_VEL);


    spawnRoadParkingVehicles(1278, randomInt(1, 2), DIRECTION::LEFT);
    spawnRoadParkingVehicles(179398, randomInt(1, 2), DIRECTION::LEFT);
    spawnRoadParkingVehicles(190784, randomInt(0, 1), DIRECTION::LEFT);
    spawnRoadParkingVehicles(190797, randomInt(0, 1), DIRECTION::LEFT);

    spawnRoadParkingVehicles(1513, randomInt(1, 2), DIRECTION::CENTER);
    spawnRoadParkingVehicles(1468, randomInt(1, 2), DIRECTION::CENTER);
    spawnRoadParkingVehicles(178766, randomInt(0, 1), DIRECTION::LEFT);
    spawnRoadParkingVehicles(179473, randomInt(0, 1), DIRECTION::LEFT);

    // 特定のlane_idの200m以内になったら、信号を時間ごとに変える。
    updateRandomTrafficLightColor({10584}, {10589}, tl_state_manager_.getCurrentState());
    updateRandomTrafficLightColor({10324, 190343}, {10316, 10322}, tl_state_manager_.getCurrentState());
    updateRandomTrafficLightColor({10352}, {10356, 10359}, tl_state_manager_.getCurrentState());
    updateRandomTrafficLightColor({179285, 10284}, {10293, 10283}, tl_state_manager_.getCurrentState());
    updateRandomTrafficLightColor({10269, 10276}, {10263, 10277}, tl_state_manager_.getCurrentState());
    updateRandomTrafficLightColor({10247, 10261}, {10249}, tl_state_manager_.getCurrentState());
    updateRandomTrafficLightColor({10237, 10236}, {10238}, tl_state_manager_.getCurrentState());
    updateRandomTrafficLightColor({10546, 10549}, {10551}, tl_state_manager_.getCurrentState());
    updateRandomTrafficLightColor({10562, 10564}, {10556}, tl_state_manager_.getCurrentState());
    updateRandomTrafficLightColor({10575, 10569}, {10571, 10581}, tl_state_manager_.getCurrentState());
    updateRandomTrafficLightColor({10610, 10598}, {10604}, tl_state_manager_.getCurrentState());
    updateRandomTrafficLightColor({10342}, {10343}, tl_state_manager_.getCurrentState());




    
    // 特定のlane_idの200m以内になったら、横断歩道歩行者をspawn（速度は毎回ランダム、人数はspawnで抽選、数秒ごとにspawnを止める）

    // 特定のlane_idの200m以内になったら、交差点の奥から車両が来る。（速度は毎回ランダム、数秒ごとにspawnを止める）

  }

  void onInitialize() override
  {

    // Initialize random seed
    srand(time(0));

    // api_.setVerbose(true);
    params_ = param_listener_->get_params();

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
