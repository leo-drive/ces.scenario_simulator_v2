// Copyright 2015-2020 Tier IV, Inc. All rights reserved.
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

#ifndef TRAFFIC_SIMULATOR__ENTITY__ENTITY_MANAGER_HPP_
#define TRAFFIC_SIMULATOR__ENTITY__ENTITY_MANAGER_HPP_

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <boost/optional.hpp>
#include <memory>
#include <openscenario_msgs/msg/bounding_box.hpp>
#include <openscenario_msgs/msg/driver_model.hpp>
#include <openscenario_msgs/msg/entity_status_with_trajectory_array.hpp>
#include <openscenario_msgs/msg/vehicle_parameters.hpp>
#include <rclcpp/rclcpp.hpp>
#include <scenario_simulator_exception/exception.hpp>
#include <stdexcept>
#include <string>
#include <traffic_simulator/api/configuration.hpp>
#include <traffic_simulator/entity/ego_entity.hpp>
#include <traffic_simulator/entity/entity_base.hpp>
#include <traffic_simulator/entity/misc_object_entity.hpp>
#include <traffic_simulator/entity/pedestrian_entity.hpp>
#include <traffic_simulator/entity/vehicle_entity.hpp>
#include <traffic_simulator/hdmap_utils/hdmap_utils.hpp>
#include <traffic_simulator/traffic/traffic_sink.hpp>
#include <traffic_simulator/traffic_lights/traffic_light_manager.hpp>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>

namespace traffic_simulator
{
namespace entity
{
class LaneletMarkerQoS : public rclcpp::QoS
{
public:
  explicit LaneletMarkerQoS(std::size_t depth = 1) : rclcpp::QoS(depth) { transient_local(); }
};

class EntityMarkerQoS : public rclcpp::QoS
{
public:
  explicit EntityMarkerQoS(std::size_t depth = 100) : rclcpp::QoS(depth) {}
};

class EntityManager
{
  Configuration configuration;

  tf2_ros::StaticTransformBroadcaster broadcaster_;
  tf2_ros::TransformBroadcaster base_link_broadcaster_;

  const rclcpp::Clock::SharedPtr clock_ptr_;

  std::unordered_map<std::string, std::unique_ptr<traffic_simulator::entity::EntityBase>> entities_;

  double step_time_;

  double current_time_;

  using EntityStatusWithTrajectoryArray = openscenario_msgs::msg::EntityStatusWithTrajectoryArray;
  const rclcpp::Publisher<EntityStatusWithTrajectoryArray>::SharedPtr entity_status_array_pub_ptr_;

  using MarkerArray = visualization_msgs::msg::MarkerArray;
  const rclcpp::Publisher<MarkerArray>::SharedPtr lanelet_marker_pub_ptr_;

  const std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils_ptr_;

  MarkerArray markers_raw_;

  const std::shared_ptr<TrafficLightManager> traffic_light_manager_ptr_;

public:
  template <typename Node>
  auto getOrigin(Node & node) const
  {
    geographic_msgs::msg::GeoPoint origin;
    {
      node.declare_parameter("origin_latitude", 0.0);
      node.declare_parameter("origin_longitude", 0.0);
      node.get_parameter("origin_latitude", origin.latitude);
      node.get_parameter("origin_longitude", origin.longitude);
    }

    return origin;
  }

  template <class NodeT, class AllocatorT = std::allocator<void>>
  explicit EntityManager(NodeT && node, const Configuration & configuration)
  : configuration(configuration),
    broadcaster_(node),
    base_link_broadcaster_(node),
    clock_ptr_(node->get_clock()),
    entity_status_array_pub_ptr_(rclcpp::create_publisher<EntityStatusWithTrajectoryArray>(
      node, "entity/status", EntityMarkerQoS(),
      rclcpp::PublisherOptionsWithAllocator<AllocatorT>())),
    lanelet_marker_pub_ptr_(rclcpp::create_publisher<MarkerArray>(
      node, "lanelet/marker", LaneletMarkerQoS(),
      rclcpp::PublisherOptionsWithAllocator<AllocatorT>())),
    hdmap_utils_ptr_(std::make_shared<hdmap_utils::HdMapUtils>(
      configuration.lanelet2_map_path(), getOrigin(*node))),
    markers_raw_(hdmap_utils_ptr_->generateMarker()),
    traffic_light_manager_ptr_(std::make_shared<TrafficLightManager>(
      hdmap_utils_ptr_,
      rclcpp::create_publisher<MarkerArray>(node, "traffic_light/marker", LaneletMarkerQoS()),
      rclcpp::create_publisher<autoware_perception_msgs::msg::TrafficLightStateArray>(
        node, "/awapi/traffic_light/put/traffic_light_status", rclcpp::QoS(10).transient_local()),
      clock_ptr_))
  {
    updateHdmapMarker();
  }

  ~EntityManager() = default;

public:
#define DEFINE_SET_TRAFFIC_LIGHT(NAME)                                               \
  template <typename... Ts>                                                          \
  decltype(auto) setTrafficLight##NAME(Ts &&... xs)                                  \
  {                                                                                  \
    return traffic_light_manager_ptr_->set##NAME(std::forward<decltype(xs)>(xs)...); \
  }                                                                                  \
  static_assert(true, "")

  DEFINE_SET_TRAFFIC_LIGHT(Arrow);
  DEFINE_SET_TRAFFIC_LIGHT(ArrowPhase);
  DEFINE_SET_TRAFFIC_LIGHT(Color);
  DEFINE_SET_TRAFFIC_LIGHT(ColorPhase);

#undef DEFINE_SET_TRAFFIC_LIGHT

#define DEFINE_GET_TRAFFIC_LIGHT(NAME)                                               \
  template <typename... Ts>                                                          \
  decltype(auto) getTrafficLight##NAME(Ts &&... xs)                                  \
  {                                                                                  \
    return traffic_light_manager_ptr_->get##NAME(std::forward<decltype(xs)>(xs)...); \
  }                                                                                  \
  static_assert(true, "")

  DEFINE_GET_TRAFFIC_LIGHT(Color);
  DEFINE_GET_TRAFFIC_LIGHT(Arrow);

#undef DEFINE_GET_TRAFFIC_LIGHT

#define FORWARD_TO_HDMAP_UTILS(NAME)                                  \
  template <typename... Ts>                                           \
  decltype(auto) NAME(Ts &&... xs) const                              \
  {                                                                   \
    return hdmap_utils_ptr_->NAME(std::forward<decltype(xs)>(xs)...); \
  }                                                                   \
  static_assert(true, "")

  FORWARD_TO_HDMAP_UTILS(toLaneletPose);
  // FORWARD_TO_HDMAP_UTILS(toMapPose);

#undef FORWARD_TO_HDMAP_UTILS

#define FORWARD_TO_ENTITY(IDENTIFIER, ...)                                     \
  template <typename... Ts>                                                    \
  decltype(auto) IDENTIFIER(const std::string & name, Ts &&... xs) __VA_ARGS__ \
  try {                                                                        \
    return entities_.at(name)->IDENTIFIER(std::forward<decltype(xs)>(xs)...);  \
  } catch (const std::out_of_range &) {                                        \
    THROW_SEMANTIC_ERROR("entity : ", name, "does not exist");                 \
  }                                                                            \
  static_assert(true, "")

  FORWARD_TO_ENTITY(engage, );
  FORWARD_TO_ENTITY(getBoundingBox, const);
  FORWARD_TO_ENTITY(getCurrentAction, const);
  FORWARD_TO_ENTITY(getEntityType, const);
  FORWARD_TO_ENTITY(getEntityStatusBeforeUpdate, const);
  FORWARD_TO_ENTITY(getLinearJerk, const);
  FORWARD_TO_ENTITY(getRouteLanelets, );
  FORWARD_TO_ENTITY(getStandStillDuration, const);
  FORWARD_TO_ENTITY(getVehicleParameters, const);
  FORWARD_TO_ENTITY(getVehicleCommand, const);
  FORWARD_TO_ENTITY(ready, const);
  FORWARD_TO_ENTITY(requestAcquirePosition, );
  FORWARD_TO_ENTITY(requestAssignRoute, );
  FORWARD_TO_ENTITY(requestLaneChange, );
  FORWARD_TO_ENTITY(requestWalkStraight, );
  FORWARD_TO_ENTITY(setDriverModel, );

#undef FORWARD_TO_SPECIFIED_ENTITY

  void setTargetSpeed(const std::string & name, double target_speed, bool continuous);

  openscenario_msgs::msg::EntityStatus updateNpcLogic(
    const std::string & name,
    const std::unordered_map<std::string, openscenario_msgs::msg::EntityType> & type_list);

  void broadcastBaseLinkTransform();

  void broadcastEntityTransform();

  void broadcastTransform(
    const geometry_msgs::msg::PoseStamped & pose, const bool static_transform = true);

  bool checkCollision(const std::string & name0, const std::string & name1);

  bool despawnEntity(const std::string & name);

  bool entityExists(const std::string & name);

  // TODO (yamacir-kit) Rename to 'hasEntityStatus'
  bool entityStatusSet(const std::string & name) const;

  auto getBoundingBoxDistance(const std::string & from, const std::string & to)
    -> boost::optional<double>;

  auto getConflictingEntityOnRouteLanelets(const std::string & name, const double horizon)
    -> std::vector<std::int64_t>;

  auto getCurrentTime() const noexcept -> double;

  auto getDistanceToCrosswalk(const std::string & name, const std::int64_t target_crosswalk_id)
    -> boost::optional<double>;

  auto getDistanceToStopLine(const std::string & name, const std::int64_t target_stop_line_id)
    -> boost::optional<double>;

  auto getEntityNames() const -> const std::vector<std::string>;

  auto getEntityStatus(const std::string & name) const
    -> const boost::optional<openscenario_msgs::msg::EntityStatus>;

  auto getEntityTypeList() const
    -> const std::unordered_map<std::string, openscenario_msgs::msg::EntityType>;

  auto getHdmapUtils() -> const std::shared_ptr<hdmap_utils::HdMapUtils> &;

  auto getLaneletPose(const std::string & name)
    -> boost::optional<openscenario_msgs::msg::LaneletPose>;

  auto getLongitudinalDistance(
    const std::string & from, const std::string & to, const double max_distance = 100)
    -> boost::optional<double>;

  auto getMapPose(const std::string & entity_name) -> geometry_msgs::msg::Pose;
  auto getMapPose(
    const std::string & reference_entity_name, const geometry_msgs::msg::Pose & relative_pose)
    -> geometry_msgs::msg::Pose;

  auto getNumberOfEgo() const -> std::size_t;

  auto getObstacle(const std::string & name) -> boost::optional<openscenario_msgs::msg::Obstacle>;

  // clang-format off
  auto getRelativePose(const geometry_msgs::msg::Pose & from, const geometry_msgs::msg::Pose & to) const -> geometry_msgs::msg::Pose;
  auto getRelativePose(const geometry_msgs::msg::Pose & from, const std::string              & to)       -> geometry_msgs::msg::Pose;
  auto getRelativePose(const std::string              & from, const geometry_msgs::msg::Pose & to)       -> geometry_msgs::msg::Pose;
  auto getRelativePose(const std::string              & from, const std::string              & to)       -> geometry_msgs::msg::Pose;
  // clang-format on

  auto getStepTime() const noexcept -> double;

  auto getSValueInRoute(const std::string & name, const std::vector<std::int64_t> & route)
    -> boost::optional<double>;

  auto getWaypoints(const std::string & name) -> openscenario_msgs::msg::WaypointsArray;

  bool isEgo(const std::string & name) const;

  const std::string getEgoName() const;

  bool isInLanelet(const std::string & name, const std::int64_t lanelet_id, const double tolerance);

  bool isStopping(const std::string & name) const;

  bool reachPosition(
    const std::string & name, const geometry_msgs::msg::Pose & target_pose,
    const double tolerance) const;
  bool reachPosition(
    const std::string & name, const std::int64_t lanelet_id, const double s, const double offset,
    const double tolerance) const;
  bool reachPosition(
    const std::string & name, const std::string & target_name, const double tolerance) const;

  void requestLaneChange(const std::string & name, const Direction & direction);

  bool setEntityStatus(const std::string & name, openscenario_msgs::msg::EntityStatus status);

  void setVerbose(const bool verbose);

  template <typename Entity, typename... Ts>
  auto spawnEntity(const std::string & name, Ts &&... xs)
  {
    const auto result =
      entities_.emplace(name, std::make_unique<Entity>(name, std::forward<decltype(xs)>(xs)...));
    if (result.second) {
      result.first->second->setHdMapUtils(hdmap_utils_ptr_);
      result.first->second->setTrafficLightManager(traffic_light_manager_ptr_);
      return result.second;
    } else {
      THROW_SEMANTIC_ERROR("entity : ", name, " is already exists.");
    }
  }

  auto toMapPose(const openscenario_msgs::msg::LaneletPose &) const
    -> const geometry_msgs::msg::Pose;

  void update(const double current_time, const double step_time);

  void updateHdmapMarker();
};
}  // namespace entity
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__ENTITY__ENTITY_MANAGER_HPP_
