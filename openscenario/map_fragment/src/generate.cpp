#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>

#define PRINT(...) std::cout << #__VA_ARGS__ " = " << std::boolalpha << (__VA_ARGS__) << std::endl

auto makePoint3d(double x, double y, double z, double elevation = 0.0)
{
  static lanelet::Id id = 0;
  auto point = lanelet::Point3d(++id, x, y, z);
  point.attributes()["ele"] = elevation;
  return point;
}

auto makeLineString3d(
  const lanelet::Point3d & origin, double length, double radius, std::size_t resolution = 100)
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

auto makeLanelet(double length = 1000, double width = 10, double curvature = -0.002)
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
    ++id, makeLineString3d(p1, length + p1_offset, p1_radius),
    makeLineString3d(p2, length + p2_offset, p2_radius));
  lane.attributes()["subtype"] = "road";
  return lane;
}

int main()
{
  auto map = lanelet::LaneletMap();

  map.add(makeLanelet());

  lanelet::write(
    "/tmp/lanelet2_map.osm", map,
    lanelet::projection::UtmProjector(lanelet::Origin({35.624285, 139.742570})));

  return EXIT_SUCCESS;
}
