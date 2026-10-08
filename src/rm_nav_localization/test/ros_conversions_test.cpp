#include "rm_nav_localization/ros_conversions.hpp"
#include <iostream>
#include <limits>
#include <stdexcept>

void require(bool condition, const char * message)
{
  if (!condition) throw std::runtime_error(message);
}

template <typename F> void rejects(F action)
{
  bool rejected = false;
  try { action(); } catch (const std::invalid_argument &) { rejected = true; }
  require(rejected, "Malformed input was accepted");
}

int main()
{
  using namespace rm_nav_localization;
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.width = cloud.height = 2;
  cloud.point_step = 24; cloud.row_step = 56; cloud.is_bigendian = true;
  cloud.data.resize(112);
  const std::array<std::string, 3> names{"x", "y", "z"};
  for (int axis = 0; axis < 3; ++axis) {
    sensor_msgs::msg::PointField field;
    field.name = names[axis]; field.offset = axis * 8; field.datatype = 8; field.count = 1;
    cloud.fields.push_back(field);
  }
  for (int row = 0; row < 2; ++row) for (int col = 0; col < 2; ++col) for (int axis = 0; axis < 3; ++axis) {
    const double value = row * 100 + col * 10 + axis + 0.25;
    std::array<std::uint8_t, 8> bytes;
    std::memcpy(bytes.data(), &value, 8);
    const std::uint16_t probe = 1;
    if (reinterpret_cast<const std::uint8_t *>(&probe)[0] == 1) std::reverse(bytes.begin(), bytes.end());
    std::memcpy(cloud.data.data() + row * 56 + col * 24 + axis * 8, bytes.data(), 8);
  }
  const auto points = read_xyz(cloud, 4);
  require(points.size() == 4 && points[3].z() == 112.25, "Organized endian/padding conversion failed");
  rejects([&]() { read_xyz(cloud, 3); });
  auto bad = cloud; bad.data.resize(111);
  rejects([&]() { read_xyz(bad, 4); });
  bad = cloud; bad.fields[0].offset = 23;
  rejects([&]() { read_xyz(bad, 4); });
  bad = cloud; bad.fields.pop_back();
  rejects([&]() { read_xyz(bad, 4); });
  bad = cloud; bad.fields[0].count = 2;
  rejects([&]() { read_xyz(bad, 4); });
  geometry_msgs::msg::Transform transform;
  transform.rotation.w = 0;
  rejects([&]() { from_ros_transform(transform); });
  transform.rotation.w = 1;
  const auto roundtrip = to_ros_transform(from_ros_transform(transform));
  require(roundtrip.rotation.w == 1, "Transform roundtrip failed");
  transform.translation.x = std::numeric_limits<double>::quiet_NaN();
  rejects([&]() { from_ros_transform(transform); });
  std::cout << "PASS PointCloud2 storage, endian, padding and transform checks\n";
}
