#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// YAML-CPP
#include <yaml-cpp/yaml.h>
#include <fstream>

// Custom Msgs
#include "laser_scan_integrator_msg/msg/line_segments.hpp"
#include "laser_scan_integrator_msg/msg/line_segment.hpp"

using namespace std::chrono_literals;

/**
 * @brief Struktur zum Speichern der Maschinen-Daten aus YAML.
 */
struct MachineInfo
{
  std::string name;
  double x;
  double y;
  double z;
  double qx;
  double qy;
  double qz;
  double qw;
};

/**
 * @brief Prüft, ob ein gegebener LineSegment zu einer Maschine gehört.
 */
bool belongsToMachine(
  const laser_scan_integrator_msg::msg::LineSegment &segment,
  const geometry_msgs::msg::TransformStamped &machine_tf,
  double position_tolerance,
  double orientation_tolerance)
{
  // Unverändert aus vorherigem Code übernommen
  const auto &p1 = segment.end_point1;
  const auto &p2 = segment.end_point2;
  
  geometry_msgs::msg::Point segment_mid;
  segment_mid.x = 0.5 * (p1.x + p2.x);
  segment_mid.y = 0.5 * (p1.y + p2.y);
  segment_mid.z = 0.5 * (p1.z + p2.z);

  double machine_x = machine_tf.transform.translation.x;
  double machine_y = machine_tf.transform.translation.y;

  tf2::Quaternion q_machine;
  tf2::fromMsg(machine_tf.transform.rotation, q_machine);

  double roll, pitch, yaw;
  tf2::Matrix3x3(q_machine).getRPY(roll, pitch, yaw);

  double dx = segment_mid.x - machine_x;
  double dy = segment_mid.y - machine_y;
  double dist = std::sqrt(dx * dx + dy * dy);

  if (dist > position_tolerance) {
    return false;
  }

  double orient_diff = std::fabs(segment.bearing - yaw);
  if (orient_diff > M_PI) {
    orient_diff = 2.0 * M_PI - orient_diff;
  }
  return orient_diff <= orientation_tolerance;
}

/**
 * @brief MapperNode, der:
 *   - Maschinen-Daten aus einer YAML-Datei lädt
 *   - Toleranzen aus einer separaten YAML-Datei lädt
 */
class MapperNode : public rclcpp::Node
{
public:
  MapperNode(const std::string &machines_yaml, const std::string &tolerance_yaml)
  : Node("mapper"),
    tf_broadcaster_(this)
  {
    // YAML-Dateien laden
    loadMachinesFromYAML(machines_yaml);
    loadTolerancesFromYAML(tolerance_yaml);

    // Subscriber für die Segmente
    line_segments_sub_ = this->create_subscription<laser_scan_integrator_msg::msg::LineSegments>(
      "line_segments",
      10,
      std::bind(&MapperNode::lineSegmentsCallback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "MapperNode gestartet. Abonniert 'line_segments'.");
  }

private:
  void loadMachinesFromYAML(const std::string &yaml_path)
  {
    YAML::Node config = YAML::LoadFile(yaml_path);
    if (config["machines"]) {
      for (const auto &m : config["machines"]) {
        MachineInfo info;
        info.name = m["name"].as<std::string>();
        info.x    = m["translation"]["x"].as<double>();
        info.y    = m["translation"]["y"].as<double>();
        info.z    = m["translation"]["z"].as<double>();
        info.qx   = m["rotation"]["x"].as<double>();
        info.qy   = m["rotation"]["y"].as<double>();
        info.qz   = m["rotation"]["z"].as<double>();
        info.qw   = m["rotation"]["w"].as<double>();
        machine_infos_.push_back(info);
      }
    }
  }

  void loadTolerancesFromYAML(const std::string &yaml_path)
  {
    YAML::Node config = YAML::LoadFile(yaml_path);
    if (config["tolerance"]) {
      position_tolerance_ = config["tolerance"]["position_tolerance"].as<double>();
      orientation_tolerance_ = config["tolerance"]["orientation_tolerance"].as<double>();
    }
  }

  void lineSegmentsCallback(const laser_scan_integrator_msg::msg::LineSegments::SharedPtr msg)
  {
    for (const auto &segment : msg->segments) {
      for (const auto &machine : machine_infos_) {
        geometry_msgs::msg::TransformStamped machine_tf;
        machine_tf.header.frame_id = "map";
        machine_tf.child_frame_id = machine.name;
        machine_tf.header.stamp = this->now();

        machine_tf.transform.translation.x = machine.x;
        machine_tf.transform.translation.y = machine.y;
        machine_tf.transform.translation.z = machine.z;

        machine_tf.transform.rotation.x = machine.qx;
        machine_tf.transform.rotation.y = machine.qy;
        machine_tf.transform.rotation.z = machine.qz;
        machine_tf.transform.rotation.w = machine.qw;

        if (belongsToMachine(segment, machine_tf, position_tolerance_, orientation_tolerance_)) {
          geometry_msgs::msg::TransformStamped corrected_tf = machine_tf;
          corrected_tf.child_frame_id = "machine_corrected_" + machine.name;
          corrected_tf.transform.translation.x += 0.05;

          tf_broadcaster_.sendTransform(corrected_tf);
          RCLCPP_INFO(this->get_logger(), "LineSegment passt zu [%s]. Neue TF '%s' veröffentlicht.", machine.name.c_str(), corrected_tf.child_frame_id.c_str());
        }
      }
    }
  }

  tf2_ros::TransformBroadcaster tf_broadcaster_;
  rclcpp::Subscription<laser_scan_integrator_msg::msg::LineSegments>::SharedPtr line_segments_sub_;

  std::vector<MachineInfo> machine_infos_;
  double position_tolerance_ = 0.2;
  double orientation_tolerance_ = 0.2;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::string machines_yaml = "../config/machines.yaml";
  std::string tolerance_yaml = "../config/tolerance.yaml";

  if (argc > 2) {
    machines_yaml = argv[1];
    tolerance_yaml = argv[2];
  }

  auto node = std::make_shared<MapperNode>(machines_yaml, tolerance_yaml);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
