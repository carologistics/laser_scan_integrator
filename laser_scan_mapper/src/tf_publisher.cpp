// Copyright (c) 2025 Carologistics
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

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

// YAML-CPP-Header
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <fstream>
#include <string>
#include <vector>

/**
 * Struktur zum Speichern einer Maschinen-Definition (Name + Pose)
 */
struct MachineDefinition {
  std::string name;
  double tx, ty, tz;
  double rx, ry, rz, rw;
};

void normalizeQuaternion(double &x, double &y, double &z, double &w) {
  double norm = std::sqrt(x * x + y * y + z * z + w * w);
  if (norm > 0.0) {
    x /= norm;
    y /= norm;
    z /= norm;
    w /= norm;
  }
}

class TfPublisherNode : public rclcpp::Node {
public:
  TfPublisherNode() : Node("tf_publisher") {
    // Statische TF-Broadcaster-Instanz
    static_broadcaster_ =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // Standard-Pfad zur YAML-Datei
    std::string yaml_path =
        ament_index_cpp::get_package_share_directory("laser_scan_mapper") +
        "/config/machines.yaml";

    // Maschinen-Definitionen aus YAML einlesen
    std::vector<MachineDefinition> machines = loadMachinesFromYAML(yaml_path);

    // Erzeuge und sende die statischen TFs
    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    transforms.reserve(machines.size());

    // Wir nehmen an, dass alle Maschinen in Bezug auf einen "world"-Frame
    // liegen (oder "map", "odom", o.ä. - je nach Konvention)
    std::string parent_frame_id = "map";

    for (const auto &m : machines) {
      geometry_msgs::msg::TransformStamped ts;
      ts.header.stamp = this->now();
      ts.header.frame_id = "map";
      ts.child_frame_id = m.name;

      ts.transform.translation.x = m.tx;
      ts.transform.translation.y = m.ty;
      ts.transform.translation.z = m.tz;

      // Normalisierung des Quaternions
      double rx = m.rx, ry = m.ry, rz = m.rz, rw = m.rw;
      normalizeQuaternion(rx, ry, rz, rw);

      ts.transform.rotation.x = rx;
      ts.transform.rotation.y = ry;
      ts.transform.rotation.z = rz;
      ts.transform.rotation.w = rw;

      transforms.push_back(ts);
    }

    // Sende alle Transforms auf einmal
    static_broadcaster_->sendTransform(transforms);

    RCLCPP_INFO(this->get_logger(),
                "Veröffentliche %zu statische Maschinen-Frames relativ zu "
                "'%s'. Node läuft ...",
                transforms.size(), parent_frame_id.c_str());
  }

private:
  /**
   * Liest die Maschinen-Definitionen aus einer YAML-Datei ein.
   * @param path Pfad zur YAML-Datei
   * @return Vektor mit allen gelesenen Maschinen (Name + Pose)
   */
  std::vector<MachineDefinition> loadMachinesFromYAML(const std::string &path) {
    std::vector<MachineDefinition> machines;

    try {
      YAML::Node config = YAML::LoadFile(path);
      if (!config["machines"]) {
        RCLCPP_ERROR(this->get_logger(),
                     "Keine 'machines'-Sektion in der YAML-Datei gefunden: %s",
                     path.c_str());
        return machines;
      }

      // Liste von Maschinen durchgehen
      for (const auto &machine_node : config["machines"]) {
        MachineDefinition mdef;
        mdef.name = machine_node["name"].as<std::string>();

        auto trans = machine_node["translation"];
        mdef.tx = trans["x"].as<double>();
        mdef.ty = trans["y"].as<double>();
        mdef.tz = trans["z"].as<double>();

        auto rot = machine_node["rotation"];
        mdef.rx = rot["x"].as<double>();
        mdef.ry = rot["y"].as<double>();
        mdef.rz = rot["z"].as<double>();
        mdef.rw = rot["w"].as<double>();

        machines.push_back(mdef);
      }
    } catch (const YAML::Exception &e) {
      RCLCPP_ERROR(this->get_logger(),
                   "Fehler beim Laden der YAML-Datei '%s': %s", path.c_str(),
                   e.what());
    }

    return machines;
  }

private:
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  // Node erzeugen
  auto node = std::make_shared<TfPublisherNode>();

  // Node am Leben halten
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
