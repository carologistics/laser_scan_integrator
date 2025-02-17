#include <map>
#include <string>
#include <memory>
#include <cmath>
#include <chrono>
#include <thread>
#include <vector>
#include <fstream>
#include <iostream>

// Eigen-Bibliothek
#include <Eigen/Dense>
#include <Eigen/Geometry>

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "laser_scan_integrator_msg/msg/line_segments.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

// Marker
#include "visualization_msgs/msg/marker.hpp"

// YAML-CPP
#include <yaml-cpp/yaml.h>

// CMake: Zum Ermitteln des Share-Verzeichnisses
#include "ament_index_cpp/get_package_share_directory.hpp"

// WICHTIG: Header für tf2_geometry_msgs inkludieren, damit Template-Instanzen erzeugt werden
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

class MapperNode : public rclcpp::Node
{
public:
  MapperNode()
  : Node("mapper"),
    tf_buffer_(std::make_unique<tf2_ros::Buffer>(this->get_clock())),
    tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
  {
    // YAML laden (über den Share-Pfad)
    std::string share_dir = ament_index_cpp::get_package_share_directory("laser_scan_mapper");
    std::string yaml_file = share_dir + "/config/mapper_params.yaml";
    RCLCPP_INFO(this->get_logger(), "Lade Maschinenliste aus YAML: %s", yaml_file.c_str());

    if (!loadMachineNamesFromYaml(yaml_file, machine_names_)) {
      RCLCPP_ERROR(this->get_logger(),
                   "Konnte Maschine-Namen aus %s nicht laden. Node wird beendet.",
                   yaml_file.c_str());
      rclcpp::shutdown();
      return;
    }

    // TF-Broadcaster initialisieren
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Marker-Publisher für segmentierte Linien (Topic: /mapped_segments)
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("mapped_segments", 10);

    // Marker-Publisher für Maschinen-Rechtecke (Topic: /machine_markers)
    machine_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("machine_markers", 10);

    // Warten auf TF-Daten
    RCLCPP_INFO(this->get_logger(), "Warte auf TF-Daten...");
    std::this_thread::sleep_for(3s);

    // Maschinen-Transforms laden (im Map-Frame) und Marker sofort publizieren
    if (!initializeMachineTransforms()) {
      RCLCPP_ERROR(this->get_logger(), "Maschinen-Frames konnten nicht initialisiert werden.");
      rclcpp::shutdown();
      return;
    }

    // Subscriber für LineSegments
    sub_segments_ = this->create_subscription<laser_scan_integrator_msg::msg::LineSegments>(
      "robotinobase2/line_segments",
      10,
      std::bind(&MapperNode::segmentsCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Mapper gestartet.");
  }

private:
  // Maschinenbreite und Toleranzen (SI-Einheiten)
  // (Hinweis: Da y nun die Fahrtrichtung ist, entspricht machine_width_ der lateralen Ausdehnung entlang der x-Achse.)
  const double machine_width_      = 0.35;  // 35 cm
  const double position_tolerance_ = 0.3;   // z. B. 30 cm
  const double angle_tolerance_    = 3;     // z. B. 3 rad

  // Für Maschinen-Rechteck:
  // (Angepasst: Die Länge (Fahrtrichtung) liegt nun entlang der y-Achse und die Breite entlang der x-Achse.)
  const double machine_length_ = 0.70;  // z. B. 35 cm (Fahrtrichtung, y-Achse)
  const double machine_height_ = 0.35;  // z. B. 70 cm (seitliche Ausdehnung, x-Achse)
  // Zähler für die Segmentmarker (Deklaration ohne Initialisierung!)
  static int global_marker_id;

  // Gelesene Maschinen-Namen aus YAML
  std::vector<std::string> machine_names_;

  // Maschinen-Transforms (im Map-Frame)
  std::map<std::string, geometry_msgs::msg::TransformStamped> machine_transforms_;

  // TF
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Marker-Publisher für segmentierte Linien (Topic: /mapped_segments)
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  // Marker-Publisher für Maschinen-Rechtecke (Topic: /machine_markers)
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr machine_marker_pub_;

  // Subscriber für LineSegments
  rclcpp::Subscription<laser_scan_integrator_msg::msg::LineSegments>::SharedPtr sub_segments_;

  // Funktion zum Laden der Maschinen-Namen aus YAML
  bool loadMachineNamesFromYaml(const std::string &file_path, std::vector<std::string> &out_names)
  {
    try {
      YAML::Node config = YAML::LoadFile(file_path);
      // Erwartete Struktur: mapper -> ros__parameters -> machine_names
      if (config["mapper"] &&
          config["mapper"]["ros__parameters"] &&
          config["mapper"]["ros__parameters"]["machine_names"])
      {
        auto machine_node = config["mapper"]["ros__parameters"]["machine_names"];
        for (std::size_t i = 0; i < machine_node.size(); ++i) {
          out_names.push_back(machine_node[i].as<std::string>());
        }
        RCLCPP_INFO(this->get_logger(),
                    "Es wurden %zu Maschinen-Namen aus YAML geladen.",
                    out_names.size());
        return !out_names.empty();
      } else {
        RCLCPP_ERROR(this->get_logger(), "Falsche YAML-Struktur in %s", file_path.c_str());
      }
    } catch (const YAML::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "YAML Parse Error: %s", e.what());
    }
    return false;
  }

  // Methode zum Erstellen und sofortigen Publizieren eines Maschinen-Markers
  void publishSingleMachineMarker(const std::string &machine_frame_id,
                                  const geometry_msgs::msg::TransformStamped &transform_stamped)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "machine_rectangles";

    // Eindeutige ID für jeden Maschinen-Marker
    static int machine_marker_id = 0;
    marker.id = machine_marker_id++;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Pose aus dem Transform übernehmen
    marker.pose.position.x = transform_stamped.transform.translation.x;
    marker.pose.position.y = transform_stamped.transform.translation.y;
    marker.pose.position.z = transform_stamped.transform.translation.z;
    marker.pose.orientation = transform_stamped.transform.rotation;

    // Skalierung: (Angepasst: Länge entlang y, Breite entlang x)
    marker.scale.x = machine_height_;
    marker.scale.y = machine_length_;
    marker.scale.z = 0.05;

    // Farbe: Halbtransparentes Blau
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 0.8;

    // Unbegrenzte Lebensdauer
    marker.lifetime = rclcpp::Duration(0, 0);

    machine_marker_pub_->publish(marker);
    RCLCPP_INFO(this->get_logger(),
                "Maschinenmarker für %s veröffentlicht: (%.3f, %.3f)",
                machine_frame_id.c_str(),
                transform_stamped.transform.translation.x,
                transform_stamped.transform.translation.y);
  }

  // Maschinen-Transforms laden (im Map-Frame) und Marker sofort publizieren
  bool initializeMachineTransforms()
  {
    if (machine_names_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Keine Maschinen-Namen vorhanden. Bitte YAML prüfen!");
      return false;
    }

    for (const auto &machine_frame_id : machine_names_) {
      bool transform_success = false;
      for (int attempt = 1; attempt <= 10; ++attempt) {
        try {
          RCLCPP_INFO(this->get_logger(), 
                      "Versuch %d/10: Lade Transform für %s ...", 
                      attempt, machine_frame_id.c_str());

          auto transform_stamped =
            tf_buffer_->lookupTransform("map", machine_frame_id, tf2::TimePointZero);
          machine_transforms_[machine_frame_id] = transform_stamped;

          RCLCPP_INFO(this->get_logger(),
                      "Transform für %s geladen: (%.3f, %.3f)",
                      machine_frame_id.c_str(),
                      transform_stamped.transform.translation.x,
                      transform_stamped.transform.translation.y);

          transform_success = true;

          // Marker sofort veröffentlichen
          publishSingleMachineMarker(machine_frame_id, transform_stamped);
          break;
        }
        catch (const tf2::TransformException &ex) {
          RCLCPP_WARN(this->get_logger(),
                      "Transform für %s fehlgeschlagen (Versuch %d): %s",
                      machine_frame_id.c_str(), attempt, ex.what());
          if (attempt == 10) {
            RCLCPP_ERROR(this->get_logger(),
                         "Transform für %s konnte nach 10 Versuchen nicht geladen werden.",
                         machine_frame_id.c_str());
            return false;
          }
          std::this_thread::sleep_for(1s);
        }
      }
      if (!transform_success) {
        return false;
      }
    }

    RCLCPP_INFO(this->get_logger(), "Alle Maschinen-Frames erfolgreich geladen.");
    return true;
  }

  // Segment-Callback: Transformation und Berechnung im Map-Frame für segmentierte Linien
  void segmentsCallback(const laser_scan_integrator_msg::msg::LineSegments::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), 
                "Empfange LineSegments mit %zu Segment(en).", msg->segments.size());

    // Für jeden in machine_transforms_ enthaltenen Maschinen-Frame
    for (const auto &entry : machine_transforms_) {
      const std::string &machine_frame_id = entry.first;
      const auto &machine_transform = entry.second;

      // Maschinenmittelpunkt und angepasste Orientierung (y ist nun Fahrtrichtung)
      double mx = machine_transform.transform.translation.x;
      double my = machine_transform.transform.translation.y;
      // Umrechnung: Da bisher angenommen wurde, dass x vorwärts ist, wird hier um 90° (M_PI/2) korrigiert.
      double machine_yaw = getYawFromQuaternion(machine_transform.transform.rotation)- M_PI/2.0;

      Eigen::Vector2d center(mx, my);
      double half_width = machine_width_ * 0.5;
      Eigen::Rotation2Dd R(machine_yaw);

      // Erwartete Kanten im Map-Frame: 
      // Statt entlang der lokalen y-Achse (bei x als Fahrtrichtung) erfolgt nun die Verschiebung
      // entlang der lokalen x-Achse (da y Fahrtrichtung ist).
      Eigen::Vector2d expected_edge1 = center + R * Eigen::Vector2d( half_width, 0);
      Eigen::Vector2d expected_edge2 = center + R * Eigen::Vector2d(-half_width, 0);

      for (size_t idx = 0; idx < msg->segments.size(); ++idx) {
        const auto &segment = msg->segments[idx];

        // Transformation der Segment-Endpunkte in den Map-Frame
        geometry_msgs::msg::PointStamped pt1_in, pt2_in, pt1_map, pt2_map;
        pt1_in.header.stamp = this->now();
        pt1_in.header.frame_id = "robotinobase2/laser_link";
        pt1_in.point = segment.end_point1;

        pt2_in.header = pt1_in.header;
        pt2_in.point = segment.end_point2;

        try {
          pt1_map = tf_buffer_->transform(pt1_in, "map", tf2::durationFromSec(1.0));
          pt2_map = tf_buffer_->transform(pt2_in, "map", tf2::durationFromSec(1.0));
        } catch (const tf2::TransformException &ex) {
          RCLCPP_WARN(this->get_logger(), 
                      "Fehler beim Transformieren der Segmentpunkte: %s", ex.what());
          continue;
        }

        // Mittelwert der beiden Endpunkte
        Eigen::Vector2d measured(
          0.5 * (pt1_map.point.x + pt2_map.point.x),
          0.5 * (pt1_map.point.y + pt2_map.point.y)
        );

        // Abstände zu den erwarteten Kanten
        double d1 = (measured - expected_edge1).norm();
        double d2 = (measured - expected_edge2).norm();
        double min_distance = std::min(d1, d2);

        // Winkelabweichung: Auch hier erfolgt die Anpassung um 90° (M_PI/2)
        double dtheta = (segment.bearing - M_PI/2.0) - machine_yaw;
        while (dtheta > M_PI)  dtheta -= 2 * M_PI;
        while (dtheta < -M_PI) dtheta += 2 * M_PI;

        // Prüfung der Toleranzen
        if (min_distance > position_tolerance_ || 
            std::fabs(dtheta) > angle_tolerance_)
        {
          RCLCPP_INFO(this->get_logger(),
                      "Segment [%zu] verworfen für %s: dist=%.3f, dtheta=%.3f",
                      idx, machine_frame_id.c_str(), min_distance, dtheta);
          continue;
        }

        // Neue Maschinenposition berechnen
        Eigen::Vector2d chosen_edge = (d1 < d2) ? expected_edge1 : expected_edge2;
        double new_yaw = segment.bearing - M_PI/2.0;

        // Richte die Maschine neu aus
        Eigen::Rotation2Dd R_new(new_yaw);
        // Bestimme den seitlichen (linken) Vektor: Bei y als Fahrtrichtung entspricht links R_new * (-1, 0)
        Eigen::Vector2d u = R_new * Eigen::Vector2d(-1, 0);
        Eigen::Vector2d v = chosen_edge - center;
        double dot = v.dot(u);
        double offset_sign = (dot >= 0.0) ? 1.0 : -1.0;

        Eigen::Vector2d new_center = measured - offset_sign * half_width * u;

        // Korrigierter Transform
        geometry_msgs::msg::TransformStamped corrected_transform = machine_transform;
        corrected_transform.transform.translation.x = new_center.x();
        corrected_transform.transform.translation.y = new_center.y();

        tf2::Quaternion corrected_q;
        corrected_q.setRPY(0, 0, new_yaw);
        corrected_transform.transform.rotation = tf2::toMsg(corrected_q);
        corrected_transform.child_frame_id = machine_frame_id + "-CORRECTED";
        corrected_transform.header.stamp = this->now();

        // Publiziere neuen Transform
        tf_broadcaster_->sendTransform(corrected_transform);

        RCLCPP_INFO(this->get_logger(),
                    "Korrigierter TF für %s (Segment[%zu]) veröffentlicht: "
                    "x=%.3f, y=%.3f, yaw=%.3f",
                    machine_frame_id.c_str(), idx,
                    corrected_transform.transform.translation.x,
                    corrected_transform.transform.translation.y,
                    new_yaw);

        // Erstelle Marker für dieses Segment
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "mapped_segments";
        marker.id = global_marker_id++;  // hochzählen, damit kein Überschreiben
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Linienbreite
        marker.scale.x = 0.02;

        // Farbe: Rot
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;

        // Marker soll unbegrenzt angezeigt werden
        marker.lifetime = rclcpp::Duration(0, 0);

        marker.points.push_back(pt1_map.point);
        marker.points.push_back(pt2_map.point);

        marker_pub_->publish(marker);
      }
    }
  }

  // Hilfsfunktion: Yaw aus Quaternion
  double getYawFromQuaternion(const geometry_msgs::msg::Quaternion &q) const
  {
    tf2::Quaternion tf2_quat(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3 m(tf2_quat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
  }
};

// **Definition** der statischen Variable außerhalb der Klasse
int MapperNode::global_marker_id = 0;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MapperNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
