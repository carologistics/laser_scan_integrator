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

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "laser_scan_integrator_msg/msg/line_segments.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>
#include <vector>
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

class MapperNode : public rclcpp::Node {
public:
  MapperNode()
      : Node("mapper"),
        tf_buffer_(std::make_unique<tf2_ros::Buffer>(this->get_clock())),
        tf_listener_(
            std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)) {
    std::string share_dir =
        ament_index_cpp::get_package_share_directory("laser_scan_mapper");
    std::string yaml_file = share_dir + "/mapper_params.yaml";
    RCLCPP_INFO(this->get_logger(), "Load machine names from YAML: %s",
                yaml_file.c_str());

    if (!loadMachineNamesFromYaml(yaml_file, machine_names_)) {
      RCLCPP_ERROR(
          this->get_logger(),
          "Failed to load machine names from YAML: %s. Node will shut down.",
          yaml_file.c_str());
      rclcpp::shutdown();
      return;
    }

    // TF-Broadcaster initialisieren
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Marker-Publisher für segmentierte Linien (Topic: /mapped_segments)
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "mapped_segments", 10);

    // Marker-Publisher für Maschinen-Rechtecke (Topic: /machine_markers)
    machine_marker_pub_ =
        this->create_publisher<visualization_msgs::msg::Marker>(
            "machine_markers", 10);

    // Erstelle Timer um statische TF zu finden
    transform_timer_ = this->create_wall_timer(
        1s, std::bind(&MapperNode::updateMachineTransforms, this));

    // Subscriber für LineSegments
    sub_segments_ =
        this->create_subscription<laser_scan_integrator_msg::msg::LineSegments>(
            "line_segments", 10,
            std::bind(&MapperNode::segmentsCallback, this,
                      std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Mapper gestartet.");

    ns_ = removeLeadingSlash(this->get_namespace());
  }

private:
  const double machine_width_ = 0.35;
  const double position_tolerance_ = 0.3;
  const double angle_tolerance_ = 0.5;
  const double machine_length_ = 0.70;
  static int global_marker_id;

  std::vector<std::string> machine_names_;

  std::map<std::string, geometry_msgs::msg::TransformStamped>
      machine_transforms_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      machine_marker_pub_;

  rclcpp::Subscription<laser_scan_integrator_msg::msg::LineSegments>::SharedPtr
      sub_segments_;

  std::string ns_;

  std::string removeLeadingSlash(const std::string &ns) {
    if (!ns.empty() && ns.front() == '/') {
      return ns.substr(1);
    }
    return ns;
  }

  bool loadMachineNamesFromYaml(const std::string &file_path,
                                std::vector<std::string> &out_names) {
    try {
      YAML::Node config = YAML::LoadFile(file_path);
      // Erwartete Struktur: mapper -> ros__parameters -> machine_names
      if (config["mapper"] && config["mapper"]["ros__parameters"] &&
          config["mapper"]["ros__parameters"]["machine_names"]) {
        auto machine_node =
            config["mapper"]["ros__parameters"]["machine_names"];
        for (std::size_t i = 0; i < machine_node.size(); ++i) {
          out_names.push_back(machine_node[i].as<std::string>());
        }
        RCLCPP_INFO(this->get_logger(), "%zu machine names loaded from YAML.",
                    out_names.size());
        return !out_names.empty();
      } else {
        RCLCPP_ERROR(this->get_logger(), "Invalid YAML structure in %s",
                     file_path.c_str());
      }
    } catch (const YAML::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "YAML Parse Error: %s", e.what());
    }
    return false;
  }

  / void publishSingleMachineMarker(
        const std::string &machine_frame_id,
        const geometry_msgs::msg::TransformStamped &transform_stamped) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "machine_rectangles";

    static int machine_marker_id = 0;
    marker.id = machine_marker_id++;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = transform_stamped.transform.translation.x;
    marker.pose.position.y = transform_stamped.transform.translation.y;
    marker.pose.position.z = transform_stamped.transform.translation.z;
    marker.pose.orientation = transform_stamped.transform.rotation;

    marker.scale.x = machine_width_;
    marker.scale.y = machine_length_;
    marker.scale.z = 0.05;

    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 0.8;

    marker.lifetime = rclcpp::Duration(0, 0);

    machine_marker_pub_->publish(marker);
    RCLCPP_INFO(this->get_logger(),
                "Maschinenmarker für %s veröffentlicht: (%.3f, %.3f)",
                machine_frame_id.c_str(),
                transform_stamped.transform.translation.x,
                transform_stamped.transform.translation.y);
  }

  void updateMachineTransforms() {
    bool all_loaded = true;
    for (const auto &machine_frame_id : machine_names_) {
      if (machine_transforms_.find(machine_frame_id) !=
          machine_transforms_.end()) {
        continue;
      }
      try {
        RCLCPP_INFO(this->get_logger(), "Try to load trasform for %s ...",
                    machine_frame_id.c_str());
        auto transform_stamped = tf_buffer_->lookupTransform(
            "map", machine_frame_id, tf2::TimePointZero);
        machine_transforms_[machine_frame_id] = transform_stamped;
        publishSingleMachineMarker(machine_frame_id, transform_stamped);
        RCLCPP_INFO(this->get_logger(),
                    "Transformation for %s successfully loaded",
                    machine_frame_id.c_str());
      } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(),
                    "Transformation for %s is yet not available %s",
                    machine_frame_id.c_str(), ex.what());
        all_loaded = false;
      }
    }

    if (all_loaded) {
      RCLCPP_INFO(this->get_logger(), "All transformation are now available");
      transform_timer_->cancel();
    }
  }

  void segmentsCallback(
      const laser_scan_integrator_msg::msg::LineSegments::SharedPtr msg) {
    for (const auto &entry : machine_transforms_) {
      const std::string &machine_frame_id = entry.first;
      const auto &machine_transform = entry.second;

      for (size_t idx = 0; idx < msg->segments.size(); ++idx) {
        const auto &segment = msg->segments[idx];

        // Transform segment endpoints into the map frame
        geometry_msgs::msg::PointStamped pt1_in, pt2_in, pt1_base, pt2_base,
            pt1_map, pt2_map;
        pt1_in.header.stamp = this->now();
        pt1_in.header.frame_id = segment.frame_id;
        pt1_in.point = segment.end_point1;

        pt2_in.header = pt1_in.header;
        pt2_in.point = segment.end_point2;

        try {
          pt1_map =
              tf_buffer_->transform(pt1_in, "map", tf2::durationFromSec(1.0));
          pt2_map =
              tf_buffer_->transform(pt2_in, "map", tf2::durationFromSec(1.0));
          RCLCPP_INFO(this->get_logger(), "pt1_map(1): x=%.3f, y=%.3f",
                      pt1_map.point.x, pt1_map.point.y);
          RCLCPP_INFO(this->get_logger(), "pt2_map(1): x=%.3f, y=%.3f",
                      pt2_map.point.x, pt2_map.point.y);
        } catch (const tf2::TransformException &ex) {
          // Transformation of segment center failed, skip this segment
          RCLCPP_WARN(this->get_logger(),
                      "Transformation von %s nach map fehlgeschlagen: %s",
                      pt1_in.header.frame_id.c_str(), ex.what());
          continue;
        }

        // Calculate segment midpoint in map frame and direction vector
        Eigen::Vector2d segment_origin_map(
            0.5 * (pt2_map.point.x + pt1_map.point.x),
            0.5 * (pt2_map.point.y + pt1_map.point.y));
        Eigen::Vector2d line(pt2_map.point.x - pt1_map.point.x,
                             pt2_map.point.y - pt1_map.point.y);
        Eigen::Vector3d z(0.0, 0.0, 1.0);
        Eigen::Vector3d line_cross_3d =
            z.cross(Eigen::Vector3d(line.x(), line.y(), 0.0));
        Eigen::Vector2d line_cross = line_cross_3d.head<2>();
        double line_angle = std::atan2(line_cross.y(), line_cross.x());

        // Create the segment transform in map frame
        geometry_msgs::msg::TransformStamped segment_transform;
        segment_transform.header.frame_id = "map";
        segment_transform.child_frame_id = "segment_frame"; // freely chosen
        segment_transform.header.stamp = this->now();
        segment_transform.transform.translation.x = segment_origin_map.x();
        segment_transform.transform.translation.y = segment_origin_map.y();
        segment_transform.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, line_angle);
        segment_transform.transform.rotation = tf2::toMsg(q);

        RCLCPP_INFO(this->get_logger(), "line: x=%.3f, y=%.3f", line.x(),
                    line.y());

        // Convert both transforms to tf2::Transform
        tf2::Transform tf_machine(
            tf2::Quaternion(machine_transform.transform.rotation.x,
                            machine_transform.transform.rotation.y,
                            machine_transform.transform.rotation.z,
                            machine_transform.transform.rotation.w),
            tf2::Vector3(machine_transform.transform.translation.x,
                         machine_transform.transform.translation.y,
                         machine_transform.transform.translation.z));

        tf2::Transform tf_segment(
            tf2::Quaternion(segment_transform.transform.rotation.x,
                            segment_transform.transform.rotation.y,
                            segment_transform.transform.rotation.z,
                            segment_transform.transform.rotation.w),
            tf2::Vector3(segment_transform.transform.translation.x,
                         segment_transform.transform.translation.y,
                         segment_transform.transform.translation.z));

        // Compute the difference transform (segment relative to machine)
        tf2::Transform tf_diff = tf_machine.inverseTimes(tf_segment);

        // Compute distance
        double dx = tf_diff.getOrigin().x();
        double dy = tf_diff.getOrigin().y();
        double dz = tf_diff.getOrigin().z();
        double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

        // Compute yaw difference
        double roll, pitch, yaw;
        tf2::Matrix3x3(tf_diff.getRotation()).getRPY(roll, pitch, yaw);

        // Adjust yaw if it's approximately ±180°
        double adjusted_yaw = yaw;
        if (std::abs(std::abs(yaw) - M_PI) < angle_tolerance_) {
          adjusted_yaw = (yaw > 0) ? yaw - M_PI : yaw + M_PI;
        }

        // Reject segment if distance or angle exceed tolerances
        if (std::abs(distance) > position_tolerance_ ||
            std::abs(adjusted_yaw) > angle_tolerance_) {
          RCLCPP_INFO(
              this->get_logger(),
              "Segment [%zu] rejected for %s: dist=%.3f, dtheta=%.3f \n --- \n",
              idx, machine_frame_id.c_str(), distance, adjusted_yaw);
          continue;
        }

        RCLCPP_INFO(this->get_logger(), "center_segment(2): x=%.3f, y=%.3f",
                    segment_origin_map.x(), segment_origin_map.y());

        // Create corrected transform based on machine transform
        geometry_msgs::msg::TransformStamped corrected_transform =
            machine_transform;
        corrected_transform.transform.translation.x = segment_origin_map.x();
        corrected_transform.transform.translation.y = segment_origin_map.y();
        tf2::Quaternion corrected_q;
        corrected_q.setRPY(
            0, 0,
            line_angle + ((std::abs(std::abs(yaw) - M_PI) < angle_tolerance_)
                              ? M_PI
                              : 0));
        corrected_transform.transform.rotation = tf2::toMsg(corrected_q);
        corrected_transform.child_frame_id =
            ns_ + "/" + machine_frame_id + "-CORRECTED";
        corrected_transform.header.stamp = this->now();
        RCLCPP_INFO(this->get_logger(),
                    "Corrected TF for %s (Segment[%zu]) published: x=%.3f, "
                    "y=%.3f, yaw=%.3f \n --- \n",
                    machine_frame_id.c_str(), idx,
                    corrected_transform.transform.translation.x,
                    corrected_transform.transform.translation.y, line_angle);

        // Publish the corrected transform
        tf_broadcaster_->sendTransform(corrected_transform);

        // Convert corrected_transform (geometry_msgs) to tf2::Transform
        tf2::Transform tf_corrected;
        tf2::fromMsg(corrected_transform.transform, tf_corrected);

        // Create offset transform for "INPUTPUT-CORRECTED":
        // 50 cm along positive x and 180° rotation around z.
        tf2::Transform tf_offset_input;
        {
          tf2::Quaternion q_offset_input;
          q_offset_input.setRPY(0, 0, M_PI); // 180° rotation around z
          tf_offset_input.setRotation(q_offset_input);
          tf_offset_input.setOrigin(
              tf2::Vector3(0.5, 0.0, 0.0)); // +0.5 m along x
        }

        // Create offset transform for "OUTPUT-CORRECTED":
        // 50 cm along negative x with no additional rotation.
        tf2::Transform tf_offset_output;
        {
          tf2::Quaternion q_offset_output;
          q_offset_output.setRPY(0, 0, 0); // no rotation
          tf_offset_output.setRotation(q_offset_output);
          tf_offset_output.setOrigin(
              tf2::Vector3(-0.5, 0.0, 0.0)); // -0.5 m along x
        }

        // Compute the new transforms relative to the corrected transform
        tf2::Transform tf_input = tf_corrected * tf_offset_input;
        tf2::Transform tf_output = tf_corrected * tf_offset_output;

        // Create and publish the new geometry_msgs::msg::TransformStamped
        // messages

        // For "INPUTPUT-CORRECTED"
        geometry_msgs::msg::TransformStamped input_transform =
            corrected_transform;
        input_transform.child_frame_id =
            ns_ + machine_frame_id + "-INPUTPUT-CORRECTED";
        input_transform.transform = tf2::toMsg(tf_input);
        input_transform.header.stamp = this->now();
        tf_broadcaster_->sendTransform(input_transform);

        // For "OUTPUT-CORRECTED"
        geometry_msgs::msg::TransformStamped output_transform =
            corrected_transform;
        output_transform.child_frame_id =
            ns_ + machine_frame_id + "-OUTPUT-CORRECTED";
        output_transform.transform = tf2::toMsg(tf_output);
        output_transform.header.stamp = this->now();
        tf_broadcaster_->sendTransform(output_transform);

        // Create a marker for visualization
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "mapped_segments";
        marker.id = global_marker_id++; // Unique ID per marker
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.02; // Line width

        // Marker color: red
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;

        // Marker persists indefinitely
        marker.lifetime = rclcpp::Duration(0, 0);

        marker.points.push_back(pt1_map.point);
        marker.points.push_back(pt2_map.point);

        marker_pub_->publish(marker);
      }
    }
  }

  // Hilfsfunktion: Yaw aus Quaternion
  double getYawFromQuaternion(const geometry_msgs::msg::Quaternion &q) const {
    tf2::Quaternion tf2_quat(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3 m(tf2_quat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
  }
};

int MapperNode::global_marker_id = 0;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MapperNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
