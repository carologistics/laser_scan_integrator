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

inline bool ends_with(const std::string &str, const std::string &suffix) {
  return str.size() >= suffix.size() &&
         str.compare(str.size() - suffix.size(), suffix.size(), suffix) == 0;
}

using namespace std::chrono_literals;

class MapperNode : public rclcpp::Node {
public:
  MapperNode()
      : Node("mapper"),
        tf_buffer_(std::make_unique<tf2_ros::Buffer>(this->get_clock())),
        tf_listener_(
            std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)) {
    // Load machine names from the ROS parameter "machine_names"
    this->declare_parameter<std::vector<std::string>>(
        "machine_names", std::vector<std::string>{});
    machine_names_ = this->get_parameter("machine_names")
                         .get_value<std::vector<std::string>>();
    if (machine_names_.empty()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Parameter 'machine_names' is empty. Shutting down node.");
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(this->get_logger(),
                "%zu machine names loaded from ROS parameters.",
                machine_names_.size());

    this->declare_parameter<double>("position_tolerance", 0.3);
    this->declare_parameter<double>("angle_tolerance", 0.5);
    position_tolerance_ = this->get_parameter("position_tolerance").as_double();
    angle_tolerance_ = this->get_parameter("angle_tolerance").as_double();
    // Initialize the TF Broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Create publisher for segmented line markers (topic: /mapped_segments)
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "mapped_segments", 10);

    // Create publisher for machine markers (topic: /machine_markers)
    machine_marker_pub_ =
        this->create_publisher<visualization_msgs::msg::Marker>(
            "machine_markers", 10);

    // Create timer to update static TFs
    transform_timer_ = this->create_wall_timer(
        10s, std::bind(&MapperNode::updateMachineTransforms, this));

    // Subscriber for LineSegments messages
    sub_segments_ =
        this->create_subscription<laser_scan_integrator_msg::msg::LineSegments>(
            "line_segments", 10,
            std::bind(&MapperNode::segmentsCallback, this,
                      std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Mapper started.");

    ns_ = removeLeadingSlash(this->get_namespace());
  }

private:
  // Constants for marker publishing
  const double machine_width_ = 0.35;
  const double machine_length_ = 0.70;
  static int global_marker_id;

  // Member variables
  rclcpp::TimerBase::SharedPtr transform_timer_;
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
  double position_tolerance_;
  double angle_tolerance_;
  double position_tolerance_aruco;
  double angle_tolerance_aruco;

  // Remove a leading slash from the namespace if it exists
  std::string removeLeadingSlash(const std::string &ns) {
    if (!ns.empty() && ns.front() == '/') {
      return ns.substr(1);
    }
    return ns;
  }

  // Publish a marker for a single machine based on its transform
  void publishSingleMachineMarker(
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
    // RCLCPP_INFO(this->get_logger(), "Published machine marker for %s: (%.3f,
    // %.3f)",
    //             machine_frame_id.c_str(),
    //             transform_stamped.transform.translation.x,
    //             transform_stamped.transform.translation.y);
  }

  // Update the transforms for each machine and publish markers
  void updateMachineTransforms() {
    bool all_loaded = true;
    for (const auto &machine_frame_id : machine_names_) {
      if (machine_transforms_.find(machine_frame_id) !=
          machine_transforms_.end()) {
        continue;
      }
      try {
        // RCLCPP_INFO(this->get_logger(), "Attempting to load transform for
        // %s...", machine_frame_id.c_str());
        auto transform_stamped = tf_buffer_->lookupTransform(
            "map", machine_frame_id, tf2::TimePointZero);
        machine_transforms_[machine_frame_id] = transform_stamped;
        publishSingleMachineMarker(machine_frame_id, transform_stamped);
        // RCLCPP_INFO(this->get_logger(), "Transform for %s successfully
        // loaded", machine_frame_id.c_str());
      } catch (const tf2::TransformException &ex) {
        // RCLCPP_WARN(this->get_logger(), "Transform for %s not yet available:
        // %s",
        //             machine_frame_id.c_str(), ex.what());
        all_loaded = false;
      }
    }

    if (all_loaded) {
      RCLCPP_INFO(this->get_logger(), "All transforms are now available");
      transform_timer_->cancel();
    }
  }

  // Callback for processing LineSegments messages
  void segmentsCallback(
      const laser_scan_integrator_msg::msg::LineSegments::SharedPtr msg) {
    for (const auto &entry : machine_transforms_) {
      const std::string &machine_frame_id = entry.first;
      const auto &machine_transform = entry.second;

      for (size_t idx = 0; idx < msg->segments.size(); ++idx) {
        const auto &segment = msg->segments[idx];

        // Transform segment endpoints into the "map" frame
        geometry_msgs::msg::PointStamped pt1_in, pt2_in, pt1_map, pt2_map;
        pt1_in.header.stamp = this->now();
        pt1_in.header.frame_id = segment.frame_id;
        pt1_in.point = segment.end_point1;

        pt2_in.header = pt1_in.header;
        pt2_in.point = segment.end_point2;
        bool aruco_tag = ends_with(machine_frame_id, "-I") ||
                         ends_with(machine_frame_id, "-O");

        try {
          pt1_map =
              tf_buffer_->transform(pt1_in, "map", tf2::durationFromSec(1.0));
          pt2_map =
              tf_buffer_->transform(pt2_in, "map", tf2::durationFromSec(1.0));
          // RCLCPP_INFO(this->get_logger(), "pt1_map: x=%.3f, y=%.3f",
          // pt1_map.point.x, pt1_map.point.y); RCLCPP_INFO(this->get_logger(),
          //"pt2_map: x=%.3f, y=%.3f", pt2_map.point.x, pt2_map.point.y);
        } catch (const tf2::TransformException &ex) {
          // RCLCPP_WARN(this->get_logger(), "Failed to transform from %s to
          // map: %s",
          //             pt1_in.header.frame_id.c_str(), ex.what());
          continue;
        }

        // Calculate the midpoint and direction vector of the segment in the map
        // frame
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

        // Create the segment transform in the "map" frame
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

        // RCLCPP_INFO(this->get_logger(), "line: x=%.3f, y=%.3f", line.x(),
        // line.y());

        // Calculate the difference transform (segment relative to machine)
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

        tf2::Transform tf_diff = tf_machine.inverseTimes(tf_segment);
        double dx = tf_diff.getOrigin().x();
        double dy = tf_diff.getOrigin().y();
        double dz = tf_diff.getOrigin().z();
        double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

        double roll, pitch, yaw;
        tf2::Matrix3x3(tf_diff.getRotation()).getRPY(roll, pitch, yaw);
        double adjusted_yaw = yaw;

        if (std::abs(std::abs(yaw) - M_PI) < angle_tolerance_) {
          adjusted_yaw = (yaw > 0) ? yaw - M_PI : yaw + M_PI;
        }

        // Reject segment if distance or angle exceed tolerances
        if (std::abs(distance) > position_tolerance_ ||
            (!aruco_tag && std::abs(adjusted_yaw) > angle_tolerance_)) {
          //   RCLCPP_INFO(this->get_logger(),
          //    "Segment [%zu] for %s rejected:\n"
          //    "dist=%.3f, dtheta=%.3f",
          //    idx, machine_frame_id.c_str(), distance, adjusted_yaw);
          continue;
        }

        if (!aruco_tag) {
          RCLCPP_INFO(this->get_logger(), "Segment center: x=%.3f, y=%.3f",
                      segment_origin_map.x(), segment_origin_map.y());

          // Create corrected transform based on the machine transform
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
          // RCLCPP_INFO(this->get_logger(), "Published corrected TF for %s
          // (Segment[%zu]): x=%.3f, y=%.3f, yaw=%.3f",
          // machine_frame_id.c_str(), idx,
          // corrected_transform.transform.translation.x,
          // corrected_transform.transform.translation.y, line_angle);

          tf_broadcaster_->sendTransform(corrected_transform);

          tf2::Transform tf_corrected;
          tf2::fromMsg(corrected_transform.transform, tf_corrected);

          // Create offset transformation for "INPUTPUT-CORRECTED" (50cm along
          // +x and 180° rotation)
          tf2::Transform tf_offset_input;
          {
            tf2::Quaternion q_offset_input;
            q_offset_input.setRPY(0, 0, M_PI);
            tf_offset_input.setRotation(q_offset_input);
            tf_offset_input.setOrigin(tf2::Vector3(0.5, 0.0, 0.0));
          }

          // Create offset transformation for "OUTPUT-CORRECTED" (50cm along -x)
          tf2::Transform tf_offset_output;
          {
            tf2::Quaternion q_offset_output;
            q_offset_output.setRPY(0, 0, 0);
            tf_offset_output.setRotation(q_offset_output);
            tf_offset_output.setOrigin(tf2::Vector3(-0.5, 0.0, 0.0));
          }

          tf2::Transform tf_input = tf_corrected * tf_offset_input;
          tf2::Transform tf_output = tf_corrected * tf_offset_output;

          // Transformation for "INPUTPUT-CORRECTED"
          geometry_msgs::msg::TransformStamped input_transform =
              corrected_transform;
          input_transform.child_frame_id =
              ns_ + machine_frame_id + "-INPUTPUT-CORRECTED";
          input_transform.transform = tf2::toMsg(tf_input);
          input_transform.header.stamp = this->now();
          tf_broadcaster_->sendTransform(input_transform);

          // Transformation for "OUTPUT-CORRECTED"
          geometry_msgs::msg::TransformStamped output_transform =
              corrected_transform;
          output_transform.child_frame_id =
              ns_ + machine_frame_id + "-OUTPUT-CORRECTED";
          output_transform.transform = tf2::toMsg(tf_output);
          output_transform.header.stamp = this->now();
          tf_broadcaster_->sendTransform(output_transform);

          // Create a marker for visualization of the segment
          visualization_msgs::msg::Marker marker;
          marker.header.frame_id = "map";
          marker.header.stamp = this->now();
          marker.ns = "mapped_segments";
          marker.id = global_marker_id++;
          marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
          marker.action = visualization_msgs::msg::Marker::ADD;
          marker.scale.x = 0.02;

          // Marker color: red
          marker.color.r = 1.0f;
          marker.color.g = 0.0f;
          marker.color.b = 0.0f;
          marker.color.a = 1.0f;

          marker.lifetime = rclcpp::Duration(0, 0);

          marker.points.push_back(pt1_map.point);
          marker.points.push_back(pt2_map.point);

          marker_pub_->publish(marker);
        } else {
          geometry_msgs::msg::TransformStamped tf1;
          try {
            auto tf1 = tf_buffer_->lookupTransform("map", ns_ + "/base_link",
                                                   tf2::TimePointZero);
          } catch (const tf2::TransformException &ex) {
            // RCLCPP_WARN(this->get_logger(), "Transform for %s not yet
            // available: %s",
            //             machine_frame_id.c_str(), ex.what())
          }
          tf2::Transform base_link_tf;
          tf2::fromMsg(tf1.transform, base_link_tf);
          tf2::Transform tf_diff_2 = base_link_tf.inverseTimes(tf_segment);
          tf2::Matrix3x3(tf_diff_2.getRotation()).getRPY(roll, pitch, yaw);

          // Create corrected transform based on the aruco transform
          geometry_msgs::msg::TransformStamped corrected_transform =
              machine_transform;
          corrected_transform.transform.translation.x =
              machine_transform.transform.translation.x;
          corrected_transform.transform.translation.y =
              machine_transform.transform.translation.y;
          tf2::Quaternion corrected_q;
          // Check if aruco tf y-axis looks away from machine center
          corrected_q.setRPY(
              0, 0,
              line_angle +
                  ((std::abs(std::abs(yaw) - M_PI) < 3.141) ? M_PI : 0));
          corrected_transform.transform.rotation = tf2::toMsg(corrected_q);
          corrected_transform.child_frame_id =
              ns_ + "/" + machine_frame_id + "-ARUCO";
          corrected_transform.header.stamp = this->now();
          tf_broadcaster_->sendTransform(corrected_transform);
        }
      }
    }
  }

  // Helper function: Extract yaw from a quaternion
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
