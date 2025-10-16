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

//
//   created by: Michael Jonathan (mich1342)
//   github.com/mich1342
//   24/2/2022
//

#ifndef LASER_SCAN_INTEGRATOR_HPP_
#define LASER_SCAN_INTEGRATOR_HPP_

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/create_timer_ros.h>

#include "laser_scan_integrator_msg/msg/line_segment.hpp"
#include "laser_scan_integrator_msg/msg/line_segments.hpp"
#include "laser_scan_integrator_msg/srv/toggle_segmentation.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>
#include <pcl/common/distances.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>

#include "visualization_msgs/msg/marker.hpp"

#include <cmath>
#include <algorithm>
#include <array>
#include <iostream>
#include <random>
#include <string>
#include <vector>

struct LaserPoint {
    float direction_;
    float distance_;
};

struct LaserPointLess {
    bool operator()(const LaserPoint &a, const LaserPoint &b) const noexcept {
        return a.direction_ < b.direction_;
    }
};

class scanMerger : public rclcpp::Node {
  public:
    scanMerger();

  private:
    bool segmentation_enabled_ = true;
    rclcpp::Publisher<laser_scan_integrator_msg::msg::LineSegments>::SharedPtr
        line_segments_pub_;
    rclcpp::Service<laser_scan_integrator_msg::srv::ToggleSegmentation>::
        SharedPtr segmentation_service_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pointcloud_;

    void scan_callback1(const sensor_msgs::msg::LaserScan::SharedPtr _msg);
    void scan_callback2(const sensor_msgs::msg::LaserScan::SharedPtr _msg);

    void handle_toggle_segmentation(
        const std::shared_ptr<
            laser_scan_integrator_msg::srv::ToggleSegmentation::Request>
            request,
        std::shared_ptr<
            laser_scan_integrator_msg::srv::ToggleSegmentation::Response>
            response);

    pcl::PointCloud<pcl::PointXYZ>::Ptr laser_scan_to_pointcloud(
        const sensor_msgs::msg::LaserScan::SharedPtr &scan);

    void publishPointCloud(const sensor_msgs::msg::LaserScan::SharedPtr &scan);

    std::vector<laser_scan_integrator_msg::msg::LineSegment>
    calc_lines(typename pcl::PointCloud<pcl::PointXYZ>::ConstPtr input,
               unsigned int segm_min_inliers = 20,
               unsigned int segm_max_iterations = 250,
               float segm_distance_threshold = 0.05f,
               float segm_sample_max_dist = 0.15f,
               float cluster_tolerance = 0.07f, float cluster_quota = 0.1f,
               float min_length = 0.6f, float max_length = 0.8f,
               float min_dist = 0.1f, float max_dist = 2,
               typename pcl::PointCloud<pcl::PointXYZ>::Ptr remaining_cloud =
                   typename pcl::PointCloud<pcl::PointXYZ>::Ptr());

    void publishLineMarkers(
        const std::vector<laser_scan_integrator_msg::msg::LineSegment> &lines,
        const std_msgs::msg::Header &header);

    void update_point_cloud_rgb();

    float GET_R(float x, float y);
    float GET_THETA(float x, float y);
    float interpolate(float angle_1, float angle_2, float magnitude_1,
                      float magnitude_2, float current_angle);
    void geometry_quat_to_rpy(double *roll, double *pitch, double *yaw,
                              geometry_msgs::msg::Quaternion geometry_quat);
    void initialize_params();
    void refresh_params();

    std::string topic1_, topic2_, integratedTopic_, integratedFrameId_;
    bool show1_, show2_;
    float laser1XOff_, laser1YOff_, laser1Alpha_;

    float laser2XOff_, laser2YOff_, laser2Alpha_;
    float RangeMin_, RangeMax_;

    float robotFrontEnd_, robotRearEnd_, robotRightEnd_, robotLeftEnd_;

    rclcpp::Time last_call_time_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub1_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub2_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_pub_;

    std::unique_ptr<tf2_ros::Buffer> tf2_;
    std::unique_ptr<tf2_ros::TransformListener> tf2_listener_;

    sensor_msgs::msg::LaserScan::SharedPtr laser1_;
    sensor_msgs::msg::LaserScan::SharedPtr laser2_;
    geometry_msgs::msg::TransformStamped trans1_;
    geometry_msgs::msg::TransformStamped trans2_;

    double tolerance_;
};

#endif  // LASER_SCAN_INTEGRATOR_HPP_
