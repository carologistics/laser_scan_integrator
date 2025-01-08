//
//   created by: Michael Jonathan (mich1342)
//   github.com/mich1342
//   24/2/2022
//

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/create_timer_ros.h>

#include "laser_scan_integrator_msg/srv/toggle_segmentation.hpp"
#include "laser_scan_integrator_msg/msg/line_segment.hpp"
#include "laser_scan_integrator_msg/msg/line_segments.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>

#include <cmath>

#include <algorithm>
#include <array>
#include <iostream>
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
  scanMerger() : Node("laser_scan_integrator") {
    tolerance_ = this->declare_parameter("transform_tolerance", 0.01);

    initialize_params();
    refresh_params();

    auto default_qos =
        rclcpp::SensorDataQoS(); // rclcpp::QoS(rclcpp::SystemDefaultsQoS());
    sub1_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        topic1_, default_qos,
        std::bind(&scanMerger::scan_callback1, this, std::placeholders::_1));
    sub2_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        topic2_, default_qos,
        std::bind(&scanMerger::scan_callback2, this, std::placeholders::_1));

    laser_scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
        integratedTopic_, rclcpp::SystemDefaultsQoS());

    tf2_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(), this->get_node_timers_interface());
    tf2_->setCreateTimerInterface(timer_interface);
    tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_);

    line_segments_pub_ = this->create_publisher<laser_scan_integrator_msg::msg::LineSegments>(
        "line_segments", rclcpp::SystemDefaultsQoS());

    segmentation_enabled_ = false; // Direkt Klassenvariable setzen

    segmentation_service_ = this->create_service<laser_scan_integrator_msg::srv::ToggleSegmentation>(
        "toggle_segmentation",
        std::bind(&scanMerger::handle_toggle_segmentation, this, std::placeholders::_1, std::placeholders::_2));

  }

private:

  bool segmentation_enabled_ = true; // Klassenvariable
  rclcpp::Publisher<laser_scan_integrator_msg::msg::LineSegments>::SharedPtr line_segments_pub_;
  rclcpp::Service<laser_scan_integrator_msg::srv::ToggleSegmentation>::SharedPtr segmentation_service_;

  void scan_callback1(const sensor_msgs::msg::LaserScan::SharedPtr _msg) {
    laser1_ = _msg;
    if (laser2_) {
      update_point_cloud_rgb();
    }
  }
  void scan_callback2(const sensor_msgs::msg::LaserScan::SharedPtr _msg) {
    laser2_ = _msg;
  }

  void handle_toggle_segmentation(
    const std::shared_ptr<laser_scan_integrator_msg::srv::ToggleSegmentation::Request> request,
    std::shared_ptr<laser_scan_integrator_msg::srv::ToggleSegmentation::Response> response) {

    segmentation_enabled_ = request->enable_segmentation;

    if (segmentation_enabled_) {
        response->success = true;
        response->message = "Segmentation enabled.";
        RCLCPP_INFO(this->get_logger(), "Segmentation has been enabled.");
    } else {
        response->success = true;
        response->message = "Segmentation disabled.";
        RCLCPP_INFO(this->get_logger(), "Segmentation has been disabled.");
    }
  }


  // Punktwolke erstellen
  pcl::PointCloud<pcl::PointXYZ>::Ptr laser_scan_to_pointcloud(
      const sensor_msgs::msg::LaserScan::SharedPtr &scan) {
      auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>(); // Verwende pcl::make_shared
      for (size_t i = 0; i < scan->ranges.size(); ++i) {
          float range = scan->ranges[i];
          if (range >= scan->range_min && range <= scan->range_max) {
              float angle = scan->angle_min + i * scan->angle_increment;
              cloud->points.emplace_back(range * std::cos(angle), range * std::sin(angle), 0.0f);
          }
      }
      return cloud;
  }

  // Liniensegmentierung
  std::vector<laser_scan_integrator_msg::msg::LineSegment> detect_lines(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, 
  float target_length = 0.8f, 
  float length_tolerance = 0.05f) {

    std::vector<laser_scan_integrator_msg::msg::LineSegment> detected_lines;

    // Standardwert für die Distanztoleranz basierend auf einer Ziellänge von 80 cm
    float distance_tolerance = 0.03f;

    // Konfiguration des SAC-Segmentierung
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    //seg.setOptimizeCoefficients(true); Funkzioniert nich bekomme dann in der Ausgabe das:  [laser_scan_integrator-10] [pcl::SampleConsensusModelLine::optimizeModelCoefficients] Not enough inliers to refine/optimize the model's coefficients (2)! Returning the same coefficients.
    seg.setModelType(pcl::SACMODEL_LINE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(distance_tolerance);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    // Eine Kopie der Punktwolke erstellen
    auto cloud_copy = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*cloud);

    while (cloud_copy->size() > 2) {
      seg.setInputCloud(cloud_copy);
      seg.segment(*inliers, *coefficients);

      if (inliers->indices.empty()) {
        break; // Keine Linie gefunden
      }

      pcl::PointXYZ pt1, pt2;
      float line_length = 0.0f;

      // Hier wurden Änderungen vorgenommen
      // Ursprünglich wurde der zweite Punkt durch Hinzufügen des Richtungsvektors bestimmt,
      // was falsch war, da der Richtungsvektor unendlich lang ist.
      // Stattdessen bestimmen wir nun anhand der Inlier-Punkte die tatsächlichen Endpunkte.
      if (coefficients->values.size() >= 6) {
        // Punkt auf der Linie
        Eigen::Vector3f point_on_line(coefficients->values[0],
                                      coefficients->values[1],
                                      coefficients->values[2]);
        // Richtungsvektor der Linie
        Eigen::Vector3f direction(coefficients->values[3],
                                  coefficients->values[4],
                                  coefficients->values[5]);
        direction.normalize();

        // Min- und Max-Projektion bestimmen
        float min_proj =  std::numeric_limits<float>::infinity();
        float max_proj = -std::numeric_limits<float>::infinity();

        for (auto idx : inliers->indices) {
          const auto &p = cloud_copy->points[idx];
          Eigen::Vector3f point(p.x, p.y, p.z);
          float projection = direction.dot(point - point_on_line);

          if (projection < min_proj) {
            min_proj = projection;
          }
          if (projection > max_proj) {
            max_proj = projection;
          }
        }

        // Berechne tatsächliche Endpunkte
        Eigen::Vector3f start_point = point_on_line + min_proj * direction;
        Eigen::Vector3f end_point   = point_on_line + max_proj * direction;

        pt1.x = start_point.x();
        pt1.y = start_point.y();
        pt1.z = start_point.z();

        pt2.x = end_point.x();
        pt2.y = end_point.y();
        pt2.z = end_point.z();

        line_length = (end_point - start_point).norm();
      }

      // Überprüfen, ob die Linie innerhalb der spezifizierten Toleranz liegt
      if (std::abs(line_length - target_length) <= length_tolerance) {
        laser_scan_integrator_msg::msg::LineSegment line_msg;
        line_msg.end_point1.x = pt1.x;
        line_msg.end_point1.y = pt1.y;
        line_msg.end_point1.z = pt1.z;
        line_msg.end_point2.x = pt2.x;
        line_msg.end_point2.y = pt2.y;
        line_msg.end_point2.z = pt2.z;
        line_msg.line_length = line_length;

        detected_lines.push_back(line_msg);
      }

      // Entfernen der segmentierten Punkte aus der Punktwolke
      pcl::ExtractIndices<pcl::PointXYZ> extract;
      extract.setInputCloud(cloud_copy);
      extract.setIndices(inliers);
      extract.setNegative(true);
      extract.filter(*cloud_copy);
    }

    return detected_lines;
  }



  void update_point_cloud_rgb() {
    refresh_params();
    trans1_ = tf2_->lookupTransform(integratedFrameId_,
                                    laser1_->header.frame_id, rclcpp::Time(0));
    trans2_ = tf2_->lookupTransform(integratedFrameId_,
                                    laser2_->header.frame_id, rclcpp::Time(0));
    double sensor1_r, sensor1_p, sensor1_y, sensor2_r, sensor2_p, sensor2_y;
    geometry_quat_to_rpy(&sensor1_r, &sensor1_p, &sensor1_y,
                         trans1_.transform.rotation);
    geometry_quat_to_rpy(&sensor2_r, &sensor2_p, &sensor2_y,
                         trans2_.transform.rotation);
    sensor1_y += laser1Alpha_;
    sensor2_y += laser2Alpha_;

    std::vector<std::array<float, 2>> scan_data;
    int count = 0;
    float min_theta = 0;
    float max_theta = 0;
    if (show1_) {
      // if(false){
      for (float i = laser1_->angle_min; i <= laser1_->angle_max;
           i += laser1_->angle_increment) {
        std::array<float, 2> pt;

        float laser_angle;
        if (fabs(sensor1_r) < M_PI / 2) {
          laser_angle = i;
        } else {
          laser_angle = -i;
        }
        float temp_x = laser1_->ranges[count] * std::cos(laser_angle);
        float temp_y = laser1_->ranges[count] * std::sin(laser_angle);
        pt[0] = temp_x * std::cos(sensor1_y) - temp_y * std::sin(sensor1_y);
        pt[0] += trans1_.transform.translation.x + laser1XOff_;
        pt[1] = temp_x * std::sin(sensor1_y) + temp_y * std::cos(sensor1_y);
        pt[1] += trans1_.transform.translation.y + laser1YOff_;
        count++;

        if (pt[0] < robotFrontEnd_ && pt[0] > -robotRearEnd_ &&
            pt[1] < robotLeftEnd_ && pt[1] > -robotRightEnd_) {
          continue;
        }

        float r_ = GET_R(pt[0], pt[1]);
        float theta_ = GET_THETA(pt[0], pt[1]);
        std::array<float, 2> res_;
        res_[1] = r_;
        res_[0] = theta_;
        scan_data.push_back(res_);
        if (theta_ < min_theta) {
          min_theta = theta_;
        }
        if (theta_ > max_theta) {
          max_theta = theta_;
        }
      }
    }

    //Second scanner
    count = 0;
    if (show2_) {
      for (float i = laser2_->angle_min; i <= laser2_->angle_max;
           i += laser2_->angle_increment) {
        std::array<float, 2> pt;

        float laser_angle;
        if (fabs(sensor2_r) < M_PI / 2) {
          laser_angle = i;
        } else {
          laser_angle = -i;
        }
        float temp_x = laser2_->ranges[count] * std::cos(laser_angle);
        float temp_y = laser2_->ranges[count] * std::sin(laser_angle);
        pt[0] = temp_x * std::cos(sensor2_y) - temp_y * std::sin(sensor2_y);
        pt[0] += trans2_.transform.translation.x + laser2XOff_;
        pt[1] = temp_x * std::sin(sensor2_y) + temp_y * std::cos(sensor2_y);
        pt[1] += trans2_.transform.translation.y + laser2YOff_;
        count++;

        if (pt[0] < robotFrontEnd_ && pt[0] > -robotRearEnd_ &&
            pt[1] < robotLeftEnd_ && pt[1] > -robotRightEnd_) {
          continue;
        }

        float r_ = GET_R(pt[0], pt[1]);
        float theta_ = GET_THETA(pt[0], pt[1]);
        std::array<float, 2> res_;
        res_[1] = r_;
        res_[0] = theta_;
        scan_data.push_back(res_);
        if (theta_ < min_theta) {
          min_theta = theta_;
        }
        if (theta_ > max_theta) {
          max_theta = theta_;
        }
      }
    }

    std::sort(scan_data.begin(), scan_data.end(),
              [](std::array<float, 2> a, std::array<float, 2> b) {
                return a[0] < b[0];
              });

    auto integrated_msg_ = std::make_shared<sensor_msgs::msg::LaserScan>();
    integrated_msg_->header.frame_id = integratedFrameId_;
    integrated_msg_->header.stamp = laser1_->header.stamp;
    integrated_msg_->angle_min = min_theta;
    integrated_msg_->angle_max = max_theta;
    integrated_msg_->angle_increment = laser1_->angle_increment;
    integrated_msg_->time_increment = laser1_->time_increment;
    integrated_msg_->scan_time = laser1_->scan_time;
    integrated_msg_->range_min = RangeMin_;
    integrated_msg_->range_max = RangeMax_;
    size_t i = 1;
    std::vector<float> temp_range;
    for (float angle = min_theta; angle < max_theta;
         angle += laser1_->angle_increment) {
      while (scan_data[i][0] < angle) {
        i++;
      }
      if (fabs(scan_data[i][1] - scan_data[i - 1][1]) < 0.2f &&
          (fabs(scan_data[i][0] - angle) < laser1_->angle_increment ||
           fabs(scan_data[i - 1][0] - angle) < laser1_->angle_increment)) {
        float range = interpolate(scan_data[i - 1][0], scan_data[i][0],
                                  scan_data[i - 1][1], scan_data[i][1], angle);
        if (range <= RangeMin_) {
          temp_range.push_back(std::numeric_limits<float>::infinity());
        } else {
          temp_range.push_back(range);
        }
      } else {
        temp_range.push_back(std::numeric_limits<float>::infinity());
      }
    }
    integrated_msg_->ranges = temp_range;
    laser_scan_pub_->publish(*integrated_msg_);

    if (segmentation_enabled_) {
      // **Hier Integration der Linienerkennung**
      auto pointcloud = laser_scan_to_pointcloud(integrated_msg_);
      auto lines = detect_lines(pointcloud, 0.8f, 0.05f); // 80 cm mit Toleranz

      // Linien in ein ROS-Message konvertieren
      laser_scan_integrator_msg::msg::LineSegments lines_msg;
      lines_msg.header = integrated_msg_->header;
      lines_msg.segments = lines;

      // Linien veröffentlichen
      line_segments_pub_->publish(lines_msg);
    }
  }

  float GET_R(float x, float y) { return sqrt(x * x + y * y); }
  float GET_THETA(float x, float y) { return atan2(y, x); }
  float interpolate(float angle_1, float angle_2, float magnitude_1,
                    float magnitude_2, float current_angle) {

    return (magnitude_1 +
            (current_angle - angle_1) *
                ((magnitude_2 - magnitude_1) / (angle_2 - angle_1)));
  }
  void geometry_quat_to_rpy(double *roll, double *pitch, double *yaw,
                            geometry_msgs::msg::Quaternion geometry_quat) {
    tf2::Quaternion quat;
    tf2::convert(geometry_quat, quat);
    tf2::Matrix3x3(quat).getRPY(*roll, *pitch,
                                *yaw); // rpy are Pass by Reference
  }
  void initialize_params() {

    this->declare_parameter<std::string>("integratedTopic");
    this->declare_parameter<std::string>("integratedFrameId");

    this->declare_parameter<std::string>("scanTopic1");
    this->declare_parameter<float>("laser1XOff");
    this->declare_parameter<float>("laser1YOff");
    this->declare_parameter<float>("laser1Alpha");
    this->declare_parameter<bool>("show1");

    this->declare_parameter<std::string>("scanTopic2");
    this->declare_parameter<float>("laser2XOff");
    this->declare_parameter<float>("laser2YOff");
    this->declare_parameter<float>("laser2Alpha");
    this->declare_parameter<bool>("show2");

    this->declare_parameter<float>("robotFrontEnd");
    this->declare_parameter<float>("robotRearEnd");
    this->declare_parameter<float>("robotRightEnd");
    this->declare_parameter<float>("robotLeftEnd");

    this->declare_parameter<float>("rangeMin");
    this->declare_parameter<float>("rangeMax");
  }
  void refresh_params() {
    this->get_parameter_or<std::string>("integratedTopic", integratedTopic_,
                                        "/robotino3base1/scan");
    this->get_parameter_or<std::string>("integratedFrameId", integratedFrameId_,
                                        "laser_link");
    this->get_parameter_or<std::string>("scanTopic1", topic1_,
                                        "/robotino/SickLaser_Front_Remaped");
    this->get_parameter_or<float>("laser1XOff", laser1XOff_, 0.0);
    this->get_parameter_or<float>("laser1YOff", laser1YOff_, 0.0);
    this->get_parameter_or<float>("laser1Alpha", laser1Alpha_, 0.0);
    this->get_parameter_or<bool>("show1", show1_, true);
    this->get_parameter_or<std::string>("scanTopic2", topic2_,
                                        "/robotino/SickLaser_Rear_Remaped");
    this->get_parameter_or<float>("laser2XOff", laser2XOff_, 0.0);
    this->get_parameter_or<float>("laser2YOff", laser2YOff_, 0.0);
    this->get_parameter_or<float>("laser2Alpha", laser2Alpha_, 0.0);
    this->get_parameter_or<bool>("show2", show2_, true);

    this->get_parameter_or<float>("robotFrontEnd", robotFrontEnd_, 0.0);
    this->get_parameter_or<float>("robotRearEnd", robotRearEnd_, 0.0);
    this->get_parameter_or<float>("robotRightEnd", robotRightEnd_, 0.0);
    this->get_parameter_or<float>("robotLeftEnd", robotLeftEnd_, 0.0);

    this->get_parameter_or<float>("rangeMin", RangeMin_, 0.0);
    this->get_parameter_or<float>("rangeMax", RangeMax_, 100.0);
  }

  std::string topic1_, topic2_, integratedTopic_, integratedFrameId_;
  bool show1_, show2_;
  float laser1XOff_, laser1YOff_, laser1Alpha_;

  float laser2XOff_, laser2YOff_, laser2Alpha_;
  float RangeMin_, RangeMax_;

  float robotFrontEnd_, robotRearEnd_, robotRightEnd_, robotLeftEnd_;

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

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<scanMerger>());
  rclcpp::shutdown();
  return 0;
}