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

//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
//#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/filters/extract_indices.h>
//#include <pcl/search/kdtree.h>

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
#include <string>
#include <vector>
#include <random>


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

    segmentation_enabled_ = false; 

    segmentation_service_ = this->create_service<laser_scan_integrator_msg::srv::ToggleSegmentation>(
        "toggle_segmentation",
        std::bind(&scanMerger::handle_toggle_segmentation, this, std::placeholders::_1, std::placeholders::_2));
    marker_pub_ = this ->create_publisher<visualization_msgs::msg::Marker>("line_markers",10);
    pub_pointcloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud_out", 10);

  }

private:

  bool segmentation_enabled_ = true; 
  rclcpp::Publisher<laser_scan_integrator_msg::msg::LineSegments>::SharedPtr line_segments_pub_;
  rclcpp::Service<laser_scan_integrator_msg::srv::ToggleSegmentation>::SharedPtr segmentation_service_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pointcloud_;

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


  // Create point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr laser_scan_to_pointcloud(
      const sensor_msgs::msg::LaserScan::SharedPtr &scan) {
      auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>(); 
      for (size_t i = 0; i < scan->ranges.size(); ++i) {
          float range = scan->ranges[i];
          if (range >= scan->range_min && range <= scan->range_max) {
              float angle = scan->angle_min + i * scan->angle_increment;
              cloud->points.emplace_back(range * std::cos(angle), range * std::sin(angle), 0.0f);
          }
      }
      return cloud;
  }

  void publishPointCloud(const sensor_msgs::msg::LaserScan::SharedPtr &scan){
	    // 1. LaserScan -> pcl::PointCloud<pcl::PointXYZ>
	    auto pcl_cloud = laser_scan_to_pointcloud(scan);

	    // 2. pcl::PointCloud -> sensor_msgs::msg::PointCloud2
	    sensor_msgs::msg::PointCloud2 ros_cloud;
	    pcl::toROSMsg(*pcl_cloud, ros_cloud);

	    // 3. Header setzen
	    ros_cloud.header.stamp = this->now();
	    ros_cloud.header.frame_id =  scan->header.frame_id;
	    // Wähle das Frame so, dass es zu deiner TF-Tree-Konfiguration passt

	    // 4. Veröffentlichen
	    pub_pointcloud_->publish(ros_cloud);
}

  // Line segmentation
  std::vector<laser_scan_integrator_msg::msg::LineSegment> detect_lines(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, 
    float target_length = 0.7f, 
    float length_tolerance = 0.02f) {

      std::vector<laser_scan_integrator_msg::msg::LineSegment> detected_lines;

      // Default value for distance tolerance, assuming a target length of 80 cm
      float distance_tolerance = 0.03f;


      // Configure SAC segmentation
      pcl::SACSegmentation<pcl::PointXYZ> seg;
      seg.setOptimizeCoefficients(true); 
      // This is commented out because it leads to the following error during execution:
      // [laser_scan_integrator-10] [pcl::SampleConsensusModelLine::optimizeModelCoefficients] 
      // Not enough inliers to refine/optimize the model's coefficients (2)! Returning the same coefficients.
      seg.setModelType(pcl::SACMODEL_LINE);
      seg.setMethodType(pcl::SAC_RANSAC);
      seg.setDistanceThreshold(distance_tolerance);
      seg.setMaxIterations(1000);
      // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>()); 
      // seg.setSamplesMaxDist(0.4, tree); 

      
      // Meomeor leak noch mal drüber schauen liebr einen unique pointer erstellen
      pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
      pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

      // Create a copy of the input point cloud to modify during processing
      auto cloud_copy = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*cloud);

      // Process the point cloud until fewer than 3 points remain (minimum for a line)
      while (cloud_copy->size() > 2) {
          seg.setInputCloud(cloud_copy);
          seg.segment(*inliers, *coefficients);

          // If no inliers are found, stop the loop
          if (inliers->indices.empty()) {
              break; // No line found
          }

          pcl::PointXYZ pt1, pt2;
          float line_length = 0.0f;

          if (coefficients->values.size() >= 6) {
              // Point on the line
              Eigen::Vector3f point_on_line(coefficients->values[0],
                                            coefficients->values[1],
                                            coefficients->values[2]);
              // Direction vector of the line
              Eigen::Vector3f direction(coefficients->values[3],
                                        coefficients->values[4],
                                        coefficients->values[5]);
              direction.normalize();

              // Determine the minimum and maximum projection onto the line
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

              // Compute the actual endpoints of the line
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

          // Check if the line length falls within the specified tolerance
          if (std::abs(line_length - target_length) <= length_tolerance) {
              laser_scan_integrator_msg::msg::LineSegment line_msg;
              line_msg.frame_id = "robotininobase1/laser_link";// CHange later
              line_msg.end_point1.x = pt1.x;
              line_msg.end_point1.y = pt1.y;
              line_msg.end_point1.z = pt1.z;
              line_msg.end_point2.x = pt2.x;
              line_msg.end_point2.y = pt2.y;
              line_msg.end_point2.z = pt2.z;
              line_msg.line_length = line_length;

              detected_lines.push_back(line_msg);
          }

          // Remove the segmented points from the point cloud
          pcl::ExtractIndices<pcl::PointXYZ> extract;
          extract.setInputCloud(cloud_copy);
          extract.setIndices(inliers);
          extract.setNegative(true);
          extract.filter(*cloud_copy);
      }

      return detected_lines;
  }



std::vector<laser_scan_integrator_msg::msg::LineSegment> calc_lines(typename pcl::PointCloud<pcl::PointXYZ>::ConstPtr input,
           unsigned int                                  segm_min_inliers = 20,
           unsigned int                                  segm_max_iterations = 250,
           float                                         segm_distance_threshold = 0.05f,
           float                                         segm_sample_max_dist = 0.15f,
           float                                         cluster_tolerance = 0.07f,
           float                                         cluster_quota = 0.1f,
           float                                         min_length = 0.6f,
           float                                         max_length = 0.8f,
           float                                         min_dist = 0.1f,
           float                                         max_dist = 2,
           typename pcl::PointCloud<pcl::PointXYZ>::Ptr      remaining_cloud = typename pcl::PointCloud<pcl::PointXYZ>::Ptr()) {


	typename pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud(new pcl::PointCloud<pcl::PointXYZ>());

  pcl::PointCloud<pcl::PointXYZ>::Ptr test_cloud(new pcl::PointCloud<pcl::PointXYZ>());

  // Zufallsgenerator initialisieren
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<float> dist_x(-2.0f, 2.0f);  // x-Werte von -2 bis 2 m
  std::uniform_real_distribution<float> dist_y(-2.0f, 2.0f);  // y-Werte von -2 bis 2 m

  
  for (float x = 0.0f; x <= 1.0f; x += 0.01f) {
      test_cloud->push_back(pcl::PointXYZ(x, 0.0f, 0.0f));
  }

  // Linie 2 (y = 2.0, parallel zur ersten Linie, 2 m versetzt)
  for (float x = 2.0f; x <= 3.0f; x += 0.01f) {
      test_cloud->push_back(pcl::PointXYZ(x, 0.0f, 0.0f)); 
  }

  // 200 zufällige Störpunkte generieren
  for (int i = 0; i < 200; ++i) {
      float rand_x = dist_x(gen);
      float rand_y = dist_y(gen);
      test_cloud->push_back(pcl::PointXYZ(rand_x, rand_y, 0.0f));
  }



	{
		// Erase non-finite points
		pcl::PassThrough<pcl::PointXYZ> passthrough;
		passthrough.setInputCloud(input);
		passthrough.filter(*in_cloud);
	}

	pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients());
	pcl::PointIndices::Ptr      inliers(new pcl::PointIndices());

  std::vector<laser_scan_integrator_msg::msg::LineSegment> detected_lines;

	while (in_cloud->points.size() > segm_min_inliers) {
		// Segment the largest linear component from the remaining cloud
		//logger->log_info(name(), "[L %u] %zu points left",
		//		     loop_count_, in_cloud->points.size());

		typename pcl::search::KdTree<pcl::PointXYZ>::Ptr search(new pcl::search::KdTree<pcl::PointXYZ>);
		search->setInputCloud(in_cloud);

		pcl::SACSegmentation<pcl::PointXYZ> seg;
		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_LINE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setMaxIterations(segm_max_iterations);
		seg.setDistanceThreshold(segm_distance_threshold);
		seg.setSamplesMaxDist(segm_sample_max_dist, search);
		seg.setInputCloud(in_cloud);
		seg.segment(*inliers, *coeff);
		if (inliers->indices.size() == 0) {
			// no line found
			break;
		}

		// check for a minimum number of expected inliers
		if ((double)inliers->indices.size() < segm_min_inliers) {
			//logger->log_warn(name(), "[L %u] no more lines (%zu inliers, required %u)",
			//	       loop_count_, inliers->indices.size(), segm_min_inliers);
			break;
		}

		//logger->log_info(name(), "[L %u] Found line with %zu inliers",
		//		     loop_count_, inliers->indices.size());

		// Cluster within the line to make sure it is a contiguous line
		// the line search can output a line which combines lines at separate
		// ends of the field of view...

		typename pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree_line_cluster(
		  new pcl::search::KdTree<pcl::PointXYZ>());
		typename pcl::search::KdTree<pcl::PointXYZ>::IndicesConstPtr search_indices(
		  new std::vector<int>(inliers->indices));
		kdtree_line_cluster->setInputCloud(in_cloud, search_indices);

		std::vector<pcl::PointIndices>             line_cluster_indices;
		pcl::EuclideanClusterExtraction<pcl::PointXYZ> line_ec;
		line_ec.setClusterTolerance(cluster_tolerance);
		size_t min_size = (size_t)floorf(cluster_quota * inliers->indices.size());
		line_ec.setMinClusterSize(min_size);
		line_ec.setMaxClusterSize(inliers->indices.size());
		line_ec.setSearchMethod(kdtree_line_cluster);
		line_ec.setInputCloud(in_cloud);
		line_ec.setIndices(inliers);
		line_ec.extract(line_cluster_indices);

		pcl::PointIndices::Ptr line_cluster_index;
		if (!line_cluster_indices.empty()) {
			line_cluster_index = pcl::PointIndices::Ptr(new pcl::PointIndices(line_cluster_indices[0]));
		}

		// re-calculate coefficients based on line cluster only
		if (line_cluster_index) {
			pcl::SACSegmentation<pcl::PointXYZ> segc;
			segc.setOptimizeCoefficients(true);
			segc.setModelType(pcl::SACMODEL_LINE);
			segc.setMethodType(pcl::SAC_RANSAC);
			segc.setMaxIterations(segm_max_iterations);
			segc.setDistanceThreshold(segm_distance_threshold);
			segc.setInputCloud(in_cloud);
			segc.setIndices(line_cluster_index);
			pcl::PointIndices::Ptr tmp_index(new pcl::PointIndices());
			segc.segment(*tmp_index, *coeff);
			*line_cluster_index = *tmp_index;
		}

		// Remove the linear or clustered inliers, extract the rest
		typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>());
		typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_line(new pcl::PointCloud<pcl::PointXYZ>());
		pcl::ExtractIndices<pcl::PointXYZ>           extract;
		extract.setInputCloud(in_cloud);
		extract.setIndices(
		  (line_cluster_index && !line_cluster_index->indices.empty()) ? line_cluster_index : inliers);
		extract.setNegative(false);
		extract.filter(*cloud_line);

		extract.setNegative(true);
		extract.filter(*cloud_f);
		*in_cloud = *cloud_f;

		if (!line_cluster_index || line_cluster_index->indices.empty())
			continue;

        pcl::PointXYZ pt1, pt2;
        float line_length = 0.0f;
        // Point on the line
        Eigen::Vector3f point_on_line(
                                    coeff->values[0],
                                    coeff->values[1],
                                    coeff->values[2]);
        // Direction vector of the line
        Eigen::Vector3f direction(coeff->values[3],
                                coeff->values[4],
                                coeff->values[5]);
        direction.normalize();

        // Determine the minimum and maximum projection onto the line
        float min_proj =  std::numeric_limits<float>::infinity();
        float max_proj = -std::numeric_limits<float>::infinity();

        for (auto idx : inliers->indices) {
            const auto &p = in_cloud->points[idx];
            Eigen::Vector3f point(p.x, p.y, p.z);
            float projection = direction.dot(point - point_on_line);

            if (projection < min_proj) {
                min_proj = projection;
            }
            if (projection > max_proj) {
                max_proj = projection;
            }
        }

        // Compute the actual endpoints of the line
        Eigen::Vector3f start_point = point_on_line + min_proj * direction;
        Eigen::Vector3f end_point   = point_on_line + max_proj * direction;

        pt1.x = start_point.x();
        pt1.y = start_point.y();
        pt1.z = start_point.z();

        pt2.x = end_point.x();
        pt2.y = end_point.y();
        pt2.z = end_point.z();

        line_length = (end_point - start_point).norm();
        
        // if (line_length == 0 || (min_length >= 0 && line_length < min_length)
       //                   || (max_length >= 0 && line_length > max_length)) {
       if (line_length == 0 || (line_length < 0.6) || (line_length > 0.8)){
        continue;
                }

        laser_scan_integrator_msg::msg::LineSegment line_msg;
        line_msg.frame_id = integratedFrameId_;// CHange later
        line_msg.end_point1.x = pt1.x;
        line_msg.end_point1.y = pt1.y;
        line_msg.end_point1.z = pt1.z;
        line_msg.end_point2.x = pt2.x;
        line_msg.end_point2.y = pt2.y;
        line_msg.end_point2.z = pt2.z;
        line_msg.line_length = line_length;

		detected_lines.push_back(line_msg);
	}

	if (remaining_cloud) {
		*remaining_cloud = *in_cloud;
	}

	return detected_lines;
}

// Methode zum Erstellen und sofortigen Publizieren von Linien-Markern in RViz
void publishLineMarkers(const std::vector<laser_scan_integrator_msg::msg::LineSegment> &lines,
                        const std_msgs::msg::Header &header)
{
  for (const auto &line : lines) {
    visualization_msgs::msg::Marker marker;
    marker.header = header;                  // Übernahme des Headers
    marker.ns = "line_markers";              // Namespace für die Linien-Marker
    static int marker_id = 0;                // Eindeutige ID für jeden Marker
    marker.id = marker_id++;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST; // Verwendung von LINE_LIST, da je Marker zwei Punkte definiert
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Skalierung: Die Breite der Linie
    marker.scale.x = 0.05; // Passen Sie diesen Wert bei Bedarf an

    // Farbe: Beispielweise ein kräftiges Grün (voll opak)
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    // Unbegrenzte Lebensdauer
    marker.lifetime = rclcpp::Duration(0, 0);

    // Definieren der Endpunkte der Linie
    geometry_msgs::msg::Point p1, p2;
    p1.x = line.end_point1.x;
    p1.y = line.end_point1.y;
    p1.z = line.end_point1.z;
    p2.x = line.end_point2.x;
    p2.y = line.end_point2.y;
    p2.z = line.end_point2.z;

    marker.points.push_back(p1);
    marker.points.push_back(p2);

    // Publizieren des Markers
    marker_pub_->publish(marker);
    RCLCPP_INFO(this->get_logger(), "Linienmarker %d veröffentlicht: (%.3f, %.3f) bis (%.3f, %.3f)", 
                marker.id, p1.x, p1.y, p2.x, p2.y);
  }
}


  void update_point_cloud_rgb() {pcl::PointCloud<pcl::PointXYZ>::Ptr
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
      // **Integration of line detection**

      // Convert the integrated laser scan into a point cloud
      auto pointcloud = laser_scan_to_pointcloud(integrated_msg_);
      //publishPointCloud(integrated_msg_);
      // Detect lines in the point cloud with a target length of 70 cm and a tolerance of 2 cm
      //auto lines = detect_lines(pointcloud, 0.7f, 0.02f);
      auto lines = calc_lines(pointcloud);

      // Convert the detected lines into a ROS message format
      laser_scan_integrator_msg::msg::LineSegments lines_msg;
      lines_msg.header = integrated_msg_->header; // Retain the original message header for consistency
      lines_msg.segments = lines; // Add the detected line segments

      // Publish the line segments as a ROS message
      line_segments_pub_->publish(lines_msg);
      publishLineMarkers(lines, integrated_msg_->header);
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