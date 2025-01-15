#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <memory>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

using namespace std::chrono_literals;

class TFPublisher : public rclcpp::Node
{
public:
  TFPublisher(const std::string & file_path)
  : Node("tf_publisher")
  {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    parse_file(file_path);

    // Timer für das periodische Veröffentlichen
    timer_ = this->create_wall_timer(
      500ms, std::bind(&TFPublisher::publish_transforms, this));
  }

private:
  struct MachineData
  {
    std::string name;
    double tx, ty, tz;  // Translation
    double qx, qy, qz, qw; // Rotation (Quaternion)
  };

  std::vector<MachineData> machines_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  void parse_file(const std::string & file_path)
  {
    std::ifstream file(file_path);
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Could not open file: %s", file_path.c_str());
      return;
    }

    std::string line;
    std::getline(file, line); // Überspringe die Kopfzeile

    while (std::getline(file, line)) {
      std::istringstream stream(line);
      MachineData data;
      char ignore; // Für die Klammern und Leerzeichen

      stream >> data.name;
      stream >> ignore >> data.tx >> data.ty >> data.tz >> ignore; // Translation
      stream >> ignore >> data.qx >> data.qy >> data.qz >> data.qw >> ignore; // Rotation

      machines_.push_back(data);
    }

    RCLCPP_INFO(this->get_logger(), "Parsed %lu machines from file.", machines_.size());
  }

  void publish_transforms()
  {
    for (const auto & machine : machines_) {
      geometry_msgs::msg::TransformStamped transform;

      // Header
      transform.header.stamp = this->get_clock()->now();
      transform.header.frame_id = "map";
      transform.child_frame_id = machine.name;

      // Translation
      transform.transform.translation.x = machine.tx;
      transform.transform.translation.y = machine.ty;
      transform.transform.translation.z = machine.tz;

      // Rotation
      transform.transform.rotation.x = machine.qx;
      transform.transform.rotation.y = machine.qy;
      transform.transform.rotation.z = machine.qz;
      transform.transform.rotation.w = machine.qw;

      // Transformation veröffentlichen
      tf_broadcaster_->sendTransform(transform);

      RCLCPP_INFO(this->get_logger(), "Published transform for %s", machine.name.c_str());
    }
  }
};

int main(int argc, char * argv[])
{
  if (argc < 2) {
    std::cerr << "Usage: tf_publisher <file_path>" << std::endl;
    return 1;
  }

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TFPublisher>(argv[1]));
  rclcpp::shutdown();
  return 0;
}
