#include <chrono>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using namespace std::chrono_literals;

class FrameListener : public rclcpp::Node
{
public:
  FrameListener()
  : Node("test_listner"),
    tf_buffer_(std::make_unique<tf2_ros::Buffer>(this->get_clock())),
    tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
  {
    // Wartezeit, um sicherzustellen, dass der Listener initialisiert ist
    RCLCPP_INFO(this->get_logger(), "Warte 1 Sekunde, um TF-Daten zu laden...");
    //rclcpp::sleep_for(1s);

    // Timer erstellen, der alle 500 ms on_timer() aufruft
    timer_ = this->create_wall_timer(
      500ms, std::bind(&FrameListener::on_timer, this));
  }

private:
  void on_timer()
  {
    std::string fromFrameRel = "robotinobase1/laser_link";
    std::string toFrameRel = "machine1";

    try {
          
        auto transform = tf_buffer_->lookupTransform(toFrameRel, fromFrameRel, tf2::TimePointZero);
        RCLCPP_INFO(this->get_logger(), "Aktuelle Simulationszeit: %f", this->now().seconds());
        RCLCPP_INFO(this->get_logger(), "Neuester Zeitstempel im Transform: %f",
                  rclcpp::Time(transform.header.stamp).seconds());
    } catch (const tf2::TransformException &ex) {
        RCLCPP_INFO(this->get_logger(), "Aktuelle Simulationszeit: %f", this->now().seconds());
        RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s",
                  toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
    }
    


  }

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrameListener>());
  rclcpp::shutdown();
  return 0;
}
