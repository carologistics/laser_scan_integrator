#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// Custom Msgs
#include "laser_scan_integrator_msg/msg/line_segments.hpp"
#include "laser_scan_integrator_msg/msg/line_segment.hpp"

using namespace std::chrono_literals;

/**
 * @brief Prüft, ob ein gegebener LineSegment zu einer Maschine gehört.
 *        (Platzhalter-Implementierung, wie vorher gezeigt)
 */
bool belongsToMachine(
  const laser_scan_integrator_msg::msg::LineSegment &segment,
  const geometry_msgs::msg::TransformStamped &machine_tf,
  double position_tolerance,
  double orientation_tolerance)
{
  // 1) Mittelpunkte u. Orientierung aus Segment holen
  const auto &p1 = segment.end_point1;
  const auto &p2 = segment.end_point2;
  
  geometry_msgs::msg::Point segment_mid;
  segment_mid.x = 0.5 * (p1.x + p2.x);
  segment_mid.y = 0.5 * (p1.y + p2.y);
  segment_mid.z = 0.5 * (p1.z + p2.z);

  double segment_bearing = segment.bearing; // Annahme: in rad
  
  // 2) Maschine-Pose aus TF extrahieren
  double machine_x = machine_tf.transform.translation.x;
  double machine_y = machine_tf.transform.translation.y;
  
  tf2::Quaternion q_machine;
  tf2::fromMsg(machine_tf.transform.rotation, q_machine);

  double roll, pitch, yaw;
  tf2::Matrix3x3(q_machine).getRPY(roll, pitch, yaw);
  
  // 3) Distanz zum Mittelpunkt
  double dx = segment_mid.x - machine_x;
  double dy = segment_mid.y - machine_y;
  double dist = std::sqrt(dx*dx + dy*dy);

  if (dist > position_tolerance) {
    return false;
  }

  // 4) Orientierungs-Differenz
  double orient_diff = std::fabs(segment_bearing - yaw);
  if (orient_diff > M_PI) {
    orient_diff = 2.0 * M_PI - orient_diff;
  }
  if (orient_diff > orientation_tolerance) {
    return false;
  }

  return true;
}


/**
 * @brief Beispiel-Node, der:
 *  1) line_segments abonniert
 *  2) alle Maschinen-Frames [machine1..machine14] aus TF abfragt
 *  3) prüft, ob eine Linie zu einer dieser Maschinen passt
 *  4) ggf. eine korrigierte TF für die Maschine publiziert
 */
class MapperNode : public rclcpp::Node
{
public:
  MapperNode()
  : Node("mapper")
  , tf_buffer_(this->get_clock())
  , tf_listener_(tf_buffer_)
  , tf_broadcaster_(this)
  {
    // Liste der Maschinen-Namen (wie gewünscht nur die Namen!)
    machine_frame_ids_ = {
      "machine1", "machine2", "machine3", "machine4", "machine5", "machine6", "machine7",
      "machine8", "machine9", "machine10","machine11","machine12","machine13","machine14"
    };

    // Subscriber für die Segmente
    line_segments_sub_ = this->create_subscription<laser_scan_integrator_msg::msg::LineSegments>(
      "line_segments",
      10,
      std::bind(&MapperNode::lineSegmentsCallback, this, std::placeholders::_1)
    );

    // Toleranz-Parameter
    position_tolerance_ = this->declare_parameter<double>("position_tolerance", 0.2);
    orientation_tolerance_ = this->declare_parameter<double>("orientation_tolerance", 0.2);

    RCLCPP_INFO(this->get_logger(), "MapperNode gestartet. Abonniert 'line_segments'.");
  }

private:
  void lineSegmentsCallback(const laser_scan_integrator_msg::msg::LineSegments::SharedPtr msg)
  {
    // Für jedes Segment schauen wir, ob es zu einer der Maschinen gehören könnte
    for (const auto & segment : msg->segments)
    {
      // 1) Durchlaufe alle Maschinen-Namen
      for (const auto & machine_name : machine_frame_ids_)
      {
        geometry_msgs::msg::TransformStamped machine_tf;
        try {
          // z.B. "map" als Ziel-Frame, machine_name als Quell-Frame
          // Du musst anpassen, welche Frames in deinem System Sinn ergeben.
          machine_tf = tf_buffer_.lookupTransform(
            "map",
            machine_name,
            tf2::TimePointZero
          );
        } catch (tf2::TransformException &ex) {
          // Falls der TF nicht verfügbar ist, überspringen
          // (z.B. Maschine existiert gerade nicht)
          RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000, // 2 Sek
            "Fehler beim TF-Lookup von '%s': %s",
            machine_name.c_str(),
            ex.what()
          );
          continue;
        }

        // 2) Linie vs. Maschine checken
        if (belongsToMachine(segment, machine_tf, position_tolerance_, orientation_tolerance_))
        {
          // 3) Neue korrigierte TF generieren und publizieren
          geometry_msgs::msg::TransformStamped corrected_tf;
          corrected_tf.header.stamp = this->now();
          corrected_tf.header.frame_id = machine_tf.header.frame_id; 
          // => z.B. "map"

          // Neuer Child-Frame => z.B. "machine_corrected_<machine_name>"
          corrected_tf.child_frame_id = "machine_corrected_" + machine_name;

          // Beispiel: wir verschieben um 0.05 in x (reine Demo)
          corrected_tf.transform.translation.x = machine_tf.transform.translation.x + 0.05;
          corrected_tf.transform.translation.y = machine_tf.transform.translation.y;
          corrected_tf.transform.translation.z = machine_tf.transform.translation.z;

          // Orientation übernehmen
          corrected_tf.transform.rotation = machine_tf.transform.rotation;

          tf_broadcaster_.sendTransform(corrected_tf);

          RCLCPP_INFO(
            this->get_logger(),
            "LineSegment passt zu [%s]. Neue TF '%s' veröffentlicht.",
            machine_name.c_str(),
            corrected_tf.child_frame_id.c_str()
          );
        }
      } // Ende Maschinen-Schleife
    } // Ende Segment-Schleife
  }

  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  // Subscriber
  rclcpp::Subscription<laser_scan_integrator_msg::msg::LineSegments>::SharedPtr line_segments_sub_;

  // Parameter
  double position_tolerance_;
  double orientation_tolerance_;

  // Nur die Namen der Maschinen-Frames
  std::vector<std::string> machine_frame_ids_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MapperNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
