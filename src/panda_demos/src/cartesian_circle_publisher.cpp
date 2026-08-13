#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <vector>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

class CirclePublisher : public rclcpp::Node {
 public:
  CirclePublisher() : Node("panda_circle_trajectory") {
    radius_ = declare_parameter<double>("radius", 0.10);
    approach_ = declare_parameter<double>("approach_seconds", 6.0);
    period_ = declare_parameter<double>("circle_seconds", 20.0);
    pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        "/panda_cartesian_impedance_controller/target_pose", 1);
    path_ = create_publisher<visualization_msgs::msg::Marker>("~/target_path", 1);
    sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/panda_cartesian_impedance_controller/initial_pose", rclcpp::QoS(1).transient_local(),
        [this](geometry_msgs::msg::PoseStamped::SharedPtr p) { if (!started_) { initial_ = *p; start_ = now(); started_ = true; } });
    timer_ = create_wall_timer(std::chrono::milliseconds(10), [this] { publish(); });
  }
 private:
  static double smooth(double x) { x=std::clamp(x,0.,1.); return x*x*x*(10.+x*(-15.+6.*x)); }
  void publish() {
    if (!started_) return;
    auto p = initial_; p.header.stamp = now(); const double t=(now()-start_).seconds();
    if (t < approach_) p.pose.position.x += radius_ * smooth(t / approach_);
    else { const double a=2.*M_PI*(t-approach_)/period_; p.pose.position.x += radius_*std::cos(a); p.pose.position.y += radius_*std::sin(a); }
    pub_->publish(p);
    visualization_msgs::msg::Marker line;
    line.header.frame_id = "panda_link0"; line.header.stamp = now(); line.ns = "target";
    line.id = 0; line.type = visualization_msgs::msg::Marker::LINE_STRIP; line.action = visualization_msgs::msg::Marker::ADD;
    line.scale.x = 0.004; line.color.r = 0.1; line.color.g = 0.9; line.color.b = 1.0; line.color.a = 1.0;
    geometry_msgs::msg::Point point; point.x=p.pose.position.x; point.y=p.pose.position.y; point.z=p.pose.position.z;
    points_.push_back(point); if (points_.size() > 2500) points_.erase(points_.begin()); line.points = points_; path_->publish(line);
  }
  double radius_, approach_, period_; bool started_{false}; rclcpp::Time start_{0,0,RCL_ROS_TIME}; geometry_msgs::msg::PoseStamped initial_;
  std::vector<geometry_msgs::msg::Point> points_; rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_; rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_; rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_; rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char** argv) { rclcpp::init(argc,argv); rclcpp::spin(std::make_shared<CirclePublisher>()); rclcpp::shutdown(); }
