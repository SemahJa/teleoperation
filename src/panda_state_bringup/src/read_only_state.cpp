#include <panda_state_bringup/snapshot.hpp>
#include <franka/robot.h>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <fstream>
#include <thread>
#include <ctime>
#include <unistd.h>
uint64_t monotonic_ns(){timespec t{};if(clock_gettime(CLOCK_MONOTONIC,&t))throw std::runtime_error("monotonic clock unavailable");return uint64_t(t.tv_sec)*1000000000ULL+t.tv_nsec;}
std::string host_id(){char host[256]{};if(gethostname(host,sizeof(host)-1))throw std::runtime_error("hostname unavailable");std::ifstream in("/proc/sys/kernel/random/boot_id");std::string boot;std::getline(in,boot);if(boot.empty())throw std::runtime_error("boot identity unavailable");return "host_monotonic:"+std::string(host)+":"+boot;}
int main(int argc,char**argv){
 rclcpp::init(argc,argv);auto node=std::make_shared<rclcpp::Node>("panda_read_only_state");
 try {
  const auto ip=node->declare_parameter<std::string>("robot_ip","");
  const auto frame=node->declare_parameter<std::string>("base_frame","panda_1/panda_link0");
  const double hz=node->declare_parameter<double>("sample_hz",100.0),seconds=node->declare_parameter<double>("duration_seconds",15.0);
  if(ip.empty()||frame.empty()||!std::isfinite(hz)||hz<1||hz>200||!std::isfinite(seconds)||seconds<=0||seconds>300)
   throw std::invalid_argument("robot_ip/frame required; sample_hz1..200; duration_seconds(0,300]");
  if(node->get_parameter("use_sim_time").as_bool())throw std::invalid_argument("simulation clock rejected for hardware observations");
  const std::vector<std::string> topics={"joint_states","measured_ee_pose","read_only_state"};
  for(int i=0;i<10&&rclcpp::ok();++i){rclcpp::spin_some(node);std::this_thread::sleep_for(std::chrono::milliseconds(100));}
  for(const auto& topic:topics)if(node->count_publishers(topic))throw std::runtime_error("measurement topic already has a publisher: "+topic);
  if(!rclcpp::ok()){rclcpp::shutdown();return 0;}
  auto qos=rclcpp::SensorDataQoS().keep_last(100);
  auto joints=node->create_publisher<sensor_msgs::msg::JointState>(topics[0],qos);
  auto pose=node->create_publisher<geometry_msgs::msg::PoseStamped>(topics[1],qos);
  auto meta=node->create_publisher<std_msgs::msg::String>(topics[2],qos);
  const auto host=host_id();const auto session=host+":"+std::to_string(monotonic_ns());
  RCLCPP_INFO(node->get_logger(),"Read-only state acquisition at %.1fHz for %.1fs; no controller, setters, recovery or Hand connection",hz,seconds);
  // These are the only robot API calls: construction, serverVersion, readOnce.
  franka::Robot robot(ip,franka::RealtimeConfig::kIgnore);
  const auto server=robot.serverVersion();
  const auto start=std::chrono::steady_clock::now();auto next=start;
  const auto period=std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(1/hz));
  uint64_t sequence=0,segment=0,last_native=0;
  while(rclcpp::ok()&&std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count()<seconds){
   const auto before=monotonic_ns();const auto state=robot.readOnce();const auto received=monotonic_ns();
   if(state.robot_mode==franka::RobotMode::kMove||state.robot_mode==franka::RobotMode::kGuiding||state.robot_mode==franka::RobotMode::kAutomaticErrorRecovery)
    throw std::runtime_error("robot left stationary observation mode; reader exits without changing robot state");
   if(sequence&&state.time.toMSec()<=last_native)throw std::runtime_error("robot native timestamp duplicate/reset; reader exits, new session required");last_native=state.time.toMSec();
   std_msgs::msg::Header header;header.stamp=node->now();header.frame_id=frame;
   auto sample=panda_state_bringup::project(state,header,before,received,++sequence,host,session,segment,server,ip);
   joints->publish(sample.joints);pose->publish(sample.pose);meta->publish(sample.provenance);
   rclcpp::spin_some(node);next+=period;
   if(next<std::chrono::steady_clock::now())next=std::chrono::steady_clock::now();
   std::this_thread::sleep_until(next);
  }
  RCLCPP_INFO(node->get_logger(),"Read-only acquisition complete: %llu samples",static_cast<unsigned long long>(sequence));
 }catch(const std::exception&e){RCLCPP_ERROR(node->get_logger(),"Read-only reader stopped: %s",e.what());rclcpp::shutdown();return 1;}
 rclcpp::shutdown();return 0;
}
