#include "rclcpp/rclcpp.hpp"

#include <px4_ros2/mission/mission_executor.hpp>
#include <px4_ros2/third_party/nlohmann/json.hpp>
#include <px4_ros2/odometry/global_position.hpp>
#include <px4_msgs/msg/transponder_report.hpp>
#include "basiccustomaction.hpp"

static const std::string kName = "巡天-自主任务";

class XTMission
{
public:
  explicit XTMission(const std::shared_ptr<rclcpp::Node>& node) : _node(node)
  {

    // 创建并注册MissionExecutor
    _mission_executor = std::make_unique<px4_ros2::MissionExecutor>(
      kName, 
      px4_ros2::MissionExecutor::Configuration()
        .addCustomAction<BasicCustomAction>(),
      *node
      );
    if (!_mission_executor->doRegister()) {
      throw std::runtime_error("Failed to register mission executor");
    }

    _context = std::make_shared<px4_ros2::Context>(*node); 
    _global_pos = std::make_shared<px4_ros2::OdometryGlobalPosition>(*_context);

    _transponder_pub = _node->create_publisher<px4_msgs::msg::TransponderReport>("/fmu/in/transponder_report",10);

    _timer = _node->create_wall_timer(
      200ms,
      [this](){
        xt_process();
      }
    );
    
  }

private:
  rclcpp::TimerBase::SharedPtr  _timer;

  // 调用px4_ros
  std::shared_ptr<rclcpp::Node> _node;
  std::unique_ptr<px4_ros2::MissionExecutor> _mission_executor;
  nlohmann::json _mission_json;
  std::mutex _mutex;
  std::shared_ptr<px4_ros2::Context> _context;
  std::shared_ptr<px4_ros2::OdometryGlobalPosition> _global_pos;

  // 调用px4_msgs
  rclcpp::Publisher<px4_msgs::msg::TransponderReport>::SharedPtr        _transponder_pub;

  bool doMission = true;
  
  void missionInit(double alt);
  void addWaypoint(double lat, double lon, double alt);
  void addCustomAction(const double arg);
  void xt_process();

  void publish_transponder_report(uint8_t node_id,double lat,double lon);
};
