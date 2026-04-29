#include "rclcpp/rclcpp.hpp"

#include <array>
#include <math.h>
#include <px4_ros2/mission/mission_executor.hpp>
#include <px4_ros2/third_party/nlohmann/json.hpp>
#include <px4_ros2/odometry/global_position.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/transponder_report.hpp>

#include "xtbasicaction.hpp"
#include "xtlandaction.hpp"
#include "xtserial.hpp"

static constexpr const char* kName = "巡天-投喂";

class XTMission
{
public:
  explicit XTMission(const std::shared_ptr<rclcpp::Node>& node, XTserial& serial) 
  : _node(node), _serial(serial)
  {

    // 创建并注册MissionExecutor
    _mission_executor = std::make_unique<px4_ros2::MissionExecutor>(
      kName, 
      px4_ros2::MissionExecutor::Configuration()
        .addCustomAction<XTBasicAction>()
        .addCustomAction<XTLandAction>(),
      *node
      );
    if (!_mission_executor->doRegister()) {
      throw std::runtime_error("Failed to register mission executor");
    }

    _context = std::make_shared<px4_ros2::Context>(*node); 
    _global_pos = std::make_shared<px4_ros2::OdometryGlobalPosition>(*_context);
    _local_pos = std::make_shared<px4_ros2::OdometryLocalPosition>(*_context);

    _vehicle_status_sub = _node->create_subscription<px4_msgs::msg::VehicleStatus>(
      "/fmu/out/vehicle_status_v3",
      rclcpp::QoS(1).best_effort().transient_local(),
      [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg)
      {std::lock_guard<std::mutex> lock(_mutex);
        _arm_state = msg->arming_state;
      });
    _transponder_pub = _node->create_publisher<px4_msgs::msg::TransponderReport>("/fmu/in/transponder_report",10);

    if(!_serial.start("/dev/ttyAMA2",115200))
      RCLCPP_ERROR(_node->get_logger(), "Open uart failed!");

    _timer = _node->create_wall_timer(
      200ms,
      [this](){
        xt_process();
      }
    );
    
  }

private:
  rclcpp::TimerBase::SharedPtr  _timer;
  std::shared_ptr<rclcpp::Node> _node;
  XTserial& _serial;

  std::unique_ptr<px4_ros2::MissionExecutor> _mission_executor;
  nlohmann::json _mission_json;
  std::mutex _mutex;
  std::shared_ptr<px4_ros2::Context> _context;
  std::shared_ptr<px4_ros2::OdometryGlobalPosition> _global_pos;
  std::shared_ptr<px4_ros2::OdometryLocalPosition>  _local_pos;

  // 调用px4_msgs
  rclcpp::Publisher<px4_msgs::msg::TransponderReport>::SharedPtr        _transponder_pub;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr         _vehicle_status_sub;

  uint8_t _arm_state;
  
  void missionInit(double alt, double speed);
  void addLandAction(const double alt, const double speed, const float home_z);
  void addWaypoint(double lat, double lon, double alt);
  void addCustomAction(const double weight);
  void commitMission();
  void xt_process();

  void publish_transponder_report(uint8_t node_id,double lat,double lon);
  static constexpr int MAX_TARGET = 50;
  std::array<px4_msgs::msg::TransponderReport,MAX_TARGET> _targets;
  int _target_count{0};
  bool _mission_init_commit{false};
  int _last_commit_count{0};
  std::vector<px4_ros2::MissionItem> _mission_items;
  px4_ros2::MissionDefaults          _defaults;

  // 防止高度跳变/home跳变导致降落失败及下次起飞高度异常，使用解锁前的多次高度取均值作为参考
  double ALT_STABLE_WIN = 1.0;   // s
  double ALT_STD_TH     = 0.2;   // m，稳定阈值
  std::size_t    MIN_SAMPLES    = 20;    //最少采集20个起飞前高度

  std::deque<double> _alt_buf;
  std::deque<float> _local_alt_buf;
  double _takeoff_alt_amsl{NAN};
  float _takeoff_alt_ned{NAN};
  bool   _alt_locked{false};
  bool _vehicle_armed{false};

  bool computeStableAlt(double &mean_out);
  bool computeStableAltNed(float &mean_out);
};
