#pragma once

#include <px4_ros2/mission/mission_executor.hpp>
#include <px4_ros2/third_party/nlohmann/json.hpp>
#include <px4_ros2/vehicle_state/home_position.hpp>
#include <px4_msgs/msg/home_position.hpp>
#include <px4_ros2/control/setpoint_types/multicopter/goto.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/bool.hpp>

class XTLandAction : public px4_ros2::ActionInterface {
 public:
  explicit XTLandAction(px4_ros2::ModeBase& mode) : _node(mode.node()) 
  {
    _home_pos_sub = _node.create_subscription<px4_msgs::msg::HomePosition>(
      "/fmu/out/home_position_v1",
      rclcpp::QoS(1).best_effort().transient_local(),
      [this](const px4_msgs::msg::HomePosition::SharedPtr msg)
      {std::lock_guard<std::mutex> lock(_mutex);
        _home_pos = *msg;
      });

    _vision_sub = _node.create_subscription<geometry_msgs::msg::Vector3>(
    "/usb_cam/position",
    10,
    [this](const geometry_msgs::msg::Vector3::SharedPtr msg)
    {std::lock_guard<std::mutex> lock(_mutex);
      _vision_msg = *msg;
      _vision_valid = true;
    });

    _vision_enable_pub = _node.create_publisher<std_msgs::msg::Bool>(
      "/usb_cam/enable",10);

    _local_pos = std::make_shared<px4_ros2::OdometryLocalPosition>(mode);
    _setpoint = std::make_shared<px4_ros2::MulticopterGotoSetpointType>(mode);
  }

  std::string name() const override { return "xtlandaction"; }

  void run(const std::shared_ptr<px4_ros2::ActionHandler>& handler,
           const px4_ros2::ActionArguments& arguments,
           const std::function<void()>& on_completed) override
    {
        RCLCPP_INFO(_node.get_logger(), "Running xt_land action");

        if(arguments.contains("altitude")) 
        {
          _altitude = arguments.at<double>("altitude");
          if(arguments.contains("speed")) //若输入了速度指令则返回阶段使用输入速度
            _speed = arguments.at<double>("speed");

          _on_completed = on_completed;

          if(!_home_pos.valid_lpos)
          {
              // 如果home position无效，切换到hold
              RCLCPP_ERROR(_node.get_logger(), "Home invalid, switching to HOLD");
              handler->runAction("hold",px4_ros2::ActionArguments{},[]() {});
              return;
          }

          _timer = _node.create_wall_timer(
              50ms,
              std::bind(&XTLandAction::landstep,this)
          );
        }
    }


 private:
  rclcpp::TimerBase::SharedPtr  _timer;
  rclcpp::Node& _node;
  std::mutex _mutex;
  std::function<void()> _on_completed;

  // home position为缓存机制，px4_msgs直接读取，px4_ros2不可靠
  rclcpp::Subscription<px4_msgs::msg::HomePosition>::SharedPtr _home_pos_sub;
  std::shared_ptr<px4_ros2::OdometryLocalPosition> _local_pos;
  std::shared_ptr<px4_ros2::MulticopterGotoSetpointType> _setpoint;

  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr _vision_sub;
  geometry_msgs::msg::Vector3 _vision_msg;
  bool _vision_valid{false};

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _vision_enable_pub;
  bool _vision_trig{false};

  enum land_state
  {
    GOTO_HOME,
    DESCEND,
    FINISHED
  };

  land_state _state{land_state::GOTO_HOME};

  double _altitude{0.0f};
  double _speed{NAN};
  px4_msgs::msg::HomePosition _home_pos{};

  void landstep();
  bool reached(const Eigen::Vector3f & target);
};
