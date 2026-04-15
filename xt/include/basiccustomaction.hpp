/****************************************************************************
 * Copyright (c) 2025 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include <px4_ros2/mission/mission_executor.hpp>
#include <px4_ros2/third_party/nlohmann/json.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <px4_ros2/control/setpoint_types/multicopter/goto.hpp>

class BasicCustomAction : public px4_ros2::ActionInterface {
 public:
  explicit BasicCustomAction(px4_ros2::ModeBase& mode) : _node(mode.node()) 
  {
    _hover_setpoint = std::make_shared<px4_ros2::MulticopterGotoSetpointType>(mode);
    _local_pos      = std::make_shared<px4_ros2::OdometryLocalPosition>(mode);
  }

  std::string name() const override { return "basicCustomAction"; }

  void run(const std::shared_ptr<px4_ros2::ActionHandler>& handler,
           const px4_ros2::ActionArguments& arguments,
           const std::function<void()>& on_completed) override
  {
    RCLCPP_INFO(_node.get_logger(), "Running custom action");
    if (!_local_pos->positionXYValid()) 
    {
      RCLCPP_ERROR(_node.get_logger(), "Position invalid, cannot hover!");
      on_completed();
      return;
    }

    // 记录当前位置用以悬停
    _hover_position = _local_pos->positionNed();
    _hover_heading = _local_pos->heading();

    if (arguments.contains("customArgument")) 
    {
      const double arg = arguments.at<double>("customArgument");
      _start_time = _node.get_clock()->now();

      _timer = _node.create_wall_timer(
        100ms,
        [this,on_completed,arg](){
          // 维持在当前位置悬停
          if((_node.get_clock()->now() - _start_time).seconds() > arg)
          {
            _timer->cancel();
            on_completed();
            return;
          }

          _hover_setpoint->update(_hover_position,_hover_heading);

        });
    }
  }

 private:
  rclcpp::TimerBase::SharedPtr  _timer;
  rclcpp::Node& _node;
  rclcpp::Time  _start_time;

  std::shared_ptr<px4_ros2::MulticopterGotoSetpointType> _hover_setpoint;
  std::shared_ptr<px4_ros2::OdometryLocalPosition>       _local_pos;

  Eigen::Vector3f _hover_position;
  double _hover_heading = 0.0;
};
