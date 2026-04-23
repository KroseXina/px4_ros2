#include <xtmission.hpp>
#include <xtlandaction.hpp>

void XTLandAction::landstep()
{
    Eigen::Vector3f target{};
    Eigen::Vector3f delta{};
    float desired_yaw;
    float current_z = _local_pos->positionNed().z();
    float current_vz = _local_pos->velocityNed().z();
    double dx,dy;
    std_msgs::msg::Bool vision_trig_msg;

    switch (_state)
    {
    case land_state::GOTO_HOME:
        target <<
            _home_pos.x,
            _home_pos.y,
            _home_pos.z - static_cast<float>(_altitude);  // z为负值

        delta = target - _local_pos->positionNed();
        desired_yaw = std::atan2(delta.y(), delta.x());

        if(fabsf(_local_pos->heading() - desired_yaw) > 0.1f) // 先完成偏航
            _setpoint->update(_local_pos->positionNed(),desired_yaw);
        else
            _setpoint->update(target,desired_yaw,_speed);

        if(reached(target))
            _state = land_state::DESCEND;
        break;

    case land_state::DESCEND:
        if(fabsf(_local_pos->heading() - _home_pos.yaw) > 0.1f) // 先完成偏航
            target =  _local_pos->positionNed();

        else if(current_z < -3.0f) // 3m以外启动相机
            {
                target <<
                    _home_pos.x,
                    _home_pos.y,
                    current_z + 1.0f;
                if(!_vision_trig)
                {
                    vision_trig_msg.data = true;
                    _vision_enable_pub->publish(vision_trig_msg);
                    _vision_trig = true;
                }
            }

        else if(_vision_valid)  // 3m以内使用视觉降落
            {
                dx = _vision_msg.y*cos(_local_pos->heading()) + _vision_msg.x*sin(_local_pos->heading());
                dy = _vision_msg.y*sin(_local_pos->heading()) - _vision_msg.x*cos(_local_pos->heading());
                target <<
                    _local_pos->positionNed().x() - dx,
                    _local_pos->positionNed().y() - dy,
                    current_z + 0.5f;
            }

        else // 如果没有视觉信号则按照home降落到底
            target <<
                _home_pos.x,
                _home_pos.y,
                current_z + 0.5f;

        _setpoint->update(target, _home_pos.yaw);

        if(current_vz < 0.005 && (_home_pos.z - current_z) < 0.1f)
            _state = land_state::FINISHED;
        break;

    case land_state::FINISHED:
        RCLCPP_INFO(_node.get_logger(), "Landing complete");
        _timer->cancel();
        _on_completed();
        _state = land_state::GOTO_HOME;
        _vision_trig = false;
        vision_trig_msg.data = false;
        _vision_enable_pub->publish(vision_trig_msg);  // 停止相机工作
        break;
    }
}

bool XTLandAction::reached(const Eigen::Vector3f & target)
{
    Eigen::Vector3f pos = _local_pos->positionNed();

    float dist_xy = std::hypot(
        pos.x() - target.x(),
        pos.y() - target.y()
    );

    float dist_z = std::abs(pos.z() - target.z());

    return((dist_xy < 0.5f) && (dist_z<0.5f));
}