#include <xtmission.hpp>
#include <xtlandaction.hpp>

void XTLandAction::landstep()
{
    Eigen::Vector3f target{};
    Eigen::Vector3f delta{};
    float desired_yaw;
    float current_z = _local_pos->positionNed().z();
    float current_vz = _local_pos->velocityNed().z();
    float yaw_current = _local_pos->heading();
    double dx,dy;
    std_msgs::msg::Bool vision_trig_msg;

    switch (_state)
    {
    case land_state::GOTO_HOME:
        target <<
            _home_pos.x,
            _home_pos.y,
            _home_z - static_cast<float>(_altitude);  // z为负值

        delta = target - _local_pos->positionNed();
        desired_yaw = std::atan2(delta.y(), delta.x());
        if(fabsf(atan2f(sinf(desired_yaw - yaw_current), 
                        cosf(desired_yaw - yaw_current))) > 0.1f) // 先完成偏航
            _setpoint->update(_local_pos->positionNed(),desired_yaw);
        else
            _setpoint->update(target,desired_yaw,_speed);

        if(reached(target))
        {
            _vision_trig = 0;
            _state = land_state::DESCEND;
        }    
        break;

    case land_state::DESCEND:
        if(fabsf(atan2f(sinf(_home_pos.yaw - yaw_current), 
                        cosf(_home_pos.yaw - yaw_current))) > 0.1f)   //先完成偏航
            target =  _local_pos->positionNed();

        else if(fabs(_home_z - current_z) > 4.0f) // 4m以外启动相机
            {
                target <<
                    _home_pos.x,
                    _home_pos.y,
                    current_z + 1.0f;
                if(_vision_trig < 5) //防止一次启动失败，多发几次
                {
                    vision_trig_msg.data = true;
                    _vision_enable_pub->publish(vision_trig_msg);
                    _vision_trig ++;
                }
            }

        else if(_vision_valid)  // 4m以内使用视觉降落
            {
                dx = _vision_msg.y*cos(yaw_current) + _vision_msg.x*sin(yaw_current);
                dy = _vision_msg.y*sin(yaw_current) - _vision_msg.x*cos(yaw_current);
                target <<
                    _local_pos->positionNed().x() - dx,
                    _local_pos->positionNed().y() - dy,
                    current_z + 0.5f;
            }

        else // 如果没有视觉信号则直接切换为land
            {
                _vision_trig = 0;
                _state = land_state::FINISHED;
                break;
            }
                        
        _setpoint->update(target, _home_pos.yaw);

        if(fabs(current_vz) < 0.1f && _vision_msg.z < 0.2f)
        {
            _vision_trig = 0;
            _state = land_state::FINISHED;
        }
        break;

    case land_state::FINISHED:
        if(_vision_trig < 5)   // 停止相机工作,多发几次
        {
            vision_trig_msg.data = false;
            _vision_enable_pub->publish(vision_trig_msg);
            _vision_trig ++;
        }
        else
        {
            RCLCPP_INFO(_node.get_logger(), "Landing complete");
            _timer->cancel();
            _on_completed();
            _state = land_state::GOTO_HOME;
        }
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