/****************************************************************************
 *
 * Copyright 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
#include <xtoffboard.hpp>

void XTOffboard::publish_transponder_report(uint8_t node_id,double lat,double lon)
{
	px4_msgs::msg::TransponderReport msg{};

	msg.timestamp = timestamp();
	msg.icao_address = 1000 + node_id;
	snprintf(reinterpret_cast<char*>(msg.callsign.data()),
         msg.callsign.size(),
         "XT_%02d",
         node_id);
	msg.lat = lat;
	msg.lon = lon;
	msg.altitude = 0;
	msg.emitter_type = px4_msgs::msg::TransponderReport::ADSB_EMITTER_TYPE_UAV;
	msg.flags = px4_msgs::msg::TransponderReport::PX4_ADSB_FLAGS_VALID_COORDS |
		        px4_msgs::msg::TransponderReport::PX4_ADSB_FLAGS_VALID_ALTITUDE |
              	px4_msgs::msg::TransponderReport::PX4_ADSB_FLAGS_VALID_CALLSIGN |
		        px4_msgs::msg::TransponderReport::PX4_ADSB_FLAGS_RETRANSLATE;

	_transponder_pub->publish(msg);
}

void XTOffboard::publish_offboard_control_mode(bool pos,bool vel,bool acc,bool att,bool br)
{
	px4_msgs::msg::OffboardControlMode msg{};
	msg.timestamp = timestamp();
	msg.position = pos;
	msg.velocity = vel;
	msg.acceleration = acc;
	msg.attitude = att;
	msg.body_rate = br;

	_offboard_control_mode_pub->publish(msg);
}

void XTOffboard::publish_trajectory_setpoint(float x,float y,float z,float vx,float vy,float vz,float yaw)
{
	px4_msgs::msg::TrajectorySetpoint msg{};
	msg.timestamp = timestamp();
	msg.position[0] = x;
	msg.position[1] = y;
	msg.position[2] = z;
	msg.velocity[0] = vx;
	msg.velocity[1] = vy;
	msg.velocity[2] = vz;
	msg.yaw = yaw;

	_trajectory_setpoint_pub->publish(msg);
}

void XTOffboard::publish_vehicle_command(uint32_t command,float param1,float param2)
{
	px4_msgs::msg::VehicleCommand msg{};
	msg.timestamp = timestamp();
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;

	_vehicle_command_pub->publish(msg);
}

void XTOffboard::xt_process()
{
	float vz;
	XTserial::lora_struct lora_s;
	if(_serial.get_data(lora_s))
	{
		publish_transponder_report(lora_s.node_id,
                                  static_cast<double>(lora_s.lat)/1e7,
                                  static_cast<double>(lora_s.lon)/1e7);

	}

	if(_xt_out.switch_to_offboard)
	{
		switch (_state)
		{
		case ENTER:
			//进入offboard
			RCLCPP_INFO_ONCE(this->get_logger(), "Enter Offboard Mode.");
			if(_vehicle_status.nav_state != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD)
			{
				if(_offboard_count == 20)
					publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,1,6);

				publish_offboard_control_mode(1,0,0,0,0);
				publish_trajectory_setpoint(_vehicle_local_pos.x,_vehicle_local_pos.y,_vehicle_local_pos.z,NAN,NAN,NAN,_vehicle_local_pos.heading);

				if(_offboard_count < 21)
					_offboard_count ++;
			}
			else
				_state = DOYAW;
			break;
		
		case DOYAW:
			//启动相机
			_vision_trig_msg.data = true;
			//先完成偏航
			if(fabsf(atan2f(sinf(_home_pos.yaw - _vehicle_local_pos.heading), 
                        	cosf(_home_pos.yaw - _vehicle_local_pos.heading))) > 0.1f)
			{
				publish_offboard_control_mode(0,1,0,0,0);
				publish_trajectory_setpoint(NAN,NAN,NAN,0,0,0,_home_pos.yaw);
			}
			else
			{   
				_state = LAND;
			}
			break;

		case LAND:
			if(_vision_valid)
			{
				RCLCPP_INFO_ONCE(this->get_logger(), "Receive vision msg.");
				double dx = _vision_msg.y*cos(_vehicle_local_pos.heading) + _vision_msg.x*sin(_vehicle_local_pos.heading);
                double dy = _vision_msg.y*sin(_vehicle_local_pos.heading) - _vision_msg.x*cos(_vehicle_local_pos.heading);

				vz = (_vision_msg.z > 4.0f) ? 0.5f : 0.2f;

				//相机物理识别极限为0.15m,安装高度不要超过0.5m
				if(_vision_msg.z < 0.5f  && fabs(_vehicle_local_pos.vz) < 0.1f) // 已经落地
				{
					publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND,NAN,NAN);
				}
				else
				{
					publish_offboard_control_mode(1,1,0,0,0);
					publish_trajectory_setpoint(_vehicle_local_pos.x - dx,_vehicle_local_pos.y - dy,NAN,NAN,NAN,vz,_home_pos.yaw);
				//	publish_offboard_control_mode(1,0,0,0,0);	
				//	publish_trajectory_setpoint(_vehicle_local_pos.x - dx,_vehicle_local_pos.y - dy,
				// 							_vehicle_local_pos.z - (4 - _vision_msg.z),
				// 							NAN,NAN,NAN,_home_pos.yaw);
				}
			}
			else
			{
				//无视觉信号，降落后会自动disarm(但保持offboard，需清除)
				RCLCPP_INFO_ONCE(this->get_logger(), "No valid vision msg, land.");
				vz = (_vehicle_local_pos.z < -2.0f) ? 0.5f : 0.3f;
				publish_offboard_control_mode(1,1,0,0,0);
				publish_trajectory_setpoint(_vehicle_local_pos.x,_vehicle_local_pos.y,NAN,NAN,NAN,vz,_home_pos.yaw);
			}
			break;
		}
	}
	else
	{
		//已经上锁
		if(_vehicle_status.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD)
		{
			//清除offboard模式标志,换为position模式
			publish_offboard_control_mode(0,0,0,0,0);
			publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,1,3);
		}
		//停止相机工作
		_vision_trig_msg.data = false;
		_vision_valid = false;
		_offboard_count = 0;
		_state = ENTER;
	}

	_vision_enable_pub->publish(_vision_trig_msg);
}
