/****************************************************************************
 * Copyright (c) 2024 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <xtmission.hpp>

// 初始化一个mission
void XTMission::missionInit(double alt, double speed)
{
  std::lock_guard<std::mutex> lock(_mutex);
  _mission_items.clear();
  _defaults.trajectory_options.horizontal_velocity = speed;

  px4_ros2::ActionItem action(
    "takeoff",
    px4_ros2::ActionArguments(
      nlohmann::json{{"altitude",alt}}
    )
  );

  _mission_items.emplace_back(action);
}

// 设置返航功能，输入返航高度
// 注意：返航使用local坐标系，输入高度为相对高度即可(z为负值)
void XTMission::addLandAction(const double alt, const double speed)
{
  std::lock_guard<std::mutex> lock(_mutex);

  px4_ros2::ActionItem action(
    "xtlandaction",
    px4_ros2::ActionArguments(
      nlohmann::json{{"altitude",alt},
                    {"speed",speed}}
  )
  );

  _mission_items.emplace_back(action);
}

// 添加waypoint，输入经纬高
// 注意：mission中使用的坐标只能是global，意味着高度是AMSL，而不是NED高度，处理为gps.alt + 输入高度
void XTMission::addWaypoint(double lat, double lon, double alt)
{
  std::lock_guard<std::mutex> lock(_mutex);

  px4_ros2::Waypoint wp(
    Eigen::Vector3d(lat,lon,alt),
    px4_ros2::MissionFrame::Global
  );

  px4_ros2::NavigationItem nav(wp);

  _mission_items.emplace_back(nav);
}

// 添加custom action，输入投喂重量
void XTMission::addCustomAction(const double weight)
{
  std::lock_guard<std::mutex> lock(_mutex);

  px4_ros2::ActionItem action(
    "xtbasicaction",
    px4_ros2::ActionArguments(
      nlohmann::json{{"weight",weight}}
    )
  );

  _mission_items.emplace_back(action);
}

void XTMission::commitMission()
{
  std::lock_guard<std::mutex> lock(_mutex);

  // 在任务最后加入land，防止着陆后处于hold模式
  px4_ros2::ActionItem action(
    "land"
  );

  _mission_items.emplace_back(action);

  px4_ros2::Mission mission(_mission_items, _defaults);
  _mission_executor->setMission(mission);
}

void XTMission::publish_transponder_report(uint8_t node_id,double lat,double lon)
{
	px4_msgs::msg::TransponderReport msg{};

	msg.timestamp = _node->get_clock()->now().nanoseconds() / 1000;
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

  //每次更新transponder_report，维护_targets列表
  std::cout<< "recv transponder" << msg.icao_address << std::endl;
  bool found = false;
  for(int i=0;i<_target_count;++i)
	{
		if(_targets[i].icao_address == msg.icao_address)
		{
			_targets[i] = msg;
			found = true;
			break;
		}
	}

	if(!found && _target_count < MAX_TARGET)
	{
		_targets[_target_count++] = msg;
	}

	//为targets列表排序
	for (int i = 0; i < _target_count - 1; ++i)
	{
		for (int j = i + 1; j < _target_count; ++j)
		{
			if (_targets[j].icao_address < _targets[i].icao_address)
			{
				auto tmp = _targets[i];
				_targets[i] = _targets[j];
				_targets[j] = tmp;
			}
		}
	}
}

void XTMission::xt_process()
{
  if(!_global_pos->positionValid())
  {
    RCLCPP_ERROR_ONCE(_node->get_logger(), "No valid global position!");
    return;
  }
  
  Eigen::Vector3d pos = _global_pos->position();
  double alt = 5.0f;
  double speed = 3.0f;

  if(!_mission_init_commit)
  {
    missionInit(pos[2]+alt, speed);
    addWaypoint(pos[0], pos[1], pos[2]+alt);
    addCustomAction(5.0f);
    addLandAction(alt,speed);
    commitMission();
    _mission_init_commit = true;
  }

  XTserial::lora_struct lora_s;
  if(_serial.get_data(lora_s))
  {
    publish_transponder_report(lora_s.node_id,
                              static_cast<double>(lora_s.lat)/1e7,
                              static_cast<double>(lora_s.lon)/1e7);
  }

  // 将targets列表生成mission，当targets数量变化时刷新一次
  // TODO:根据控制软件指令生成mission,并获取相关参数
  if(_last_commit_count != _target_count)
    {
      _last_commit_count = _target_count;

      missionInit(pos[2]+alt, speed);
      for(int i=0;i<_target_count;i++)
      {
        addWaypoint(_targets[i].lat, _targets[i].lon, pos[2] + alt);
        addCustomAction((i+1)*5.0f);
      }
      addLandAction(alt,speed);
      commitMission();
    }
}
