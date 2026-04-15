/****************************************************************************
 * Copyright (c) 2024 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <xtmission.hpp>

// 初始化一个mission
void XTMission::missionInit(double alt)
{
  nlohmann::json mission_json = {
    {"version", 1},
    {"mission", {
      {"defaults", {
        {"horizontalVelocity", 5.0},
        {"maxHeadingRate", 60.0}
      }},
      {"items", nlohmann::json::array({
        {{"type", "takeoff"},
         {"altitude", alt}},
        {{"type", "rtl"}}
      })}
    }}
  };
  const auto mission_init = px4_ros2::Mission(mission_json);
  _mission_executor->setMission(mission_init);
}

// 添加waypoint
void XTMission::addWaypoint(double lat, double lon, double alt)
{
  std::lock_guard<std::mutex> lock(_mutex);
  const auto& current_mission = _mission_executor->mission();
  std::vector<px4_ros2::MissionItem> items = current_mission.items();

  px4_ros2::Waypoint wp(
    Eigen::Vector3d(lat,lon,alt),
    px4_ros2::MissionFrame::Global
  );

  px4_ros2::NavigationItem nav(wp);

  int insert_index;
  for (int i = 0; i < static_cast<int>(items.size()); ++i) 
  {
    if (std::holds_alternative<px4_ros2::ActionItem>(items[i])) {
      const auto& act = std::get<px4_ros2::ActionItem>(items[i]);
      if (act.name == "rtl") {
        insert_index = i;
        break;
      }
    }
  }
  items.insert(items.begin() + insert_index, nav);

  px4_ros2::Mission new_mission(items, current_mission.defaults());
  _mission_executor->setMission(new_mission);
}

// 添加custom action
void XTMission::addCustomAction(const double arg)
{
  std::lock_guard<std::mutex> lock(_mutex);

  const auto& current_mission = _mission_executor->mission();
  std::vector<px4_ros2::MissionItem> items = current_mission.items();

  px4_ros2::ActionItem action(
    "basicCustomAction",
    px4_ros2::ActionArguments(
      nlohmann::json{{"customArgument",arg}}
    )
  );

  int insert_index;
  for (int i = 0; i < static_cast<int>(items.size()); ++i) 
  {
    if (std::holds_alternative<px4_ros2::ActionItem>(items[i])) {
      const auto& act = std::get<px4_ros2::ActionItem>(items[i]);
      if (act.name == "rtl") {
        insert_index = i;
        break;
      }
    }
  }
  items.insert(items.begin() + insert_index, action);

  px4_ros2::Mission new_mission(items, current_mission.defaults());
  _mission_executor->setMission(new_mission);
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
}

void XTMission::xt_process()
{
  if(_global_pos->positionValid())
  {
    Eigen::Vector3d pos = _global_pos->position();
    if(doMission) // 测试建立mission,只执行一次
    {
      double lat1 = pos[0]+0.0005;
      double lon1 = pos[1]+0.0005;
      double lat2 = pos[0]+0.0005;
      double lon2 = pos[1]-0.0005;
      publish_transponder_report(1,lat1,lon1);
      publish_transponder_report(2,lat2,lon2);
      XTMission::missionInit(pos[2] + 5);
      XTMission::addWaypoint(lat1,lon1,pos[2] + 5);
      XTMission::addCustomAction(10.0f);
      XTMission::addWaypoint(lat2,lon2,pos[2] + 5);
      doMission = false;
    }
  }
}
