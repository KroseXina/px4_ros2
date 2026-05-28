#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/transponder_report.hpp>
#include <px4_msgs/msg/home_position.hpp>
#include <px4_msgs/msg/xt_main_out.hpp>

#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <math.h>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/bool.hpp>

#include <chrono>
#include <iostream>
#include "xtserial.hpp"

using namespace std::chrono;
using namespace std::chrono_literals;

class XTOffboard : public rclcpp::Node
{
public:
    XTOffboard() : Node("XT_offboard"),_serial()
    {
        _offboard_control_mode_pub = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		_trajectory_setpoint_pub = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		_vehicle_command_pub = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
        _transponder_pub = this->create_publisher<px4_msgs::msg::TransponderReport>("/fmu/in/transponder_report", 10);
        _vision_enable_pub = this->create_publisher<std_msgs::msg::Bool>("/usb_cam/enable",10);

        _vehicle_status_sub = this->create_subscription<px4_msgs::msg::VehicleStatus>(
        "/fmu/out/vehicle_status_v3",
        rclcpp::QoS(1).best_effort(),
        [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg)
        {std::lock_guard<std::mutex> lock(_mutex);
            _vehicle_status = *msg;
        });

        _vehicle_local_pos_sub = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position_v1",
        rclcpp::QoS(1).best_effort(),
        [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
        {std::lock_guard<std::mutex> lock(_mutex);
            _vehicle_local_pos = *msg;
        });

        _xt_out_sub = this->create_subscription<px4_msgs::msg::XtMainOut>(
        "/fmu/out/xt_main_out",
        rclcpp::QoS(1).best_effort(),
        [this](const px4_msgs::msg::XtMainOut::SharedPtr msg)
        {std::lock_guard<std::mutex> lock(_mutex);
            _xt_out = *msg;
        });

        _home_pos_sub = this->create_subscription<px4_msgs::msg::HomePosition>(
        "/fmu/out/home_position_v1",
        rclcpp::QoS(1).best_effort(),
        [this](const px4_msgs::msg::HomePosition::SharedPtr msg)
        {std::lock_guard<std::mutex> lock(_mutex);
            _home_pos = *msg;
        });

        _vision_sub = this->create_subscription<geometry_msgs::msg::Vector3>(
        "/usb_cam/position",
        10,
        [this](const geometry_msgs::msg::Vector3::SharedPtr msg)
        {std::lock_guard<std::mutex> lock(_mutex);
            _vision_msg = *msg;
            _vision_valid = true;
        });
        
        _vision_trig_msg.data = false;

        if(!_serial.start("/dev/ttyAMA2",115200))
            RCLCPP_ERROR(this->get_logger(), "Open uart failed!");

        _timer = this->create_wall_timer(
                50ms,
                [this](){
                    xt_process();
                });
    }

private:
    XTserial _serial;

    rclcpp::TimerBase::SharedPtr _timer;
    std::mutex _mutex;

    enum _offboard_state
    {
        ENTER,
        DOYAW,
        LAND
    };

    _offboard_state _state{ENTER};

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr    _offboard_control_mode_pub;
	rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr     _trajectory_setpoint_pub;
	rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr         _vehicle_command_pub;
    rclcpp::Publisher<px4_msgs::msg::TransponderReport>::SharedPtr      _transponder_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                   _vision_enable_pub;

    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr        _vehicle_status_sub;
    px4_msgs::msg::VehicleStatus _vehicle_status;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr _vehicle_local_pos_sub;
    px4_msgs::msg::VehicleLocalPosition _vehicle_local_pos;
    rclcpp::Subscription<px4_msgs::msg::XtMainOut>::SharedPtr            _xt_out_sub;
    px4_msgs::msg::XtMainOut _xt_out; 
    rclcpp::Subscription<px4_msgs::msg::HomePosition>::SharedPtr         _home_pos_sub;
    px4_msgs::msg::HomePosition _home_pos;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr         _vision_sub;
    geometry_msgs::msg::Vector3 _vision_msg;

    uint64_t timestamp() {return this->get_clock()->now().nanoseconds() / 1000;};
    void xt_process();
    void publish_transponder_report(uint8_t node_id,double lat,double lon);
    void publish_offboard_control_mode(bool pos,bool vel,bool acc,bool att,bool br);
	void publish_trajectory_setpoint(float x,float y,float z,float vx,float vy,float vz,float yaw);
    void publish_vehicle_command(uint32_t command,float param1,float param2);

    uint8_t _offboard_count{0};
    std_msgs::msg::Bool _vision_trig_msg;
    bool _vision_valid{false};
};
