#include <xtserial.hpp>
#include <xtoffboard.hpp>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<XTOffboard>());

	rclcpp::shutdown();
	return 0;
}