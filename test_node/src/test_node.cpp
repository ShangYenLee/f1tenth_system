#include <chrono>
#include <memory>
#include <vector>
#include <string>
#include <stdexcept>
#include <iterator>
#include <algorithm>
#include <math.h>
#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
  
using ackermann_msgs::msg::AckermannDriveStamped;
using std_msgs::msg::Float64;
using std::placeholders::_1;
using namespace std::chrono_literals;

float degree2radious(float degree)
{
	return degree*M_PI/180;
}

float DEGREE = -10;

class FTGPublisher : public rclcpp::Node
{
	public:
		FTGPublisher()
		: Node("follow_the_gap_pub")
		{
			// auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
			publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 0);
			timer_ = this->create_wall_timer(500ms, std::bind(&FTGPublisher::PubAndSubCallback, this));
		}
	private:
		void PubAndSubCallback()
		{
			auto message = ackermann_msgs::msg::AckermannDriveStamped();
			message.drive.speed = 1;
			message.drive.steering_angle = degree2radious(DEGREE);
			if(DEGREE < 10)
			{
				DEGREE += 0.5;
			}
			RCLCPP_INFO(this->get_logger(), "Sending VESC Speed '%f'm/s Steering '%f'", message.drive.speed, message.drive.steering_angle);
			publisher_->publish(message);

		}
		rclcpp::TimerBase::SharedPtr timer_;
		rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;

};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<FTGPublisher>());
	rclcpp::shutdown();
	return 0;
}

