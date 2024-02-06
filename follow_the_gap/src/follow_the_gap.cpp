#include <chrono>
#include <memory>
#include <vector>
#include <string>
#include <stdexcept>
#include <iterator>
#include <algorithm>
#include <math.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
  
using ackermann_msgs::msg::AckermannDriveStamped;
using sensor_msgs::msg::LaserScan;
using std_msgs::msg::Float64;
using std::placeholders::_1;
using namespace std::chrono_literals;

int const DATA_SIZE = 1081;
float const CAR_RADIOUS = 0.21;

class CenterOutsideGapException : public std::logic_error
{
	public:
		CenterOutsideGapException(std::string const & msg)
			: std::logic_error(msg){};
};
class InvalidAngleException : public std::logic_error
{
	public:
		InvalidAngleException(std::string const & msg)
			: std::logic_error(msg){};
};

// Obstacle compute_basic_info(float* scan_data);
// int find_the_maximum_gap(Obstacle obs);
// float calculate_gap_center_angle(Obstacle obs, int max_gap_index);

struct Obstacle
{
	float dist[DATA_SIZE];
	float angle_center[DATA_SIZE];
	float angle_left[DATA_SIZE];
	float angle_right[DATA_SIZE];	
};

Obstacle compute_basic_info(std::vector<float> scan_data)
{
	Obstacle obs;
	for (int i = 0; i < DATA_SIZE; i++)
	{
		float degree = std::asin(CAR_RADIOUS/scan_data[i]);
		obs.angle_center[i] = -135 + i*0.25;
		obs.dist[i] = std::sqrt(std::pow(scan_data[i], 2) - CAR_RADIOUS*CAR_RADIOUS);
		obs.angle_left[i] = obs.angle_center[i] - degree;
		obs.angle_right[i] = obs.angle_center[i] - degree;
	}

	return obs;
}

int find_the_maximum_gap(Obstacle obs)
{
	float gap_array[DATA_SIZE+1] = {0};
	for(int i = 1; i < DATA_SIZE; i++)
	{
		float gap = obs.angle_left[i] - obs.angle_right[i-1];
		if (gap < 0 || obs.angle_center[i-1] <= -15 || obs.angle_center[i] >= 15)
		{
			gap_array[i] = 0;
		}
		else
		{
			gap_array[i] = gap;
		}
	}
	int max_gap_index = (int) std::distance(gap_array, std::max_element(gap_array, gap_array+DATA_SIZE+1));
	return max_gap_index;
}

float calculate_gap_center_angle(Obstacle obs, int max_gap_index)
{
	float const d_1 = obs.dist[max_gap_index];
	float const d_2 = obs.dist[max_gap_index-1];
	float const phi_1 = std::abs(obs.angle_left[max_gap_index]);
	float const phi_2 = std::abs(obs.angle_right[max_gap_index-1]);
	float phi_gap_c = 0;

	if( (obs.angle_left[max_gap_index] >= 0) && (obs.angle_right[max_gap_index-1] <= 0) )
	{
		phi_gap_c = std::acos((d_1+d_2*std::cos(phi_1+phi_2))/(std::sqrt(d_1*d_1+d_2*d_2+2*d_1*d_2*std::cos(phi_1+phi_2))))-phi_1;
	}
	else if(obs.angle_left[max_gap_index] <= 0)
	{
		float l_squared = (d_1*d_1+d_2*d_2-2*d_1*d_2*std::cos(phi_2-phi_1))/4;
		float h_squared = (d_1*d_1+d_2*d_2-2*l_squared)/2;
		float h = std::sqrt(h_squared);
		float phi_x = std::acos((h_squared+d_1*d_1-l_squared)/(2*d_1*h));
		phi_gap_c = -(phi_1+phi_x);
	}
	else
	{
		float l_squared = (d_1*d_1+d_2*d_2-2*d_1*d_2*std::cos(phi_1-phi_2))/4;
		float h_squared = (d_1*d_1+d_2*d_2-2*l_squared)/2;
		float h = std::sqrt(h_squared);
		float phi_x = std::acos((h_squared+d_2*d_2-l_squared)/(2*d_2*h));
		phi_gap_c = phi_2+phi_x;
	}

	if((phi_gap_c > obs.angle_left[max_gap_index]) || (phi_gap_c < obs.angle_right[max_gap_index-1]))
	{
		throw CenterOutsideGapException("The calculated center of gap was outside the gap!");
	}

	if(std::isnan(phi_gap_c))
	{
		throw InvalidAngleException("Gap center angle was nan!");
	}
	return phi_gap_c;
}

float degree2radious(float degree)
{
	return degree*M_PI/180;
}

class FTGPublisher : public rclcpp::Node
{
	public:
		FTGPublisher()
		: Node("follow_the_gap_pub")
		{
			// auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
			publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 0);
			subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", rclcpp::QoS{10}, std::bind(&FTGPublisher::PubAndSubCallback, this, _1));
			// timer_ = this->create_wall_timer(500ms, std::bind(&FTGPublisher::PubAndSubCallback, this));
		}
	private:
		void PubAndSubCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_message)
		{
			auto scan_data = scan_message->ranges;
			std::reverse(scan_data.begin(), scan_data.end());
			Obstacle obs = compute_basic_info(scan_data);
			int max_gap_index = find_the_maximum_gap(obs);
			float gap_center_angle = calculate_gap_center_angle(obs, max_gap_index);
			gap_center_angle = degree2radious(gap_center_angle);
			auto message = ackermann_msgs::msg::AckermannDriveStamped();
			message.drive.speed = 1;
			message.drive.steering_angle = gap_center_angle;
			RCLCPP_INFO(this->get_logger(), "Sending VESC Speed '%f'm/s Steering '%f'", message.drive.speed, message.drive.steering_angle);
			publisher_->publish(message);

		}
		// rclcpp::TimerBase::SharedPtr timer_;
		rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
		rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;

};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<FTGPublisher>());
	rclcpp::shutdown();
	return 0;
}

