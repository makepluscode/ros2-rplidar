#include <chrono>
#include <cstdio>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "rplidar.h"

#define IS_OK(x)    ( ((x) & RESULT_FAIL_BIT) == 0 )
#define IS_FAIL(x)  ( ((x) & RESULT_FAIL_BIT) )

using namespace std::chrono_literals;
using namespace rp::standalone::rplidar;

class RPLidarPublisher : public rclcpp::Node
{
public:
  RPLidarPublisher()
  : Node("rplidar_publisher"), count_(0)
  {
    // driver
    RCLCPP_INFO(this->get_logger(), "RPLIDAR SDK version: '%s'", RPLIDAR_SDK_VERSION);

    if(!initDriver())
    {
      exit(1);
    }

    if(!connectDevice())
    {
      exit(1);
    }

    // publisher
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
    500ms, std::bind(&RPLidarPublisher::timer_callback, this));
  }

private:
  RPlidarDriver *drv;

  bool initDriver()
  {
    drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);

    if (!drv)
    {
      RCLCPP_ERROR(this->get_logger(), "fail to initialize the driver");
      return false;
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "initialize the driver successfully");
    }

    return true;
  }

  bool connectDevice()
  {
    int64_t op_result;
    rplidar_response_device_info_t devinfo;

    bool is_connected = false;

    if(IS_FAIL(drv->connect("/dev/ttyUSB0", 115200)))
    {
      RCLCPP_ERROR(this->get_logger(), "fail to connect the device");
      return false;
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "connect the device successfully");
      op_result = drv->getDeviceInfo(devinfo);

      if (IS_FAIL(op_result))
      {
        RCLCPP_ERROR(this->get_logger(), "fail to get the device's information");
        delete drv;
        drv = NULL;
        return false;      
      }
      else
      {
        is_connected = true;
        RCLCPP_INFO(this->get_logger(), "read the device information successfully");
      }

      drv->startMotor();
      drv->startScan(0,1);
    }
    // print out the device serial number, firmware and hardware version number..
    RCLCPP_INFO(this->get_logger(), "Firmware Ver: %d.%02d",
              devinfo.firmware_version>>8,
              devinfo.firmware_version & 0xFF
    );
    RCLCPP_INFO(this->get_logger(), "Hardware Rev: %d",
              (int)devinfo.hardware_version
    );
    RCLCPP_INFO(this->get_logger(), "");
    RCLCPP_INFO(this->get_logger(), "");

    return true;
  }

  void timer_callback()
  {
    u_result op_result;
    rplidar_response_measurement_node_hq_t node;
    auto message = std_msgs::msg::String();
    size_t   count = 1;

    op_result = drv->grabScanDataHq(&node, count);

    if (IS_OK(op_result)) {
      std::ostringstream ss;
      ss << "theta=" << (node.angle_z_q14 * 90.f / (1 << 14))
        << " dist=" << (node.dist_mm_q2/4.0f);

      message.data = ss.str();
    }
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<RPLidarPublisher>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
