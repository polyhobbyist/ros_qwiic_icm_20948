/*
MIT License

Copyright (c) 2022 Lou Amadio

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"

#include "Arduino.h"
#include "Wire.h"

#include "ICM_20948.h"

#define AD0_VAL 1      // The value of the last bit of the I2C address.                
                       // On the SparkFun 9DoF IMU breakout the default is 1, and when 
                       // the ADR jumper is closed the value becomes 0

inline float MicroTeslaToTesla(float mT)
{
  return mT * 1000000;
}

inline float MilliGToMeterPerSecond(float g)
{
  return g * 1000 * 9.80665;
}

inline float DegreesPerSecondToRadsPerSecond(float dps)
{
  return dps * 0.01745;
}

using namespace std::chrono_literals;
using std::placeholders::_1;

class I2CPublisher : public rclcpp::Node
{
  public:
    I2CPublisher()
    : Node("ros_qwiic_icm_20948")
    , _id(0) 
    {
      get_parameter_or<uint8_t>("id", _id, 0x5D); 
      get_parameter_or<std::string>("frame_id", _frameId, "imu"); 
      get_parameter_or<std::string>("topicImu", _topicImu, "/imu/data_raw"); 
      get_parameter_or<std::string>("topicMag", _topicMag, "/imu/mag"); 
      get_parameter_or<double>("poll", _poll, 15.0);

      Wire.begin();
      Wire.setAddressSize(1); 
      Wire.setPageBytes(256);
      myICM.begin(Wire, AD0_VAL);

      _publisherImu = this->create_publisher<sensor_msgs::msg::Imu>(_topicImu, 10);
      _publisherMag = this->create_publisher<sensor_msgs::msg::MagneticField>(_topicMag, 10);
      _timer = this->create_wall_timer(
        std::chrono::duration<double, std::milli>(_poll), 
        std::bind(&I2CPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      if (myICM.dataReady())
      {
        myICM.getAGMT();         // The values are only updated when you call 'getAGMT'

        auto messageMag = sensor_msgs::msg::MagneticField();
        messageMag.header.frame_id = _frameId;
        messageMag.header.stamp = rclcpp::Clock().now();
        messageMag.magnetic_field.x = MicroTeslaToTesla(myICM.magX());
        messageMag.magnetic_field.y = MicroTeslaToTesla(myICM.magY());
        messageMag.magnetic_field.z = MicroTeslaToTesla(myICM.magZ());
        _publisherMag->publish(messageMag);

        auto messageImu = sensor_msgs::msg::Imu();
        messageImu.header.frame_id = _frameId;
        messageImu.header.stamp = messageMag.header.stamp;
        messageImu.linear_acceleration.x = MilliGToMeterPerSecond(myICM.accX());
        messageImu.linear_acceleration.y = MilliGToMeterPerSecond(myICM.accY());
        messageImu.linear_acceleration.z = MilliGToMeterPerSecond(myICM.accZ());

        messageImu.angular_velocity.x = DegreesPerSecondToRadsPerSecond(myICM.gyrX());
        messageImu.angular_velocity.y = DegreesPerSecondToRadsPerSecond(myICM.gyrY());
        messageImu.angular_velocity.z = DegreesPerSecondToRadsPerSecond(myICM.gyrZ());

        _publisherImu->publish(messageImu);
      }
    }

    ICM_20948_I2C myICM;

    rclcpp::TimerBase::SharedPtr _timer;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr _publisherImu;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr _publisherMag;
    uint8_t _id;
    double _poll;
    std::string _topicImu;
    std::string _topicMag;
    std::string _frameId;

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<I2CPublisher>();
    node->declare_parameter("i2c_address");
    node->declare_parameter("frame_id");
    node->declare_parameter("poll");
    node->declare_parameter("topicImu");
    node->declare_parameter("topicMag");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}