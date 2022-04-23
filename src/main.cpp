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
#include <boost/asio.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/twist.hpp>

#ifdef _WIN32
#pragma optimize( "", off )
#else
#include "Wire.h"

#pragma GCC optimize ("O0")
#endif

using namespace std::chrono_literals;
using std::placeholders::_1;

class I2CPublisher : public rclcpp::Node
{
  public:
    I2CPublisher()
    : Node("i2cpublisher")
    , _id(0) 
    {
      get_parameter_or<uint8_t>("id", _id, 0x5D); 

      //_publisher = this->create_publisher<std_msgs::msg::String>("topic", 10);
      _timer = this->create_wall_timer(500ms, std::bind(&I2CPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      //auto message = std_msgs::msg::String();
      //_publisher->publish(message);
    }
    rclcpp::TimerBase::SharedPtr _timer;
    //rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _publisher;
    uint8_t _id;

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<I2CPublisher>();
    node->declare_parameter("i2c_address");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}