#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"

#include "Arduino.h"
#include <string.h>
#include "Wire.h"

TwoWire Wire;
TwoWire::TwoWire(std::string bus)
    : _bus(bus)
    , _readIndex(0)
{



}

void TwoWire::begin()
{
    if ((_i2cFileDescriptor = i2c_open(_bus.c_str())) == -1) 
    {
        return;
    }

    i2c_init_device(&_i2cDevice);
}


void TwoWire::beginTransmission(uint16_t id)
{
    _i2cDevice.bus = _i2cFileDescriptor;
    _i2cDevice.addr = id;
}

void TwoWire::endTransmission(bool)
{
    int ret = i2c_ioctl_write(&_i2cDevice, 0, _writeBuffer.data(), _writeBuffer.size());
    if (ret == -1 || (size_t)ret != 1)
    {
        RCLCPP_INFO(rclcpp::get_logger("i2c"), "failed to write: [%d]", ret);
    }
}

bool TwoWire::available()
{
    return _readIndex < _readBuffer.size();
}

uint8_t TwoWire::read()
{
    if (_readIndex < _readBuffer.size())
    {
        return _readBuffer[_readIndex++];
    }

    return 0;
}

void TwoWire::write(uint8_t value)
{
    _writeBuffer.push_back(value);
}

void TwoWire::write(uint8_t* value, size_t len)
{
    _writeBuffer.insert(_writeBuffer.end(), value, value + len);
}

uint32_t TwoWire::requestFrom(uint16_t id, size_t size)
{
    _i2cDevice.bus = _i2cFileDescriptor;
    _i2cDevice.addr = id;

    _readIndex = 0;

    _readBuffer.reserve(size);

    int ret = i2c_read(&_i2cDevice, 0, &_readBuffer.front(), size);
    if (ret == -1 || (size_t)ret != 1)
    {
        RCLCPP_INFO(rclcpp::get_logger("i2c"), "failed to read status: [%d]", ret);
    }

    return (size_t)ret;
}
