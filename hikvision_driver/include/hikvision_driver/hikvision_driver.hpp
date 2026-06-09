#pragma once
#ifndef HIKVISION_DRIVER_HPP  
#define HIKVISION_DRIVER_HPP  

#include <rclcpp/rclcpp.hpp>

namespace hikvision_driver {  

class HikvisionDriver : public rclcpp::Node {
   public:
    HikvisionDriver(const rclcpp::NodeOptions &options);
    ~HikvisionDriver();

   private:
    struct Impl;
    std::unique_ptr<Impl> pImpl;
};

}  

#endif  