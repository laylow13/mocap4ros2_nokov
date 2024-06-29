//
// Created by lay on 24-6-29.
//
#include <iostream>
#include <memory>
#include "mocap4r2_nokov_driver/mocap4r2_nokov_driver.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<mocap4r2_nokov_driver::NokovDriverNode>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();

    return 0;
}
