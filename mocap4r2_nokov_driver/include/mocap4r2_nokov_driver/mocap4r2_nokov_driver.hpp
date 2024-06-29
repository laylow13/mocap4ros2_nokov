//
// Created by lay on 24-6-29.
//

#ifndef BUILD_MOCAP4R2_NOKOV_DRIVER_H
#define BUILD_MOCAP4R2_NOKOV_DRIVER_H


#include <iostream>
#include <sstream>
#include <map>
#include <string>
#include <memory>
#include <chrono>
#include <vector>


#include "rclcpp/rclcpp.hpp"
#include "mocap4r2_control/ControlledLifecycleNode.hpp"

#include "mocap4r2_msgs/msg/marker.hpp"
#include "mocap4r2_msgs/msg/markers.hpp"
#include "mocap4r2_msgs/msg/rigid_bodies.hpp"

namespace mocap4r2_nokov_driver
{

    class NokovDriverNode : public mocap4r2_control::ControlledLifecycleNode
    {
    public:
        NokovDriverNode();

        using CallbackReturnT =
                rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

        CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);
        CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);
        CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);
        CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state);
        CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state);
        CallbackReturnT on_error(const rclcpp_lifecycle::State & state);
        bool connect_nokov();
        void set_settings_nokov();
        void initParameters();

    protected:
        rclcpp_lifecycle::LifecyclePublisher<mocap4r2_msgs::msg::Markers>::SharedPtr markers_pub_;
        rclcpp_lifecycle::LifecyclePublisher<mocap4r2_msgs::msg::RigidBodies>::SharedPtr
                rigid_bodies_pub_;
        rclcpp::TimerBase::SharedPtr timer_;

        std::string stream_mode_;
        std::string host_name_;
        std::string frame_id_;
        int frameCount_ {0};

        void process_frame();

        void control_start(const mocap4r2_control_msgs::msg::Control::SharedPtr msg) override;
        void control_stop(const mocap4r2_control_msgs::msg::Control::SharedPtr msg) override;
    };

}  // namespace mocap4r2_nokov_driver

#endif //BUILD_MOCAP4R2_NOKOV_DRIVER_H
