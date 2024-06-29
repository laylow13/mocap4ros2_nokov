//
// Created by lay on 24-6-29.
//

#include "mocap4r2_nokov_driver/mocap4r2_nokov_driver.hpp"

using namespace std::chrono_literals;

namespace mocap4r2_nokov_driver {

// The vicon driver node has differents parameters
// to initialized with the mocap4r2_vicon_driver_params.yaml
    NokovDriverNode::NokovDriverNode()
            : ControlledLifecycleNode("mocap4r2_vicon_driver_node") {
        declare_parameter<std::string>("stream_mode", "ClientPull");
        declare_parameter<std::string>("host_name", "192.168.10.1:801");
        declare_parameter<std::string>("frame_id", "vicon_world");
    }

// In charge of choose the different driver options related and provided by the Vicon SDK
    void NokovDriverNode::set_settings_nokov() {
    }

// In charge of the transition of the lifecycle node
    void NokovDriverNode::control_start(const mocap4r2_control_msgs::msg::Control::SharedPtr msg) {
        (void) msg;
    }

// In charge of the transition of the lifecycle node
    void NokovDriverNode::control_stop(const mocap4r2_control_msgs::msg::Control::SharedPtr msg) {
        (void) msg;
    }

// In charge of get the Vicon information and convert it to vicon_msgs
    void NokovDriverNode::process_frame() {

    }

    using CallbackReturnT =
            rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;


// The next Callbacks are used to manage behavior in the different states of the lifecycle node.

    CallbackReturnT
    NokovDriverNode::on_configure(const rclcpp_lifecycle::State &) {

    }

    CallbackReturnT
    NokovDriverNode::on_activate(const rclcpp_lifecycle::State &) {

    }

    CallbackReturnT
    NokovDriverNode::on_deactivate(const rclcpp_lifecycle::State &) {

    }

    CallbackReturnT
    NokovDriverNode::on_cleanup(const rclcpp_lifecycle::State &) {
    }

    CallbackReturnT
    NokovDriverNode::on_shutdown(const rclcpp_lifecycle::State &) {
    }

    CallbackReturnT
    NokovDriverNode::on_error(const rclcpp_lifecycle::State &) {
    }


// Init the necessary parameters
    void NokovDriverNode::initParameters() {
    }

}  // namespace mocap4r2_nokov_driver
