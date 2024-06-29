//
// Created by lay on 24-6-29.
//

#include "mocap4r2_nokov_driver/mocap4r2_nokov_driver.hpp"

using namespace std::chrono_literals;

namespace mocap4r2_nokov_driver {
    struct sFrameData {
        mocap4r2_msgs::msg::Markers markers;
        mocap4r2_msgs::msg::RigidBodies rigidBodies;
    } frameData;
    std::mutex mtx;

    void DataHandler(sFrameOfMocapData *pFrameOfData, void *pUserData) {
        if (nullptr == pFrameOfData)
            return;
        // Store the frame
        std::lock_guard<std::mutex> lck(mtx);

        int nmarker = (pFrameOfData->nOtherMarkers < MAX_MARKERS) ? pFrameOfData->nOtherMarkers : MAX_MARKERS;

        frameData.markers.frame_number = pFrameOfData->iFrame;
        for (int i = 0; i < nmarker; ++i) {
            mocap4r2_msgs::msg::Marker marker;
            marker.id_type = mocap4r2_msgs::msg::Marker::USE_INDEX;
//            marker.marker_index =; //TODO:
            marker.translation.x = pFrameOfData->OtherMarkers[i][0];
            marker.translation.y = pFrameOfData->OtherMarkers[i][1];
            marker.translation.z = pFrameOfData->OtherMarkers[i][2];
            frameData.markers.markers.push_back(marker);
        }


        frameData.rigidBodies.frame_number = pFrameOfData->iFrame;
        for (int i = 0; i < pFrameOfData->nRigidBodies; ++i) {
            mocap4r2_msgs::msg::RigidBody rigidBody;
            auto &rigidBodyData = pFrameOfData->RigidBodies[i];
            rigidBody.rigid_body_name = rigidBodyData.ID;
            rigidBody.pose.position.x = rigidBodyData.x;
            rigidBody.pose.position.y = rigidBodyData.y;
            rigidBody.pose.position.z = rigidBodyData.z;
            rigidBody.pose.orientation.w = rigidBodyData.qx;
            rigidBody.pose.orientation.x = rigidBodyData.qy;
            rigidBody.pose.orientation.y = rigidBodyData.qz;
            rigidBody.pose.orientation.z = rigidBodyData.qw;
            auto nMarkerofBody = rigidBodyData.nMarkers;
            for (size_t j = 0; j < nMarkerofBody; j++) {
                mocap4r2_msgs::msg::Marker marker;
                marker.id_type = mocap4r2_msgs::msg::Marker::USE_INDEX;
                marker.marker_index = rigidBodyData.MarkerIDs[j];
                marker.translation.x = rigidBodyData.Markers[j][0];
                marker.translation.y = rigidBodyData.Markers[j][1];
                marker.translation.z = rigidBodyData.Markers[j][2];
                rigidBody.markers.push_back(marker);
            }
            frameData.rigidBodies.rigidbodies.push_back(rigidBody);
        }
    }

    const sFrameData &GetCurrentFrame() {
        static sFrameData frame;
        {
            std::lock_guard<std::mutex> lck(mtx);
            frame = frameData;
        }
        return frame;
    }

    NokovDriverNode::NokovDriverNode()
            : ControlledLifecycleNode("mocap4r2_nokov_driver_node") {
        declare_parameter<std::string>("host_name", "192.168.10.1:801");
        declare_parameter<std::string>("frame_id", "nokov_world");
    }

// In charge of choose the different driver options related and provided by the Nokov SDK
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

    void NokovDriverNode::process_frame() {
        if (markers_pub_->get_subscription_count() == 0 &&
            rigid_bodies_pub_->get_subscription_count() == 0) {
            return;
        }
        auto frame = GetCurrentFrame();
        frame.markers.header.stamp = now();
        frame.markers.header.frame_id = frame_id_;
        frame.rigidBodies.header.stamp = now();
        frame.rigidBodies.header.frame_id = frame_id_;
        if (!frame.markers.markers.empty()) {
            markers_pub_->publish(frame.markers);
        }
        if (!frame.rigidBodies.rigidbodies.empty()) {
            rigid_bodies_pub_->publish(frame.rigidBodies);
        }
    }

    using CallbackReturnT =
            rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;


// The next Callbacks are used to manage behavior in the different states of the lifecycle node.

    CallbackReturnT
    NokovDriverNode::on_configure(const rclcpp_lifecycle::State &) {
        initParameters();

        markers_pub_ = create_publisher<mocap4r2_msgs::msg::Markers>("/markers", rclcpp::QoS(1000));
        rigid_bodies_pub_ = create_publisher<mocap4r2_msgs::msg::RigidBodies>(
                "/rigid_bodies", rclcpp::QoS(1000));

        nokovClient = std::make_unique<NokovSDKClient>();
        nokovClient->SetDataCallback(DataHandler);
        unsigned char sdkVersion[4] = {0};
        nokovClient->NokovSDKVersion(sdkVersion);
        RCLCPP_INFO(get_logger(), "SDK Ver:%c.%c.%c.%c", sdkVersion[0], sdkVersion[1], sdkVersion[2], sdkVersion[3]);
        // Check the ret value
        int retValue = nokovClient->Initialize((char *) host_name_.c_str());
        if (ErrorCode_OK != retValue) {
            RCLCPP_ERROR(get_logger(), "Error connecting XINGYING on address: %s,Code: %d", host_name_.c_str(),
                         retValue);
            RCLCPP_ERROR(get_logger(), "... not connected");
            return CallbackReturnT::FAILURE;
        } else {
            RCLCPP_INFO(get_logger(), "... connected!");
            return CallbackReturnT::SUCCESS;
        }
    }

    CallbackReturnT
    NokovDriverNode::on_activate(const rclcpp_lifecycle::State &) {
        markers_pub_->on_activate();
        rigid_bodies_pub_->on_activate();

        timer_ = create_wall_timer(10ms, std::bind(&NokovDriverNode::process_frame, this));
        return CallbackReturnT::SUCCESS;
    }

    CallbackReturnT
    NokovDriverNode::on_deactivate(const rclcpp_lifecycle::State &) {
        markers_pub_->on_deactivate();
        rigid_bodies_pub_->on_deactivate();

        //client disable data

        timer_ = nullptr;
        return CallbackReturnT::SUCCESS;
    }

    CallbackReturnT
    NokovDriverNode::on_cleanup(const rclcpp_lifecycle::State &) {
//        Disconnect
        return CallbackReturnT::SUCCESS;
    }

    CallbackReturnT
    NokovDriverNode::on_shutdown(const rclcpp_lifecycle::State &) {
        //        Disconnect
        return CallbackReturnT::SUCCESS;
    }

    CallbackReturnT
    NokovDriverNode::on_error(const rclcpp_lifecycle::State &) {
        //        Disconnect
        return CallbackReturnT::SUCCESS;
    }


// Init the necessary parameters
    void NokovDriverNode::initParameters() {
        get_parameter<std::string>("host_name", host_name_);
        get_parameter<std::string>("frame_id", frame_id_);
    }

}  // namespace mocap4r2_nokov_driver
