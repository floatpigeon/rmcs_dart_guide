#include "camera/identifier.hpp"
#include <hikcamera/image_capturer.hpp>
#include <memory>
#include <mutex>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rmcs_executor/component.hpp>
#include <thread>

namespace rmcs_dart_guide {

class DartCameraController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DartCameraController()
        : Node(get_component_name(), rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {
        camera_profile_.invert_image  = get_parameter("invert_image").as_bool();
        camera_profile_.exposure_time = std::chrono::microseconds(get_parameter("exposure_time").as_int());
        camera_profile_.gain          = static_cast<float>(get_parameter("gain").as_double());
        image_capture_                = std::make_unique<hikcamera::ImageCapturer>(camera_profile_);

        identifdier_.set_default_limit(
            cv::Scalar(
                get_parameter("L_H").as_double(), get_parameter("L_S").as_double(),
                get_parameter("L_V").as_double()),
            cv::Scalar(
                get_parameter("U_H").as_double(), get_parameter("U_S").as_double(),
                get_parameter("U_V").as_double()));
        identifdier_.Init();

        register_output("/dart_guide/camera_image", camera_image_);
        register_output("/dart_guide/display_image", display_image_);
        camera_thread_ = std::thread(&DartCameraController::camera_update, this);
    }

    void update() override {}

private:
    void camera_update() {
        while (true) {
            cv::Mat read   = image_capture_->read();
            *camera_image_ = read;
            identifdier_.update(read);
            *display_image_ = identifdier_.get_display_image();

            if (!identifdier_.result_status_()) {
                continue;
            } else {
                cv::Point2d initial_position_ = identifdier_.get_result();
                RCLCPP_INFO(
                    logger_, "target initial position:(%lf,%lf)", initial_position_.x, initial_position_.y);
            }
        }
    }
    rclcpp::Logger logger_;

    std::thread camera_thread_;
    std::mutex camera_thread_mtx_;
    std::atomic<bool> camera_enable_flag_ = false;

    hikcamera::ImageCapturer::CameraProfile camera_profile_;
    std::unique_ptr<hikcamera::ImageCapturer> image_capture_;

    OutputInterface<cv::Mat> camera_image_;
    OutputInterface<cv::Mat> display_image_;

    Identifier identifdier_;
};
} // namespace rmcs_dart_guide

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_dart_guide::DartCameraController, rmcs_executor::Component)