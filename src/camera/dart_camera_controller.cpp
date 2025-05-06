#include <hikcamera/image_capturer.hpp>
#include <memory>
#include <mutex>
#include <opencv2/core/mat.hpp>
#include <rclcpp/logger.hpp>
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

        register_output("/dart_guide/camera_image", camera_image_);
        camera_thread_ = std::thread(&DartCameraController::image_capture, this);
    }

    void update() override {}

private:
    void image_capture() {
        while (true) {

            cv::Mat read   = image_capture_->read();
            *camera_image_ = read;
        }
    }
    rclcpp::Logger logger_;

    std::thread camera_thread_;
    std::mutex camera_thread_mtx_;
    std::atomic<bool> camera_enable_flag_ = false;

    hikcamera::ImageCapturer::CameraProfile camera_profile_;
    std::unique_ptr<hikcamera::ImageCapturer> image_capture_;

    OutputInterface<cv::Mat> camera_image_;
};
} // namespace rmcs_dart_guide

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_dart_guide::DartCameraController, rmcs_executor::Component)