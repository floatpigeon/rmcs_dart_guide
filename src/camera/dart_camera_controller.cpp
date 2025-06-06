#include "camera/identifier.hpp"
#include "camera/tracker.hpp"
#include <chrono>
#include <hikcamera/image_capturer.hpp>
#include <memory>
#include <mutex>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
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

        lower_limit_default_ = cv::Scalar(
            get_parameter("L_H").as_double(), get_parameter("L_S").as_double(), get_parameter("L_V").as_double());
        upper_limit_default_ = cv::Scalar(
            get_parameter("U_H").as_double(), get_parameter("U_S").as_double(), get_parameter("U_V").as_double());

        identifdier_.set_default_limit(lower_limit_default_, upper_limit_default_);
        identifdier_.Init();

        register_output("/dart_guide/camera/camera_image", camera_image_);
        register_output("/dart_guide/camera/display_image", display_image_);
        register_output("/dart_guide/camera/target_position", target_position_, PointT(-1, -1));
        camera_thread_     = std::thread(&DartCameraController::camera_update, this);
        update_time_point_ = std::chrono::steady_clock::now();
    }

    void update() override {
        // auto current = std::chrono::steady_clock::now();
        // auto delta   = std::chrono::duration_cast<std::chrono::microseconds>(current - update_time_point_);
        // RCLCPP_INFO(logger_, "fps:%ld", 1000000 / delta.count());
        // update_time_point_ = current;
    }
    std::chrono::steady_clock::time_point update_time_point_;

private:
    void camera_update() {
        while (true) {
            cv::Mat read = image_capture_->read();
            cv::line(read, cv::Point(645, 0), cv::Point(645, 720), cv::Scalar(255, 0, 255), 1);
            *camera_image_ = read;

            cv::Mat preprocessed_image;
            image_to_binary(read, preprocessed_image);

            cv::Mat display = preprocessed_image.clone();
            if (!is_tracker_stage_) {
                identifdier_.update(preprocessed_image);

                cv::putText(
                    display, "Identifing", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 255), 2);

                if (!identifdier_.result_status_()) {
                    *display_image_ = display;
                    continue;
                } else {
                    cv::Point2i initial_position_ = identifdier_.get_result();
                    // RCLCPP_INFO(logger_, "target initial position:(%d,%d)", initial_position_.x,
                    // initial_position_.y);

                    is_tracker_stage_ = true;
                    tracker_.init(initial_position_);
                }
                *target_position_ = PointT(-1, -1);
            } else {
                // tracker enable
                tracker_.update(preprocessed_image);
                cv::Point2i current_position = tracker_.get_current_position();
                RCLCPP_INFO(logger_, "current:(%d,%d)", current_position.x, current_position.y);

                cv::putText(
                    display, tracker_.get_tracking_status() ? "Tracking" : "Loss", cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 255), 2);

                if (!tracker_.get_tracking_status()) {
                    tracker_loss_count_++;
                    *target_position_ = PointT(-1, -1);
                } else {
                    tracker_loss_count_ = 0;
                    *target_position_   = current_position;
                    cv::circle(display, current_position, 20, cv::Scalar(255, 0, 255), 2);
                }

                if (tracker_loss_count_ > 100) {
                    is_tracker_stage_ = false;
                    identifdier_.Init();
                }
            }

            cv::line(display, cv::Point(0, 645), cv::Point(720, 645), cv::Scalar(255, 0, 255), 1);

            *display_image_ = display;
        }
    }

    void image_to_binary(const cv::Mat& src, cv::Mat& output) {
        cv::Mat HSV_image;
        cv::cvtColor(src, HSV_image, cv::COLOR_BGR2HSV);

        cv::Mat binary;
        cv::inRange(HSV_image, lower_limit_default_, upper_limit_default_, binary);

        static cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(9, 9));
        cv::morphologyEx(binary, binary, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, kernel);
        output = binary;
    }

    rclcpp::Logger logger_;

    std::thread camera_thread_;
    std::mutex camera_thread_mtx_;
    std::atomic<bool> camera_enable_flag_ = false;

    cv::Scalar lower_limit_default_, upper_limit_default_;

    hikcamera::ImageCapturer::CameraProfile camera_profile_;
    std::unique_ptr<hikcamera::ImageCapturer> image_capture_;

    OutputInterface<cv::Mat> camera_image_;
    OutputInterface<cv::Mat> display_image_;

    OutputInterface<cv::Point2i> target_position_;

    DartGuideIdentifier identifdier_;
    DartGuideTracker tracker_;
    bool is_tracker_stage_  = false;
    int tracker_loss_count_ = 0;
};
} // namespace rmcs_dart_guide

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_dart_guide::DartCameraController, rmcs_executor::Component)