#include <cmath>
#include <opencv2/core/types.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_dart_guide {

class DartLauncherGuidance
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DartLauncherGuidance()
        : Node(get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {

        register_input("/dart_guide/camera/target_position", target_position_, false);
        register_input("/dart/pitch_angle/current_angle", dart_current_pitch_angle_, false);

        register_input("/dart/launch_count", dart_launch_count_, false);
        register_output("/dart_guide/pitch_angle_lock", pitch_lock_, true);
        register_output("/dart_guide/fire_command", fire_command_, false);

        register_output("/dart_guide/yaw_angle_error", yaw_angle_error_, nan);
        register_output("/dart_guide/pitch_angle_setpoint", pitch_angle_setpoint_);

        pitch_default_setpoint_  = get_parameter("default_pitch").as_double();
        guidelight_yaw_setpoint_ = get_parameter("guidelight_yaw_setpoint").as_double();
    }

    void update() override {
        if (*dart_launch_count_ == 0) {
            *pitch_angle_setpoint_ = pitch_default_setpoint_;
        }
        *yaw_angle_error_ = guidelight_yaw_setpoint_ - target_position_->x;
    }

private:
    rclcpp::Logger logger_;
    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();

    InputInterface<cv::Point2i> target_position_;
    InputInterface<double> dart_current_pitch_angle_;

    InputInterface<int> dart_launch_count_;
    OutputInterface<bool> pitch_lock_;
    OutputInterface<bool> fire_command_;

    OutputInterface<double> yaw_angle_error_;
    OutputInterface<double> pitch_angle_setpoint_;

    double pitch_default_setpoint_;
    double guidelight_yaw_setpoint_;
};
} // namespace rmcs_dart_guide

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_dart_guide::DartLauncherGuidance, rmcs_executor::Component)
