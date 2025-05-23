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

        guidelight_yaw_setpoint_ = get_parameter("guidelight_yaw_setpoint").as_double();

        register_input("/dart_guide/camera/target_position", target_position_, false);
        register_output("/dart_guide/yaw_angle_error", yaw_angle_error_, nan);
        register_output("/dart_guide/guide_ready", guide_ready_, false);
    }

    void update() override {
        *yaw_angle_error_ = guidelight_yaw_setpoint_ - target_position_->x;
        guide_ready_judge();
    }

private:
    void guide_ready_judge() {
        // TODO:
    }

    rclcpp::Logger logger_;
    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();

    InputInterface<cv::Point2i> target_position_;
    OutputInterface<double> yaw_angle_error_;
    OutputInterface<bool> guide_ready_;

    double guidelight_yaw_setpoint_;
};
} // namespace rmcs_dart_guide

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_dart_guide::DartLauncherGuidance, rmcs_executor::Component)
