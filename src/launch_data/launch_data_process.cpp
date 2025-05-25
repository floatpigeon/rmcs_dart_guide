#include <cmath>
#include <eigen3/Eigen/Dense>
#include <opencv2/core/types.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>

namespace rmcs_dart_guide {

// TODO: 单发校准相关

class LaunchDataProcess
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    LaunchDataProcess()
        : Node(get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {
        pitch_default_setpoint_ = get_parameter("default_pitch").as_double();

        register_input("/dart/launch_count", launch_count_);
        register_output("/dart_guide/pitch_angle/setpoint", pitch_angle_setpoint_);

        //
        register_input("/remote/switch/left", input_switch_left_, false);
        register_input("/remote/switch/right", input_switch_right_, false);
        register_input("/remote/joystick/left", input_joystick_left_, false);
        register_input("/remote/joystick/right", input_joystick_right_, false);
    }

    void update() override {

        if (*input_switch_left_ == rmcs_msgs::Switch::UP && *input_switch_right_ == rmcs_msgs::Switch::DOWN) {
            pitch_default_setpoint_ = pitch_default_setpoint_ + 0.0001 * input_joystick_left_->x();
        }

        *pitch_angle_setpoint_ = pitch_default_setpoint_;
    }

private:
    rclcpp::Logger logger_;

    OutputInterface<double> pitch_angle_setpoint_;
    InputInterface<int> launch_count_;

    double pitch_default_setpoint_;

    //
    InputInterface<rmcs_msgs::Switch> input_switch_left_;
    InputInterface<rmcs_msgs::Switch> input_switch_right_;
    InputInterface<Eigen::Vector2d> input_joystick_left_;
    InputInterface<Eigen::Vector2d> input_joystick_right_;
};
} // namespace rmcs_dart_guide
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_dart_guide::LaunchDataProcess, rmcs_executor::Component)