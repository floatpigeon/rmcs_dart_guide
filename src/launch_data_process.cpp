#include <cmath>
#include <eigen3/Eigen/Dense>
#include <opencv2/core/types.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_dart_guide {

class LaunchDataProcess
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    LaunchDataProcess()
        : Node(get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {
        pitch_default_setpoint_ = get_parameter("default_pitch").as_double();

        register_input("/dart/launch_count", launch_count_, 0);
        register_output("/dart_guide/pitch_angle/setpoint", pitch_angle_setpoint_);
    }

    void update() override {}

private:
    rclcpp::Logger logger_;

    OutputInterface<double> pitch_angle_setpoint_;
    InputInterface<int> launch_count_;
    double pitch_default_setpoint_;
};
} // namespace rmcs_dart_guide
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_dart_guide::LaunchDataProcess, rmcs_executor::Component)