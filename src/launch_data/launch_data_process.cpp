#include "launch_data_correction.hpp"
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <opencv2/core/types.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/robot_id.hpp>
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
        register_input("/dart/first_right_friction/velocity", dart_friction_velocity_);

        //
        register_input("/remote/switch/left", input_switch_left_, false);
        register_input("/remote/switch/right", input_switch_right_, false);
        register_input("/remote/joystick/left", input_joystick_left_, false);
        register_input("/remote/joystick/right", input_joystick_right_, false);

        register_input("/referee/id", robot_id_);
        register_input("/referee/outpost_hp/red", red_outpost_hp_);
        register_input("/referee/outpost_hp/blue", blue_outpost_hp_);
    }

    void update() override {

        // *pitch_angle_setpoint_ = launch_data_collection_.get_dart_calibration_data(*launch_count_ % 4).second;

        // RCLCPP_INFO(logger_, "launch_count:%d", *launch_count_);

        yaw_auto_aim();
        // debug();
        // correction_debug();
    }

private:
    void yaw_auto_aim() {
        if (*robot_id_ == rmcs_msgs::RobotId::RED_DART) {
            enemy_outpost_hp_ = *blue_outpost_hp_;
        } else if (*robot_id_ == rmcs_msgs::RobotId::BLUE_DART) {
            enemy_outpost_hp_ = *red_outpost_hp_;
        }

        if (enemy_outpost_hp_ > 0) {
            *pitch_angle_setpoint_ =
                launch_data_collection_.get_launch_parameter(*launch_count_ % 4, DartTarget::OUTPOST).pitch_setpoint;
        } else {
            *pitch_angle_setpoint_ =
                launch_data_collection_.get_launch_parameter(*launch_count_ % 4, DartTarget::BASE).pitch_setpoint;
        }
    }

    void debug() {
        if (*input_switch_left_ == rmcs_msgs::Switch::UP && *input_switch_right_ == rmcs_msgs::Switch::DOWN) {
            if (input_joystick_left_->x() != 0)
                pitch_default_setpoint_ = pitch_default_setpoint_
                                        + ((abs(input_joystick_left_->x()) > 0.5) ? 0.001 : 0.0001)
                                              * ((input_joystick_left_->x() > 0) ? 1 : -1);
        }
        *pitch_angle_setpoint_ = pitch_default_setpoint_;
    }

    void correction_debug() {
        *pitch_angle_setpoint_ =
            launch_data_collection_.get_launch_parameter(*launch_count_ % 4, DartTarget::DEBUG).pitch_setpoint;
    }

    rclcpp::Logger logger_;

    OutputInterface<double> pitch_angle_setpoint_;
    InputInterface<int> launch_count_;

    InputInterface<double> dart_friction_velocity_;

    double pitch_default_setpoint_;

    //
    InputInterface<rmcs_msgs::Switch> input_switch_left_;
    InputInterface<rmcs_msgs::Switch> input_switch_right_;
    InputInterface<Eigen::Vector2d> input_joystick_left_;
    InputInterface<Eigen::Vector2d> input_joystick_right_;

    InputInterface<rmcs_msgs::RobotId> robot_id_;
    InputInterface<uint16_t> red_outpost_hp_;
    InputInterface<uint16_t> blue_outpost_hp_;
    double enemy_outpost_hp_;

    LaunchData launch_data_collection_;
};
} // namespace rmcs_dart_guide
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_dart_guide::LaunchDataProcess, rmcs_executor::Component)