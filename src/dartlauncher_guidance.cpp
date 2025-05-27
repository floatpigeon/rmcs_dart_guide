#include <cmath>
#include <eigen3/Eigen/Dense>
#include <opencv2/core/types.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/game_stage.hpp>
#include <rmcs_msgs/switch.hpp>

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
        register_output("/dart_guide/stop_all", stop_all_, false);

        //
        register_input("/remote/switch/left", input_switch_left_, false);
        register_input("/remote/switch/right", input_switch_right_, false);
        register_input("/remote/joystick/left", input_joystick_left_, false);
        register_input("/remote/joystick/right", input_joystick_right_, false);

        register_input("/dart/conveyor/velocity", conveyor_current_velocity_, false);

        // register_input("/referee/game/stage", game_stage_);
    }

    void update() override {
        if (*input_switch_left_ == rmcs_msgs::Switch::UP && *input_switch_right_ == rmcs_msgs::Switch::DOWN) {
            dart_guide_enable_ = false;
            *yaw_angle_error_  = 30 * input_joystick_right_->y();
        } else {
            dart_guide_enable_ = true;
        }
        if (dart_guide_enable_) {
            if (target_position_->x == -1 || target_position_->y == -1) {
                *yaw_angle_error_ = nan;
            } else {
                *yaw_angle_error_ = guidelight_yaw_setpoint_ - target_position_->x;
            }
        }

        // RCLCPP_INFO(logger_, "error:%lf,,current:(%d,%d)", *yaw_angle_error_, target_position_->x,
        // target_position_->y);
        guide_ready_judge();
    }

private:
    void guide_ready_judge() {
        // TODO:
        if (*conveyor_current_velocity_ < -60) {
            *guide_ready_ = false;
        }

        if (*input_switch_left_ == rmcs_msgs::Switch::UP && *input_switch_right_ == rmcs_msgs::Switch::UP) {
            *guide_ready_ = true;
        }

        if (*input_switch_left_ == rmcs_msgs::Switch::DOWN && *input_switch_right_ == rmcs_msgs::Switch::DOWN) {
            *stop_all_    = true;
            *guide_ready_ = false;
        } else {
            *stop_all_ = false;
        }
    }

    void update_guide_status() {}

    rclcpp::Logger logger_;
    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();

    InputInterface<cv::Point2i> target_position_;
    OutputInterface<double> yaw_angle_error_;
    OutputInterface<bool> guide_ready_;
    OutputInterface<bool> stop_all_;

    InputInterface<rmcs_msgs::Switch> input_switch_left_;
    InputInterface<rmcs_msgs::Switch> input_switch_right_;
    InputInterface<Eigen::Vector2d> input_joystick_left_;
    InputInterface<Eigen::Vector2d> input_joystick_right_;

    InputInterface<rmcs_msgs::GameStage> game_stage_;
    InputInterface<double> conveyor_current_velocity_;
    bool dart_guide_enable_;

    double guidelight_yaw_setpoint_;
};
} // namespace rmcs_dart_guide

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_dart_guide::DartLauncherGuidance, rmcs_executor::Component)
