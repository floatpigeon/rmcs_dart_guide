#include "launch_data/launch_data_correction.hpp"
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <eigen3/Eigen/Dense>
#include <opencv2/core/types.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/game_stage.hpp>
#include <rmcs_msgs/robot_id.hpp>
#include <rmcs_msgs/robots_hp.hpp>
#include <rmcs_msgs/switch.hpp>

namespace rmcs_dart_guide {

class DartLauncherGuidance
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DartLauncherGuidance()
        : Node(get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {

        // guidelight_yaw_setpoint_ = get_parameter("guidelight_yaw_setpoint").as_double();

        base_first_fric_control_velocity_     = get_parameter("base_first_fric_control_velocity").as_double();
        base_second_fric_control_velocity_    = get_parameter("base_second_fric_control_velocity").as_double();
        outpost_first_fric_control_velocity_  = get_parameter("outpost_first_fric_control_velocity").as_double();
        outpost_second_fric_control_velocity_ = get_parameter("outpost_second_fric_control_velocity").as_double();

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

        register_input("/referee/game/stage", game_stage_);
        register_input("/referee/id", robot_id_);
        register_input("/referee/dart/remaining_time", dart_remaining_time_);
        register_input("/referee/outpost_hp/red", red_outpost_hp_);
        register_input("/referee/outpost_hp/blue", blue_outpost_hp_);

        register_output("/dart_guide/first_fric_velocity_setpoint", first_fric_velocity_setpoint_, nan);
        register_output("/dart_guide/second_fric_velocity_setpoint", second_fric_velocity_setpoint_, nan);
    }

    void update() override {

        // guidelight_yaw_setpoint_ = launch_data_collection_.get_dart_calibration_data(launch_count_ % 4).first;

        if (*robot_id_ == rmcs_msgs::RobotId::RED_DART) {
            enemy_outpost_hp_ = *blue_outpost_hp_;
        } else if (*robot_id_ == rmcs_msgs::RobotId::BLUE_DART) {
            enemy_outpost_hp_ = *red_outpost_hp_;
        }

        if (enemy_outpost_hp_ > 0) {
            *first_fric_velocity_setpoint_ =
                launch_data_collection_.get_launch_parameter(launch_count_ % 4, DartTarget::OUTPOST)
                    .first_fric_velocity;
            *second_fric_velocity_setpoint_ =
                launch_data_collection_.get_launch_parameter(launch_count_ % 4, DartTarget::OUTPOST)
                    .seconnd_fric_velocity;
            guidelight_yaw_setpoint_ =
                launch_data_collection_.get_launch_parameter(launch_count_ % 4, DartTarget::OUTPOST).yaw_setpoint;

        } else {
            *first_fric_velocity_setpoint_ =
                launch_data_collection_.get_launch_parameter(launch_count_ % 4, DartTarget::BASE).first_fric_velocity;
            *second_fric_velocity_setpoint_ =
                launch_data_collection_.get_launch_parameter(launch_count_ % 4, DartTarget::BASE).seconnd_fric_velocity;

            guidelight_yaw_setpoint_ =
                launch_data_collection_.get_launch_parameter(launch_count_ % 4, DartTarget::BASE).yaw_setpoint;
        }

        // manual_control();
        // guide_ready_judge();

        auto_control();
        guide_ready_judge_auto();
        // launch_velocity_select();

        //
        // RCLCPP_INFO(logger_, "guide:%d,ready:%d", dart_guide_enable_ ? 1 : 0, *guide_ready_ ? 1 : 0);
        RCLCPP_INFO(logger_, "fric_v:%lf,hp:%f", *first_fric_velocity_setpoint_, enemy_outpost_hp_);
    }

private:
    void manual_control() {
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
    }

    void guide_ready_judge() {
        // TODO:
        if (*conveyor_current_velocity_ > 50) {
            count_lock_ = false;
        }
        if (*conveyor_current_velocity_ < -60) {
            if (*guide_ready_ && !count_lock_) {
                launch_count_++;
                count_lock_ = true;
                RCLCPP_INFO(logger_, "launch_count:%d", launch_count_);
            }
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

    void auto_control() {

        if (*conveyor_current_velocity_ > 50) {
            count_lock_ = false;
        }
        if (*conveyor_current_velocity_ < -60) {
            if (*guide_ready_ && !count_lock_) {
                launch_count_++;
                count_lock_ = true;
                RCLCPP_INFO(logger_, "launch_count:%d", launch_count_);
                guide_ready_judge_count_ = 0;
            }
            *guide_ready_ = false;
        }

        //  // 场下
        // if (*input_switch_left_ == rmcs_msgs::Switch::UP) {
        //     dart_guide_enable_ = true;
        //     if (*input_switch_right_ == rmcs_msgs::Switch::UP) {
        //         enemy_outpost_hp_ = 0;
        //     } else if (*input_switch_right_ == rmcs_msgs::Switch::MIDDLE) {
        //         enemy_outpost_hp_ = 500;
        //     }
        // } else {
        //     dart_guide_enable_ = false;
        // };

        // // 比赛
        if (*game_stage_ == rmcs_msgs::GameStage::STARTED && *dart_remaining_time_ > 5) {
            dart_guide_enable_ = true;
        } else {
            dart_guide_enable_ = false;
        }

        if (dart_guide_enable_) {
            if (target_position_->x == -1 || target_position_->y == -1) {
                *yaw_angle_error_ = nan;
            } else {
                *yaw_angle_error_ = guidelight_yaw_setpoint_ - target_position_->x;
            }
        } else {
            guide_ready_judge_count_ = 0;
        }
    }

    void guide_ready_judge_auto() {
        if (dart_guide_enable_ && abs(target_position_->x - guidelight_yaw_setpoint_) < 5) {
            guide_ready_judge_count_++;
            if (guide_ready_judge_count_ == 500) {
                *guide_ready_            = true;
                guide_ready_judge_count_ = 0;
            }
        }
    }

    void launch_velocity_select() {
        if (*robot_id_ == rmcs_msgs::RobotId::RED_DART) {
            enemy_outpost_hp_ = *blue_outpost_hp_;
        } else if (*robot_id_ == rmcs_msgs::RobotId::BLUE_DART) {
            enemy_outpost_hp_ = *red_outpost_hp_;
        }

        if (enemy_outpost_hp_ > 0) {
            *first_fric_velocity_setpoint_  = outpost_first_fric_control_velocity_;
            *second_fric_velocity_setpoint_ = outpost_second_fric_control_velocity_;
        } else {
            *first_fric_velocity_setpoint_  = base_first_fric_control_velocity_;
            *second_fric_velocity_setpoint_ = base_second_fric_control_velocity_;
        }
    }

    int guide_ready_judge_count_;
    cv::Point2i last_target_point_;

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
    InputInterface<rmcs_msgs::RobotId> robot_id_;
    InputInterface<uint16_t> red_outpost_hp_;
    InputInterface<uint16_t> blue_outpost_hp_;

    InputInterface<double> conveyor_current_velocity_;
    InputInterface<uint8_t> dart_remaining_time_;
    bool dart_guide_enable_;

    double guidelight_yaw_setpoint_;
    int launch_count_ = 0;
    bool count_lock_  = true;

    LaunchData launch_data_collection_;
    double enemy_outpost_hp_;

    double base_first_fric_control_velocity_, base_second_fric_control_velocity_;
    double outpost_first_fric_control_velocity_, outpost_second_fric_control_velocity_;
    OutputInterface<double> first_fric_velocity_setpoint_, second_fric_velocity_setpoint_;
};
} // namespace rmcs_dart_guide

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_dart_guide::DartLauncherGuidance, rmcs_executor::Component)
