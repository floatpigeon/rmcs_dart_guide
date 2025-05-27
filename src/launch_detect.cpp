
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
namespace rmcs_dart_guide {

class DartLaunchControl
    : public rmcs_executor::Component
    , public rclcpp::Node {

public:
    DartLaunchControl()
        : Node(get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {
        register_input("/dart/first_right_friction/velocity", first_fric_current_velocity_, false);
        register_input("/dart/second_right_friction/velocity", second_fric_current_velocity_, false);
        register_input("/dart/conveyor/velocity", conveyor_current_velocity_, false);
    }

    void update() override {
        //
        if (*first_fric_current_velocity_ > 550) {
            friction_stable_working_ = true;
        }

        if (*conveyor_current_velocity_ > 0) {
            if (friction_stable_working_ && *first_fric_current_velocity_ < 550) {
                velocity_wave_count_++;
            }

            if (velocity_wave_count_ == 5) {
                friction_stable_working_ = false;
                launch_count_++;
            }
        }
    }

private:
    rclcpp::Logger logger_;

    InputInterface<double> conveyor_current_velocity_;
    InputInterface<double> first_fric_current_velocity_;
    InputInterface<double> second_fric_current_velocity_;

    int velocity_wave_count_ = 0;
    int launch_count_        = 0;

    bool friction_stable_working_;
};
} // namespace rmcs_dart_guide