
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
namespace rmcs_dart_guide {

class PointCloudMessagePublisher
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    PointCloudMessagePublisher();

    void update() override {}

private:
    rclcpp::Logger logger_;
};
} // namespace rmcs_dart_guide