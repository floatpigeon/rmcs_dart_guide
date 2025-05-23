#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

namespace rmcs_dart_guide {
class DartGuideTracker : public rclcpp::Node {
public:
    explicit DartGuideTracker(double min_contour_area = 20.0, double max_distance_threshold = 200.0)
        : Node("dart_guide_Tracker", rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger())
        , current_position_(-1, -1)
        , is_initialized_(false)
        , min_contour_area_(min_contour_area)
        , max_distance_threshold_(max_distance_threshold) {}

    void init(cv::Point2i& initial_position) {
        current_position_ = initial_position;
        is_initialized_   = true;
    }

    void update(const cv::Mat& binary_image) {
        if (!is_initialized_) {
            RCLCPP_INFO(logger_, "Tracker not initialized. Call init() first.");
            return;
        }

        if (binary_image.empty() || binary_image.channels() != 1 || binary_image.type() != CV_8U) {
            RCLCPP_INFO(logger_, "Invalid binary image format. Expected CV_8U single channel.");
            return;
        }

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(binary_image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        double min_dist           = std::numeric_limits<double>::max();
        cv::Point2f best_centroid = current_position_;
        bool found_match          = false;

        for (const auto& contour : contours) {
            double area = cv::contourArea(contour);
            if (area < min_contour_area_) {
                continue;
            }

            cv::Moments moments = cv::moments(contour);

            if (moments.m00 == 0) {
                continue;
            }

            cv::Point2f centroid(moments.m10 / moments.m00, moments.m01 / moments.m00);

            double dist = cv::norm(centroid - (cv::Point2f)current_position_);

            if (dist < min_dist) {
                min_dist      = dist;
                best_centroid = centroid;
                found_match   = true;
            }
        }

        if (found_match && min_dist < max_distance_threshold_) {
            current_position_ = cv::Point2i(std::round(best_centroid.x), std::round(best_centroid.y));
            tracking_flag_    = true;
        } else {
            RCLCPP_WARN(logger_, "lose target");
            tracking_flag_ = false;
        }
    }

    cv::Point2i get_current_position() const { return current_position_; }

    bool get_tracking_status() const { return tracking_flag_; }

private:
    rclcpp::Logger logger_;

    bool tracking_flag_;

    cv::Point2i current_position_;
    bool is_initialized_;
    double min_contour_area_;
    double max_distance_threshold_;
};

} // namespace rmcs_dart_guide