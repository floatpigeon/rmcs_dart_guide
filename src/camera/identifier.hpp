#pragma once

#include <chrono>
#include <cstddef>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <vector>
namespace rmcs_dart_guide {

using PointT = cv::Point2d;

struct TargetData {
    PointT first_position;
    PointT latest_position;
    double max_move_dist;
    int catch_count;
    int miss_count;
};

class Identifier : rclcpp::Node {
public:
    Identifier()
        : Node(
              "dart_guide_Identifier", rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {
        Init();
    }

    void Init() {
        detect_frame_count_ = 0;
        result_ready_       = false;
        // RCLCPP_INFO(logger_, "Identifier Init Compelete!");
    }

    void set_default_limit(const cv::Scalar& lower, const cv::Scalar& upper) {
        lower_limit_ = lower;
        upper_limit_ = upper;
    }

    std::chrono::steady_clock::time_point begin_time_;
    void update(const cv::Mat& image) {
        if (result_ready_) { // debug
            Init();
            begin_time_ = std::chrono::steady_clock::now();
        }
        //

        std::vector<PointT> possible_targets = preprocess(image);
        // RCLCPP_INFO(logger_, "preprocessed num:%zu", possible_targets.size());
        target_filter(possible_targets);

        detect_frame_count_++;

        if (detect_frame_count_ < 100) {
            return;
        }

        double highest_score           = 0;
        size_t most_possible_target_id = -1;

        // scan the data in collection
        if (possible_targets_collection_.empty()) {
            RCLCPP_INFO(logger_, "detected nothing");
            Init();
            // TODO:什么都没有检测到的处理方法
            // 自适应收张范围
        } else {
            for (size_t i = 0; i < possible_targets_collection_.size(); i++) {
                double this_score = possible_targets_collection_[i].catch_count
                                  + 1000 / (possible_targets_collection_[i].max_move_dist + 1);

                if (this_score > highest_score) {
                    most_possible_target_id = i;
                    highest_score           = this_score;
                }
            }
            target_initial_position_ = possible_targets_collection_[most_possible_target_id].latest_position;
            result_ready_            = true;
            auto end_time_           = std::chrono::steady_clock::now();
            auto delta_time =
                std::chrono::duration_cast<std::chrono::milliseconds>(end_time_ - begin_time_).count();
            RCLCPP_INFO(logger_, "delta_time:%ldms", delta_time);
        }
    }

    cv::Mat get_display_image() { return display_image_; }
    bool result_status_() const { return result_ready_; }
    PointT get_result() { return target_initial_position_; }

private:
    rclcpp::Logger logger_;

    cv::Mat display_image_;
    cv::Scalar lower_limit_, upper_limit_;
    cv::Scalar latest_lower_limit_, latest_upper_limit_;
    std::vector<TargetData> possible_targets_collection_;
    int detect_frame_count_ = 0;
    PointT target_initial_position_;
    bool result_ready_ = false;

    // Image Procss Methods
    std::vector<PointT> image_preprocess(const cv::Mat& src) {
        cv::Mat HSV_image;
        cv::cvtColor(src, HSV_image, cv::COLOR_BGR2HSV);

        cv::Mat binary;
        cv::inRange(HSV_image, lower_limit_, upper_limit_, binary);

        static cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(9, 9));
        cv::morphologyEx(binary, binary, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, kernel);
        display_image_ = binary;

        std::vector<cv::Vec3f> circles;
        cv::HoughCircles(binary, circles, cv::HOUGH_GRADIENT, 1.0, binary.rows / 10.0, 50, 30, 3, 100);

        std::vector<PointT> possible_targets;
        for (const auto& circle : circles) {
            PointT center(cvRound(circle[0]), cvRound(circle[1]));
            int radius = cvRound(circle[2]);
            cv::circle(display_image_, center, radius, cv::Scalar(255, 0, 255), 3);
            possible_targets.emplace_back(center);
        }

        return possible_targets;
    }

    std::vector<PointT> preprocess(const cv::Mat& src) {
        cv::Mat HSV_image;
        cv::cvtColor(src, HSV_image, cv::COLOR_BGR2HSV);

        cv::Mat binary;
        cv::inRange(HSV_image, lower_limit_, upper_limit_, binary);

        static cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(9, 9));
        cv::morphologyEx(binary, binary, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, kernel);
        display_image_ = binary;

        // 轮廓检测
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(binary, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        std::vector<PointT> possible_targets;
        for (const auto& contour : contours) {
            double area = cv::contourArea(contour);

            if (area < 100 || area > 5000)
                continue;
            if (contour.size() < 5)
                continue;

            cv::Point2f circle_center;
            float circle_radius;
            cv::minEnclosingCircle(contour, circle_center, circle_radius);
            double enclosing_circle_area = CV_PI * circle_radius * circle_radius;
            double area_ratio            = 0;
            if (enclosing_circle_area > 0) {
                area_ratio = area / enclosing_circle_area;
            }

            // 圆形度阈值，轮廓面积应接近其最小外接圆的面积
            // 对于一个完美的实心圆，这个比率接近1。考虑到像素化等因素，可以设置如0.75-0.8以上。
            if (area_ratio > 0.7) {
                possible_targets.emplace_back(circle_center);
            }
        }

        return possible_targets;
    }

    void target_filter(const std::vector<PointT>& points) {
        const int distance_threshold = 10;
        std::vector<bool> matched(points.size(), false);

        // target match
        for (auto& collected : possible_targets_collection_) {
            double min_dist = std::numeric_limits<double>::max();
            int point_id    = -1;

            for (size_t i = 0; i < points.size(); i++) {
                if (matched[i])
                    continue;

                const PointT& current_pt(points[i]);
                double dist = cv::norm(current_pt - collected.latest_position);

                if (dist < distance_threshold && dist < min_dist) {
                    min_dist = dist;
                    point_id = static_cast<int>(i);
                }
            }

            if (point_id != -1) {
                double this_move_dist     = cv::norm(collected.first_position - points[point_id]);
                collected.max_move_dist   = std::max(collected.max_move_dist, this_move_dist);
                collected.latest_position = points[point_id];
                collected.catch_count++;
                collected.miss_count = 0;
                matched[point_id]    = true;
            } else {
                collected.miss_count++;
            }
        }

        //  if target no found in collection ,add as new target
        for (size_t i = 0; i < points.size(); i++) {
            if (!matched[i]) {
                TargetData new_target_data(points[i], points[i], 0, 1, 0);
                possible_targets_collection_.emplace_back(new_target_data);
            }
        }

        // remove target witch missed so long
        possible_targets_collection_.erase(
            std::remove_if(
                possible_targets_collection_.begin(), possible_targets_collection_.end(),
                [](const TargetData& target) { return (target.miss_count > 20); }),
            possible_targets_collection_.end());
    }
};
} // namespace rmcs_dart_guide