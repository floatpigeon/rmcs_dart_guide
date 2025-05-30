// 单发校准参数

#include "dart_signal_parameters.hpp"
#include <utility>
namespace rmcs_dart_guide {
struct DartCalibrationData {
    int name;

    double pitch_calibration_;

    double yaw_calibration_;
};

enum class DartTarget { OUTPOST, BASE };

class LaunchData {
public:
    LaunchData() {}

    std::pair<double, double> get_dart_calibration_data(int dart_name) {
        int site = dart_name % 1000;

        std::pair<double, double> calibration_data(
            data_collection_[site].yaw_calibration_, data_collection_[site].pitch_calibration_);

        return calibration_data;
    }

    DartSignalParameter get_launch_parameter(int launch_num, DartTarget target) {
        DartSignalParameter launch_parameter;
        if (target == DartTarget::OUTPOST) {
            launch_parameter = outpost_launch_parameters[launch_num];
        } else {
            launch_parameter = base_launch_parameters[launch_num];
        }

        return launch_parameter;
    }

private:
    DartCalibrationData data_collection_[5]{
        {0, 38.0, 690.0},
        {1, 38.5, 690.0},
        {2, 38.0, 690.0},
        {3, 38.5, 690.0},
    };

    DartSignalParameter base_launch_parameters[4]{
        {0, 660, 325, 37.5, 690},
        {1, 660, 325, 37.0, 690},
        {2, 660, 325, 36.5, 690},
        {3, 660, 325, 37.0, 690},
    };

    DartSignalParameter outpost_launch_parameters[4]{
        {0, 560, 285, 36.9, 643},
        {1, 565, 285, 38.0, 647}, //+
        {2, 565, 285, 36.6, 652},
        {3, 560, 285, 36.0, 652}, // -
    };
};

} // namespace rmcs_dart_guide

/*

outpost

c1: v:560-285 p:36.50

c2: v560-285, p:36.90, y:643

b17: v565-285,p 37.5, y:647

b12: v560-285,p:36.6,y:652

a15: v560-285,p:36.5,y:652
*/