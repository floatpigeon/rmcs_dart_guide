// 单发校准参数

#include "dart_signal_parameters.hpp"
#include <utility>
namespace rmcs_dart_guide {
struct DartCalibrationData {
    int name;

    double pitch_calibration_;

    double yaw_calibration_;
};

enum class DartTarget { OUTPOST, BASE, DEBUG };

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
        // } else if (target == DartTarget::DEBUG) {
        //     launch_parameter = debug_parameters[launch_num];
        // }

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
        {0, 665, 340, 38.3, 710},
        {1, 665, 340, 37.5, 710}, // +
        {2, 665, 340, 38.3, 708},
        {3, 665, 340, 37.5, 708}, // -
    };

    DartSignalParameter outpost_launch_parameters[4]{
        {0, 565, 290, 38.0, 647}, // N5,右
        {1, 565, 290, 38.5, 652}, // N9，中
        {2, 560, 290, 38.0, 652}, // C13，中
        {3, 560, 290, 37.5, 643}, // C14，左
    };

    DartSignalParameter debug_parameters[6]{
        {1, 565, 290, 38.0, 647}, // N5,右
        {2, 565, 290, 38.5, 652}, // N9，中
        {3, 560, 290, 38.0, 652}, // C13，中
        {0, 560, 290, 37.5, 643}, // C14，左
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

/*
base:

b13: v665-332,p38.0,

b18: v665-332,p38.0,

a15: v665-335,p37.4,
*/