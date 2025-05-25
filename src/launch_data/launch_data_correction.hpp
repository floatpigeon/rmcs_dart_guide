// 单发校准参数

#include <utility>
namespace rmcs_dart_guide {
struct DartCalibrationData {
    int name;

    double pitch_calibration_;

    double yaw_calibration_;
};

class LaunchData {
public:
    LaunchData() {}

    std::pair<double, double> get_dart_calibration_data(int dart_name) {
        int site = dart_name % 1000;

        std::pair<double, double> calibration_data(
            data_collection_[site].yaw_calibration_, data_collection_[site].pitch_calibration_);

        return calibration_data;
    }

private:
    DartCalibrationData data_collection_[2]{
        // 25000mm
        {1000,  0.0,   0.0},
        {1001, 36.8, 673.0},
    };
};

} // namespace rmcs_dart_guide