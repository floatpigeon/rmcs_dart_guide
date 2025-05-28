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
    DartCalibrationData data_collection_[5]{

        {0, 37.0,   0.0},
        {1, 36.8, 673.0},
        {2, 38.0,   720},
        {3, 37.5,   720},
        {4, 36.5,   720}
    };
};

} // namespace rmcs_dart_guide