#pragma once

namespace rmcs_dart_guide {

struct DartSignalParameter {
    int name;
    double first_fric_velocity;
    double seconnd_fric_velocity;
    double pitch_setpoint;
    double yaw_setpoint;
};
} // namespace rmcs_dart_guide