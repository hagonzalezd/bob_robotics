#pragma once

namespace GeNN_Robotics {
namespace Robotics {
// Forward declarations
class MotorI2C;
}
}

// Functions
float driveMotorFromCPU1(MotorI2C &motor, bool display = false);
