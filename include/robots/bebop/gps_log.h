#pragma once

// BoB robotics includes
#include "common/stopwatch.h"
#include "bebop.h"

// Third-party includes
#include "third_party/path.h"

// Standard C++ includes
#include <fstream>

namespace BoBRobotics {
namespace Robots {
class BebopGPSLog {
public:
    BebopGPSLog(const filesystem::path &filePath);
    void log(const Bebop::GPSData &gps, const filesystem::path &imagePath = "");

private:
    std::ofstream m_ofs;
    Stopwatch m_Stopwatch;
}; // BebopGPSLog
} // Robots
} // BoBRobotics
