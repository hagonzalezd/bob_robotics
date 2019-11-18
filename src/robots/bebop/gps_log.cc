// BoB robotics includes
#include "common/macros.h"
#include "robots/bebop/gps_log.h"

namespace BoBRobotics {
namespace Robots {
BebopGPSLog::BebopGPSLog(const filesystem::path &filePath)
{
    // Don't overwrite existing file
    BOB_ASSERT(!filePath.exists());

    // Try to open file
    m_ofs.open(filePath.str());
    BOB_ASSERT(m_ofs.good());

    // CSV header
    m_ofs << "Lat [deg], Lon [deg], Height [m], LatErr [m], LonErr [m], HeightErr [m], ImagePath" << std::endl;
}

void
BebopGPSLog::log(const Bebop::GPSData &gps, const filesystem::path &imagePath)
{
    m_ofs << gps.coordinate.lat.value() << ", " << gps.coordinate.lon.value()
          << ", " << gps.coordinate.height.value() << ", "
          << gps.latError.value() << ", " << gps.lonError.value() << ", "
          << gps.heightError.value() << ", " << imagePath << std::endl;
}
}
}
