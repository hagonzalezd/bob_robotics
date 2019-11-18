// BoB robotics includes
#include "common/logging.h"
#include "common/macros.h"
#include "common/main.h"
#include "common/path.h"
#include "hid/joystick.h"
#include "robots/bebop/bebop.h"
#include "robots/bebop/gps_log.h"

// OpenCV
#include <opencv2/opencv.hpp>

// Standard C++ includes
#include <atomic>
#include <memory>
#include <mutex>
#include <sstream>
#include <thread>

using namespace BoBRobotics;
using namespace BoBRobotics::Robots;
using namespace std::literals;

constexpr auto imageWritePeriod = 100ms;

// Logs GPS data and images
class Logger
{
public:
    Logger(const filesystem::path &dbPath,
           Bebop &drone,
           HID::Joystick &joystick,
           std::mutex &loggerMutex)
      : m_DatabasePath{ dbPath }
      , m_Logger{ m_DatabasePath / "gps.csv" }
      , m_ImageWriterThread{ &Logger::logImages, this }
      , m_Joystick{ joystick }
      , m_Drone{ drone }
      , m_Camera{ drone.getVideoStream() }
      , m_LoggerMutex{ loggerMutex }
    {
        LOGI << "Saving files to " << m_DatabasePath;
        filesystem::create_directory(m_DatabasePath);
    }

    ~Logger()
    {
        m_StopFlag = true;
        if (m_ImageWriterThread.joinable()) {
            m_ImageWriterThread.join();
        }
    }

    void logGPS(const Bebop::GPSData &gps)
    {
        m_Logger.log(gps);
    }

private:
    const filesystem::path m_DatabasePath;
    Robots::BebopGPSLog m_Logger;
    std::thread m_ImageWriterThread;
    HID::Joystick &m_Joystick;
    Bebop &m_Drone;
    Bebop::VideoStream &m_Camera;
    std::mutex &m_LoggerMutex;
    std::atomic_bool m_StopFlag{ false };

    void logImages()
    {
        int imageCount = 0;
        Bebop::GPSData gps;
        cv::Mat fr;
        while (!m_StopFlag) {
            const auto imagePath = "image_" + std::to_string(++imageCount) + ".png";
            std::lock_guard<std::mutex> guard{ m_LoggerMutex };

            // We don't check return value because we don't care how new the data is
            m_Drone.getGPSData(gps);

            m_Camera.readFrameSync(fr);
            BOB_ASSERT(cv::imwrite((m_DatabasePath / imagePath).str(), fr));
            m_Logger.log(gps, imagePath);

            LOGD << "Saving image to " << imagePath;

            // Wait a bit
            std::this_thread::sleep_for(imageWritePeriod);
        }
    }
};

int bob_main(int, char **argv)
{
    // Save files relative to program's path
    const auto rootPath = filesystem::path{ argv[0] }.parent_path();
    std::unique_ptr<Logger> logger;
    std::mutex loggerMutex; // We access it from different threads

    HID::Joystick joystick;
    Bebop drone;
    drone.addJoystick(joystick);

    LOGI << "Toggle image collection with Y button";
    joystick.addHandler([&](HID::JButton button, bool pressed) {
        if (pressed && button == HID::JButton::Y) {
            std::lock_guard<std::mutex> guard{ loggerMutex };
            if (!logger) {
                // Start logging
                LOGI << "Starting logging";
                const auto dbPath = Path::getNewPath(rootPath);
                BOB_ASSERT(filesystem::create_directory(dbPath));
                logger = std::make_unique<Logger>(dbPath, drone, joystick, loggerMutex);
            } else {
                // Stop logging
                LOGI << "Stopping logging";
                logger.reset(); // Destroy logger object
            }
            return true;
        } else {
            return false;
        }
    });

    // If we're in logging mode, then log coords (this happens about once per sec)
    drone.setGPSUpdateHandler([&](const auto &gps) {
        LOGD << "Position: " << gps.toString();

        std::lock_guard<std::mutex> guard{ loggerMutex };
        if (logger) {
            logger->logGPS(gps);
        }
    });

    // Terminate program when drone lands
    drone.setFlyingStateChangedHandler([&joystick](const auto state) {
        switch (state) {
        case Bebop::FlyingState::Landing:
        case Bebop::FlyingState::Landed:
        case Bebop::FlyingState::Emergency:
        case Bebop::FlyingState::EmergencyLanding:
            joystick.stop();
        default:
            break;
        }
    });

    // Run on main thread
    joystick.run();

    return EXIT_SUCCESS;
}
