// BoB robotics includes
#include "common/logging.h"
#include "common/macros.h"
#include "common/main.h"
#include "common/path.h"
#include "hid/joystick.h"
#include "robots/bebop/bebop.h"
#include "robots/bebop/gps_log.h"

#ifdef OPTION_DUMMY
#include "video/randominput.h"
#include <random>
#endif

// OpenCV
#include <opencv2/opencv.hpp>

// Standard C++ includes
#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <sstream>
#include <thread>

using namespace BoBRobotics;
using namespace BoBRobotics::Robots;
using namespace std::literals;

constexpr auto imageWritePeriod = 250ms;

using GPSGetter = std::function<void(Bebop::GPSData &)>;

// Logs GPS data and images
class Logger
{
public:
    Logger(const filesystem::path &dbPath,
           GPSGetter gpsGetter,
           Video::Input &camera,
           HID::Joystick &joystick,
           std::mutex &loggerMutex)
      : m_DatabasePath{ dbPath }
      , m_Logger{ m_DatabasePath / "gps.csv" }
      , m_ImageWriterThread{ &Logger::logImages, this }
      , m_Joystick{ joystick }
      , m_GPSGetter{ gpsGetter }
      , m_Camera{ camera }
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
    GPSGetter m_GPSGetter;
    Video::Input &m_Camera;
    std::mutex &m_LoggerMutex;
    std::atomic_bool m_StopFlag{ false };

    void logImages()
    {
        int imageCount = 0;
        Bebop::GPSData gps;
        cv::Mat fr;
        while (!m_StopFlag) {
            const auto imagePath = "image_" + std::to_string(++imageCount) + ".png";

            m_GPSGetter(gps);
            m_Camera.readFrame(fr);
            BOB_ASSERT(cv::imwrite((m_DatabasePath / imagePath).str(), fr));

            std::lock_guard<std::mutex> guard{ m_LoggerMutex };
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
    std::shared_ptr<Logger> logger;
    std::mutex loggerMutex; // We access it from different threads

    HID::Joystick joystick;

    auto onGPSUpdate = [&](const Bebop::GPSData &gps) {
        LOGD << "Position: " << gps.toString();

        auto loggerLocal = logger;
        if (loggerLocal) {
            std::lock_guard<std::mutex> guard{ loggerMutex };
            loggerLocal->logGPS(gps);
        }
    };

#ifdef OPTION_DUMMY
    using namespace units::angle;
    using namespace units::length;
    Video::RandomInput<> camera({ 100, 100 });

    const auto getGPS = [](Bebop::GPSData &gps) {
        const auto rnd = [](double low, double high) {
            static std::default_random_engine generator{ std::random_device{}() };
            static std::uniform_real_distribution<double> dist{ 0, 1 };
            return low + dist(generator) * (high - low);
        };

        gps.coordinate.lat = degree_t{ rnd(-90, 90) };
        gps.coordinate.lon = degree_t{ rnd(-180, 180) };
        gps.coordinate.height = meter_t{ rnd(5, 15) };
        gps.latError = meter_t{ rnd(-5, 5) };
        gps.lonError = meter_t{ rnd(-5, 5) };
        gps.heightError = meter_t{ -3, 3 };
        gps.numberOfSatellites = 3;
    };

    // Simulate drone by giving random GPS readings every second
    std::thread gpsThread{ [&]() {
        while (true) {
            std::this_thread::sleep_for(1s);
            Bebop::GPSData gps;
            getGPS(gps);
            onGPSUpdate(gps);
        }
    } };
#else
    Bebop drone;
    auto &camera = drone.getVideoStream();
    drone.addJoystick(joystick);

    // If we're in logging mode, then log coords (this happens about once per sec)
    drone.setGPSUpdateHandler(onGPSUpdate);

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

    auto getGPS = [&drone](Bebop::GPSData &gps) {
        drone.getGPSData(gps);
    };
#endif

    LOGI << "Toggle image collection with Y button";
    joystick.addHandler([&](HID::JButton button, bool pressed) {
        if (pressed && button == HID::JButton::Y) {
            if (!logger) {
                // Start logging
                LOGI << "Starting logging";
                const auto dbPath = Path::getNewPath(rootPath);
                BOB_ASSERT(filesystem::create_directory(dbPath));
                logger = std::make_shared<Logger>(dbPath, getGPS, camera, joystick, loggerMutex);
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

    // Run on main thread
    joystick.run();

    return EXIT_SUCCESS;
}
