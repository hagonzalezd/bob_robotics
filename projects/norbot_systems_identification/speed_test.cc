// BoB robotics includes
#include "common/background_exception_catcher.h"
#include "common/main.h"
#include "common/pose.h"
#include "common/stopwatch.h"
#include "hid/joystick.h"
#include "net/client.h"
#include "robots/tank_netsink.h"
#include "vicon/udp.h"

// Third-party includes
#include "third_party/path.h"
#include "third_party/units.h"

// Standard C++ includes
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>

using namespace BoBRobotics;
using namespace std::literals;
using namespace units::length;
using namespace units::time;

class DataFile
{
public:
    DataFile(Robots::Tank &robot, Vicon::UDPClient<> &vicon)
      : m_Robot(robot)
      , m_Vicon(vicon)
    {
        // Set the driving speed at which we're testing the robot
        float robotSpeed;
        do {
            std::cout << "Enter desired robot speed: ";
            std::cin >> robotSpeed;
        } while (robotSpeed < -1.f || robotSpeed > 1.f);

        // Construct path for data file
        const filesystem::path dataDir = "data";
        filesystem::create_directory(dataDir); // Make sure folder exists
        std::stringstream ss;
        ss << "data@" << robotSpeed << ".csv";
        m_FilePath = dataDir / ss.str();

        // Open file and write header
        m_FileStream = std::ofstream(m_FilePath.str());
        m_FileStream << "x, y, t\n";
        m_FileStream << std::setprecision(10); // set number of decimal places

        // Start driving robot forwards; start stopwatch
        robot.tank(robotSpeed, robotSpeed);
        m_Stopwatch.start();

        std::cout << "Recording" << std::endl;
    }

    ~DataFile()
    {
        m_Robot.stopMoving();
        std::cout << "Data written to " << m_FilePath << std::endl;
    }

    bool update()
    {
        const auto elapsed = m_Stopwatch.elapsed();
        if (m_StopwatchSample.elapsed() >= 100ms) {
            // Restart timer
            m_StopwatchSample.start();

            // Get coordinates from Vicon
            const auto position = m_Vicon.getObjectData(0).getPosition<meter_t>();

            // Write to CSV file
            m_FileStream << position[0].value() << ", " << position[1].value()
                         << ", " << static_cast<second_t>(elapsed).value() << "\n";

            return true;
        } else {
            return false;
        }
    }

    bool finished() const
    {
        return m_Stopwatch.elapsed() > 10s;
    }

private:
    std::ofstream m_FileStream;
    filesystem::path m_FilePath;
    Robots::Tank &m_Robot;
    Vicon::UDPClient<> &m_Vicon;
    Stopwatch m_Stopwatch, m_StopwatchSample;
};

int
bob_main(int argc, char **argv)
{
    std::string ipAddress;
    if (argc > 1) {
        ipAddress = argv[1];
    } else {
        std::cout << "IP address [127.0.0.1]: ";
        std::getline(std::cin, ipAddress);
        if (ipAddress.empty()) {
            ipAddress = "127.0.0.1";
        }
    }

    // Connect to robot
    std::cout << "Connecting to robot" << std::endl;
    Net::Client client(ipAddress);
    std::cout << "Connected to " << ipAddress << std::endl;

    // Send motor commands to robot
    Robots::TankNetSink robot(client);

    // Open joystick
    HID::Joystick joystick;
    robot.addJoystick(joystick);
    std::cout << "Opened joystick" << std::endl;

    std::unique_ptr<DataFile> dataFile; // CSV output file
    Vicon::UDPClient<> vicon(51001); // For getting robot's position

    // If we're recording, ignore axis movements
    joystick.addHandler([&dataFile](HID::JAxis, float) {
        if (dataFile != nullptr) {
            std::cout << "Ignoring joystick command" << std::endl;
            return true;
        } else {
            return false;
        }
    });

    // Toggle testing mode with buttons
    joystick.addHandler([&dataFile, &robot, &vicon](HID::JButton button, bool pressed) {
        if (!pressed) {
            return false;
        }

        if (button == HID::JButton::Y) {
            // Start recording
            if (!dataFile) {
                // Open a new file for writing
                dataFile = std::make_unique<DataFile>(robot, vicon);
            }
            return true;
        } else if (button == HID::JButton::X) {
            // Stop recording
            if (dataFile) {
                dataFile.reset();
            }
            return true;
        } else {
            return false;
        }
    });

    // Wait for Vicon system
    while (vicon.getNumObjects() == 0) {
        std::cout << "Waiting for object" << std::endl;
        std::this_thread::sleep_for(1s);
    }
    std::cout << "Connected to Vicon system" << std::endl;

    // Run client in background, checking for background errors thrown
    BackgroundExceptionCatcher catcher;
    catcher.trapSignals(); // Catch ctrl-C
    client.runInBackground();
    while (!joystick.isPressed(HID::JButton::B)) {
        // Check for errors
        catcher.check();

        // Poll joystick for events
        const bool joystickUpdate = joystick.update();

        if (dataFile && dataFile->update()) { // If we're recording
            if (dataFile->finished()) {
                dataFile.reset();
            }
        } else if (!joystickUpdate) {
            std::this_thread::sleep_for(5ms);
        }
    }

    return EXIT_SUCCESS;
}