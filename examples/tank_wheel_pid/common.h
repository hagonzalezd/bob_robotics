#pragma once

// BoB robotics includes
#include "common/assert.h"
#include "common/background_exception_catcher.h"
#include "common/circstat.h"
#include "common/main.h"
#include "common/plot_agent.h"
#include "common/stopwatch.h"
#include "hid/joystick.h"
#include "robots/tank_pid.h"

// Third-party includes
#include "third_party/units.h"

// Standard C includes
#include <cmath>

// Standard C++ includes
#include <iostream>
#include <limits>
#include <tuple>
#include <vector>

using namespace BoBRobotics;
using namespace units::literals;
using namespace units::angle;
using namespace units::length;
using namespace units::math;
using namespace units::time;
using namespace units::angular_velocity;
using namespace units::velocity;
namespace plt = matplotlibcpp;

template<typename TankPIDType, typename PoseableType>
void
runWheelPID(HID::Joystick &joystick, TankPIDType &robot, PoseableType &poseable)
{
    constexpr meter_t plotLimits = 2_m;

    robot.controlWithThumbsticks(joystick);

    std::cout << "Drive the robot using the two thumbsticks: each stick is for one motor" << std::endl;

    BackgroundExceptionCatcher catcher;
    catcher.trapSignals(); // Catch Ctrl-C

    Pose2<meter_t, radian_t> lastPose;
    std::vector<double> x(1), y(1);
    Stopwatch stopwatch;
    stopwatch.start();
    bool driveWithVelocities = false;
    joystick.addHandler([&](HID::JButton button, bool pressed) {
        if (pressed && button == HID::JButton::Y) {
            robot.stopMoving();
            driveWithVelocities = !driveWithVelocities;
            if (driveWithVelocities) {
                std::cout << "Driving using velocities" << std::endl;
                stopwatch.start();
                robot.start();
                robot.tankVelocities(0.05_mps, 0_mps);
            } else {
                std::cout << "Driving using standard controls" << std::endl;
            }
            // robot.setDriveWithVelocities(driveWithVelocities);
            return true;
        } else {
            return false;
        }
    });
    do {
        // Rethrow any exceptions caught on background thread
        catcher.check();

        const auto &currentPose = poseable.getPose();

        plt::figure(1);
        plt::clf();
        plotAgent(currentPose, -plotLimits, plotLimits, -plotLimits, plotLimits);
        if (driveWithVelocities && lastPose != currentPose) {
            robot.updatePose(currentPose, stopwatch.lap());
        }
        plt::pause(0.1);

        lastPose = currentPose;

        // Check for joystick events
        joystick.update();
    } while (plt::fignum_exists(1));
    plt::close();
}