#ifdef __linux__
// BoB robotics includes
#include "robots/tudbot.h"

// Standard C includes
#include <cmath>
#include <cstdint>

#include <iostream>

// Standard C++ includes
#include <vector>

// Include to dump csv file
#include <stdio.h>
#include <stdlib.h>

using namespace units::literals;
using namespace units::length;
using namespace units::velocity;


uint8_t floatToI2C(float speed)
{
        return (uint8_t) std::min(
                255.0f,
                std::max(0.0f, std::round(((speed + 1.0f) / 2.0f) * 255.0f)));
}

namespace BoBRobotics {
namespace Robots {

  TUDbot::TUDbot(const char *path, int slaveAddress_left, int slaveAddress_right)
      : m_I2C(path, slaveAddress_left, slaveAddress_right)
    {
        // Sometimes Norbots get stuck driving, so let's stop it if we need to
        stopMoving();
        std::cout << "Motion stopped" << std::endl;
	// Executed at start:
        watchdog_timer = 0; 	
    }

    //----------------------------------------------------------------------------
    // Tank virtuals
    //----------------------------------------------------------------------------
  TUDbot::~TUDbot()
    {
        watchdog_timer = 0;
        stopMoving();
        std::cout << "Motion stopped " << std::endl;
        stopReadingFromNetwork();
        std::cout << "Stop Reading from network " << std::endl;
    }

  void TUDbot::tank(float left, float right)
    {
        setWheelSpeeds(left, right);
	int slaveAddress_left = 12;
	int slaveAddress_right = 13;
	const char *path = "/dev/i2c-0";
	//LOG_INFO << "3rd check";

        // Cache left and right
        // const float maxSpeed = getMaximumSpeedProportion();
	const float maxSpeed = 3200;
        left *= maxSpeed;
        right *= maxSpeed;
	//LOG_INFO<<"Left:%04x"<<left<<"Right:%04x"<<right;

        // Convert standard (-1,1) values to bytes in order to send to I2C slave
        //uint8_t buffer[2] = { floatToI2C(left), floatToI2C(right) };

        // Send buffer
        //write(buffer);
	int left_tud = left;
	int right_tud = right;
	right_tud *= (-1);
	set_target_speed(left_tud,right_tud,slaveAddress_left,slaveAddress_right, path);
    }

  meters_per_second_t TUDbot::getAbsoluteMaximumSpeed() const
    {
        return 0.11_mps;
    }

  millimeter_t TUDbot::getRobotWidth() const
    {
        return 104_mm;
    }

} // Robots
} // BoBRobotics
#endif
