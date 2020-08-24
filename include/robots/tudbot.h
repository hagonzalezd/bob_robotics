#pragma once
#ifdef __linux__

// BoB robotics includes
#include "../common/i2c_interface_tud.h"
#include "tank.h"

// Standard C includes
#include <cmath>
#include <cstdint>

#include <iostream>

// Standard C++ includes
#include <vector>

// third party includes
#include "../third_party/units.h"

// Include to dump csv file
#include <stdio.h>
#include <stdlib.h>

namespace BoBRobotics {
namespace Robots {
//----------------------------------------------------------------------------
// BoBRobotics::Robots::TUDbot
//----------------------------------------------------------------------------
//! An interface for wheeled, Arduino-based robots developed at TU Dresden, and inspired by Norbot

class TUDbot : public Tank
{
    using meters_per_second_t = units::velocity::meters_per_second_t;
    using millimeter_t = units::length::millimeter_t;

public:
    unsigned int watchdog_timer;
    unsigned int refresh_rate = 0x3e8;
    
    TUDbot(const char *path = "/dev/i2c-0", int slaveAddress_left = 0xc, int slaveAddress_right = 0xd);
    
    //----------------------------------------------------------------------------
    // Tank virtuals
    //----------------------------------------------------------------------------
    virtual ~TUDbot() override;

    virtual void tank(float left, float right) override;

    virtual meters_per_second_t getAbsoluteMaximumSpeed() const override;

    virtual millimeter_t getRobotWidth() const override;

    //----------------------------------------------------------------------------
    // Public API
    //----------------------------------------------------------------------------
    void set_target_speed(int left_tud, int right_tud, int slaveAddress_left, int slaveAddress_right, const char *path)
    {

	m_I2C.set_target_speed(left_tud, right_tud, slaveAddress_left, slaveAddress_right);


	if (watchdog_timer == refresh_rate){ // To prevent exclusion errors for an individual i2c device
	  m_I2C.refresh_connection(path, slaveAddress_left,slaveAddress_right);
	  watchdog_timer = 0;
	}
	else {
          watchdog_timer++;
	}


    }

private:
    //----------------------------------------------------------------------------
    // Private members
    //----------------------------------------------------------------------------
    BoBRobotics::I2CInterface_TUD m_I2C;
}; // TUDbot
} // Robots
} // BoBRobotics
#endif // __linux__
