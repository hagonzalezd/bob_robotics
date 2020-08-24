#pragma once

// BoB robotics includes
#include "logging.h"

// Standard C++ includes
#include <iostream>
#include <string>
#include <vector>

// Standard C includes
#include <cstring>

// POSIX includes
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/ioctl.h>

// Include to dump csv file
#include <stdio.h>


extern "C"
{
// I2C includes
#include <linux/i2c-dev.h>
#include<linux/i2c.h>

// This extra header is needed after Ubuntu 16.04 (newer kernel?)
/*
#ifndef I2C_SMBUS_BYTE_DATA
#include <i2c/smbus.h>
#endif
*/
}

namespace BoBRobotics {
//----------------------------------------------------------------------------
// BoBRobotics::I2CInterface
//----------------------------------------------------------------------------
//! Class for communicating over I2C
class I2CInterface_TUD
{
public:
    I2CInterface_TUD() : m_I2C(0)
    {}

    // Define log variable
    FILE * log;
    

    // Constructor
    I2CInterface_TUD(const char *path, int slaveAddress_left, int slaveAddress_right) : m_I2C(0)
    {
      // Open log file:
      log = fopen("log_csv.txt","w+");
      std::cout << "Log file created" << std::endl;

        setup(path, slaveAddress_left, slaveAddress_right);
    }

    // Destructor
    ~I2CInterface_TUD()
    {
        // Close I2C device
        if(m_I2C >= 0) {
            close(m_I2C);
        }

        LOG_DEBUG << "I2C closed";
        std::cout << "I2C closed" << std::endl;

	// CLosing the log file
	fclose(log);
        std::cout << "Log file closed" << std::endl;
	
    }

    //---------------------------------------------------------------------
    // Public API
    //---------------------------------------------------------------------
    void setup(const char *path, int slaveAddress_left, int slaveAddress_right )
    {
        m_I2C = open(path, O_RDWR);
        if (m_I2C < 0) {
            throw std::runtime_error("Error in setup: " + std::string(strerror(errno)) + "\n" +
                "The error is usually permission error which, on Ubuntu, can be fixed by" +
                "creating a file /etc/udev/rules.d/90-i2c.rules and adding the following line:\n" +
                "   KERNEL==\"i2c-[0-7]\",MODE=\"0666\"");
        }

        std::cout << "Setup performed only once" << std::endl;

	uint8_t command = 0x83;
	uint8_t address = slaveAddress_left;
	struct i2c_msg message = { address, 0, 1, &command };
	struct i2c_rdwr_ioctl_data ioctl_data = { &message, 1 };
	int result = ioctl(m_I2C, I2C_RDWR, &ioctl_data);
        std::cout << "After Setting up Left Motor" << std::endl;
	if (result != 1)
	{
		throw std::runtime_error("Cannot connect to I2C slave");
		//return -1;
                std::cout << "Cannot connect to I2C slave Left" << std::endl;
	}

	
	address = slaveAddress_right;
	message = { address, 0, 1, &command };
	ioctl_data = { &message, 1 };
	result = ioctl(m_I2C, I2C_RDWR, &ioctl_data);
        std::cout << "After Setting up Right Motor" << std::endl;
	if (result != 1)
	{
		throw std::runtime_error("Cannot connect to I2C slave");
                std::cout << "Cannot connect to I2C slave Right" << std::endl;

		//return -1;
	}
	
	//return 0;
        /*if (ioctl(m_I2C, I2C_SLAVE, slaveAddress) < 0) {
            throw std::runtime_error("Cannot connect to I2C slave");
        } else {
            LOG_INFO << "I2C successfully initialized";
        }*/
    }

    void refresh_connection(const char *path, int slaveAddress_left, int slaveAddress_right )
    {
        //std::cout << "+++ Refreshing the connection" << std::endl;
        // Close I2C device
        if(m_I2C >= 0) {
            close(m_I2C);
        }
        //std::cout << "+++ I2C closed" << std::endl;

        m_I2C = open(path, O_RDWR);
        if (m_I2C < 0) {
            throw std::runtime_error("Error in setup: " + std::string(strerror(errno)) + "\n" +
                "The error is usually permission error which, on Ubuntu, can be fixed by" +
                "creating a file /etc/udev/rules.d/90-i2c.rules and adding the following line:\n" +
                "   KERNEL==\"i2c-[0-7]\",MODE=\"0666\"");
        }
	
	uint8_t command = 0x83;
	uint8_t address = slaveAddress_left;
	struct i2c_msg message = { address, 0, 1, &command };
	struct i2c_rdwr_ioctl_data ioctl_data = { &message, 1 };
	int result = ioctl(m_I2C, I2C_RDWR, &ioctl_data);
        //std::cout << "+++ After Setting up Left Motor" << std::endl;
	if (result != 1)
	{
		throw std::runtime_error("Cannot connect to I2C slave");
                std::cout << "Cannot connect to I2C slave Left" << std::endl;
	}

	
	address = slaveAddress_right;
	message = { address, 0, 1, &command };
	ioctl_data = { &message, 1 };
	result = ioctl(m_I2C, I2C_RDWR, &ioctl_data);
        //std::cout << "+++ After Setting up Right Motor" << std::endl;
	if (result != 1)
	{
		throw std::runtime_error("Cannot connect to I2C slave");
                std::cout << "Cannot connect to I2C slave Right" << std::endl;
	}

	//fprintf(log,"\n");	
	//fprintf(log, "Connection refreshed! \n");
	//fprintf(log,"\n");
	
    }
/*
    uint8_t readByteCommand(uint8_t address)
    {
        const auto data = i2c_smbus_read_byte_data(m_I2C, address);
        if(data < 0) {
            throw std::runtime_error("Failed to read byte from i2c bus");
        } else {
            return static_cast<uint8_t>(data);
        }
    }

    uint8_t readByte()
    {
        const auto data = i2c_smbus_read_byte(m_I2C);
        if(data < 0) {
            throw std::runtime_error("Failed to read byte from i2c bus");
        } else {
            return static_cast<uint8_t>(data);
        }
    }

    template<typename T, size_t N>
    void read(T (&data)[N])
    {
        const size_t size = sizeof(T) * N;
        if (::read(m_I2C, &data[0], size) != size) {
            throw std::runtime_error("Failed to read from i2c bus");
        }
    }

    void writeByteCommand(uint8_t address, uint8_t byte)
    {
        if(i2c_smbus_write_byte_data(m_I2C, address, byte) < 0) {
            throw std::runtime_error("Failed to write byte to i2c bus");
        }
    }

    void writeByte(uint8_t byte)
    {
        if(i2c_smbus_write_byte(m_I2C, byte) < 0) {
            throw std::runtime_error("Failed to write byte to i2c bus");
        }
    }

    // writes data
    template<typename T, size_t N>
    void write(const T (&data)[N])
    {
        //LOG_INFO << "4th check";
	const size_t size = sizeof(T) * N;
        if (::write(m_I2C, &data[0], size) != size) {
            throw std::runtime_error("Failed to write to i2c bus");
        }
    }
*/
    void set_target_speed(int left_tud, int right_tud, int slaveAddress_left, int slaveAddress_right){
	//LOG_INFO << "Do Nothing";

        //std::cout << "Setting up target speed from the I2C_interface_tud.h" << std::endl;
        // Documentation:
        // This function is executed in a loop multiple times
        // Whenever we can't communicate or send the commands, this function is anyway executed
        // It seems the command ioctl() is not well received sometimes, but this is not shown in the result variable.
        // These errors are solved by implementing a refresh function to re launch the i2c devices
      

	uint8_t command[3];
	int16_t speed;
	uint8_t address = slaveAddress_left;
	if (left_tud < 0)
	{
		command[0] = 0x86; // Motor Reverse
		speed = -left_tud;
	}
	else
	{
		command[0] = 0x85; // Motor Forward
		speed = left_tud;
	}
	command[1] = speed & 0x1F;
	command[2] = speed >> 5 & 0x7F;
	struct i2c_msg message_l = { address, 0, sizeof(command), command };
	struct i2c_rdwr_ioctl_data ioctl_data_l = { &message_l, 1 };
	int result = ioctl(m_I2C, I2C_RDWR, &ioctl_data_l);

        //std::cout << " "<< std::endl;
        //std::cout << "left_tud: "<< left_tud << std::endl;
        //std::cout << "left_result: "<< result << std::endl;

	// Uncomment for logging
	//fprintf(log,"\n");
	//fprintf(log, "left_tud: %d \n", left_tud);
	//fprintf(log, "left_result: %d \n", result);
	  


	if (result != 1)
	{
		throw std::runtime_error("Cannot set speed to left side");
                std::cout << "Cannot set speed to left side from the I2C_interface_tud.h" << std::endl;

	}

	address = slaveAddress_right;
	if (right_tud < 0)
	{
		command[0] = 0x86; // Motor Reverse
		speed = -right_tud;
	}
	else
	{
		command[0] = 0x85; // Motor Forward
		speed = right_tud;
	}
	command[1] = speed & 0x1F;
	command[2] = speed >> 5 & 0x7F;
	struct i2c_msg message_r = { address, 0, sizeof(command), command };
	struct i2c_rdwr_ioctl_data ioctl_data_r = { &message_r, 1 };
	result = ioctl(m_I2C, I2C_RDWR, &ioctl_data_r);

        //std::cout << "right_tud: "<< right_tud << std::endl;
        //std::cout << "right_result: "<< result << std::endl;
        //std::cout << " "<< std::endl;

	// Uncomment for logging
	//fprintf(log, "right_tud: %d \n", right_tud);
	//fprintf(log, "right_result: %d \n", result);
	//fprintf(log,"\n");

	if (result != 1)
	{
		throw std::runtime_error("Cannot set speed to right side");
                std::cout << "Cannot set speed to right side from the I2C_interface_tud.h" << std::endl;

	}
    }

private:
    //---------------------------------------------------------------------
    // Members
    //---------------------------------------------------------------------
    int m_I2C;                                      // i2c file
};
} // BoBRobotics
