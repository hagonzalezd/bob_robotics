#pragma once

// BoB robotics includes
#include "../common/assert.h"
#include "../common/pose.h"
#include "../common/stopwatch.h"
#include "../os/net.h"

// Standard C++ includes
#include <algorithm>
#include <atomic>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace BoBRobotics
{
namespace Vicon
{
using namespace units::literals;

//----------------------------------------------------------------------------
// Vicon::ObjectData
//----------------------------------------------------------------------------
//! Simplest object data class - just tracks position and attitude
class ObjectData
{
    using radian_t = units::angle::radian_t;
    using millimeter_t = units::length::millimeter_t;

public:
    ObjectData()
      : m_FrameNumber{ 0 }
      , m_Position{ 0_mm, 0_mm, 0_mm }
      , m_Attitude{ 0_rad, 0_rad, 0_rad }
    {
    }

    //----------------------------------------------------------------------------
    // Public API
    //----------------------------------------------------------------------------
    void update(const char(&name)[24], uint32_t frameNumber, millimeter_t x, millimeter_t y, millimeter_t z,
                radian_t yaw, radian_t pitch, radian_t roll)
    {
        // Copy name
        // **NOTE** this must already be NULL-terminated
        memcpy(&m_Name[0], &name[0], 24);

        // Log the time when this packet was received
        m_ReceivedTimer.start();

        // Cache frame number
        m_FrameNumber = frameNumber;

        // Copy vectors into class
        m_Position[0] = x;
        m_Position[1] = y;
        m_Position[2] = z;
        m_Attitude[0] = yaw;
        m_Attitude[1] = pitch;
        m_Attitude[2] = roll;
    }

    uint32_t getFrameNumber() const
    {
        return m_FrameNumber;
    }

    template <class LengthUnit = millimeter_t>
    Vector3<LengthUnit> getPosition() const
    {
        return convertUnitArray<LengthUnit>(m_Position);
    }

    template <class AngleUnit = radian_t>
    Vector3<AngleUnit> getAttitude() const
    {
        return convertUnitArray<AngleUnit>(m_Attitude);
    }

    const char *getName() const{ return m_Name; }

    auto timeSinceReceived() const { return m_ReceivedTimer.elapsed(); }

private:
    //----------------------------------------------------------------------------
    // Members
    //----------------------------------------------------------------------------
    uint32_t m_FrameNumber;
    char m_Name[24];
    Vector3<millimeter_t> m_Position;
    Vector3<radian_t> m_Attitude;
    Stopwatch m_ReceivedTimer;
};

//----------------------------------------------------------------------------
// Vicon::ObjectDataVelocity
//----------------------------------------------------------------------------
//! Object data class which also calculate (un-filtered) velocity
class ObjectDataVelocity : public ObjectData
{
    using radian_t = units::angle::radian_t;
    using meters_per_second_t = units::velocity::meters_per_second_t;
    using millimeter_t = units::length::millimeter_t;
    using millisecond_t = units::time::millisecond_t;

public:
    ObjectDataVelocity() : m_Velocity{0_mps, 0_mps, 0_mps}
    {}

    //----------------------------------------------------------------------------
    // Public API
    //----------------------------------------------------------------------------
    void update(const char(&name)[24], uint32_t frameNumber, millimeter_t x, millimeter_t y, millimeter_t z,
                radian_t yaw, radian_t pitch, radian_t roll)
    {
        // Superclass
        ObjectData::update(name, frameNumber, x, y, z, yaw, pitch, roll);

        const Vector3<millimeter_t> position{ x, y, z };
        constexpr millisecond_t frameS = 10_ms;
        constexpr millisecond_t smoothingS = 50_ms;

        // Calculate time since last frame
        const uint32_t deltaFrames = frameNumber - getFrameNumber();
        const auto deltaS = frameS * deltaFrames;

        // Calculate exponential smoothing factor
        const double alpha = 1.0 - units::math::exp(-deltaS / smoothingS);

        // Calculate instantaneous velocity
        const auto oldPosition = getPosition<>();
        Vector3<meters_per_second_t> instVelocity;
        const auto calcVelocity = [deltaS](auto curr, auto prev) {
            return (curr - prev) / deltaS;
        };
        std::transform(std::begin(position), std::end(position),
                       std::begin(oldPosition), std::begin(instVelocity),
                       calcVelocity);

        // Exponentially smooth velocity
        const auto smoothVelocity = [alpha](auto inst, auto prev) {
            return (alpha * inst) + ((1.0 - alpha) * prev);
        };

        std::transform(std::begin(instVelocity), std::end(instVelocity),
                       std::begin(m_Velocity), std::begin(m_Velocity),
                       smoothVelocity);
    }

    template <class VelocityUnit = meters_per_second_t>
    Vector3<VelocityUnit> getVelocity() const
    {
        return convertUnitArray<VelocityUnit>(m_Velocity);
    }

private:
    //----------------------------------------------------------------------------
    // Members
    //----------------------------------------------------------------------------
    Vector3<meters_per_second_t> m_Velocity;
};

//----------------------------------------------------------------------------
// BoBRobotics::Vicon::UDPClient
//----------------------------------------------------------------------------
//! Receiver for Vicon UDP streams
template<typename ObjectDataType = ObjectData>
class UDPClient
{
public:
    UDPClient(){}
    UDPClient(uint16_t port)
    {
        connect(port);
    }

    virtual ~UDPClient()
    {
        // Set quit flag and join read thread
        if(m_ReadThread.joinable()) {
            m_ShouldQuit = true;
            m_ReadThread.join();
        }
    }

    //----------------------------------------------------------------------------
    // Public API
    //----------------------------------------------------------------------------
    void connect(uint16_t port)
    {
        // Create socket
        int socket = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if(socket < 0) {
            throw OS::Net::NetworkError("Cannot open socket");
        }

        // Set socket to have 1s read timeout
        // **NOTE** this is largely to allow read thread to be stopped
#ifdef _WIN32
        DWORD timeout = 1000;
#else
        timeval timeout;
        timeout.tv_sec = 1;
        timeout.tv_usec = 0;
#endif
        if(setsockopt(socket, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) != 0) {
            throw OS::Net::NetworkError("Cannot set socket timeout");
        }

        // Create socket address structure
        sockaddr_in localAddress;
        memset(&localAddress, 0, sizeof(sockaddr_in));
        localAddress.sin_family = AF_INET;
        localAddress.sin_port = htons(port);
        localAddress.sin_addr.s_addr = htonl(INADDR_ANY);

        // Bind socket to local port
        if(bind(socket, reinterpret_cast<sockaddr*>(&localAddress), sizeof(localAddress)) < 0) {
            throw OS::Net::NetworkError("Cannot bind socket");
        }

        // Clear atomic stop flag and start thread
        m_ShouldQuit = false;
        m_ReadThread = std::thread(&UDPClient::readThread, this, socket);
    }

    unsigned int getNumObjects()
    {
        std::lock_guard<std::mutex> guard(m_ObjectDataMutex);
        return m_ObjectData.size();
    }

    unsigned int findObjectID(const std::string &name)
    {
        // Search for object with name
        std::lock_guard<std::mutex> guard(m_ObjectDataMutex);
        auto objIter = std::find_if(m_ObjectData.cbegin(), m_ObjectData.cend(),
            [&name](const ObjectDataType &object)
            {
                return (strcmp(object.getName(), name.c_str()) == 0);
            });

        // If object wasn't found, raise error
        if(objIter == m_ObjectData.cend()) {
            throw std::out_of_range("Cannot find object '" + name + "'");
        }
        // Otherwise, return its index i.e. object ID
        else {
            return (unsigned int)std::distance(m_ObjectData.cbegin(), objIter);
        }
    }

    ObjectDataType getObjectData(unsigned int id)
    {
        std::lock_guard<std::mutex> guard(m_ObjectDataMutex);
        return m_ObjectData.at(id);
    }

private:
    //----------------------------------------------------------------------------
    // Private API
    //----------------------------------------------------------------------------
    void updateObjectData(unsigned int id, const char(&name)[24],
                          uint32_t frameNumber,
                          const Vector3<double> &position,
                          const Vector3<double> &attitude)
    {
        // Lock mutex
        std::lock_guard<std::mutex> guard(m_ObjectDataMutex);

        // If no object data structure has been created for this ID, add one
        if (id >= m_ObjectData.size()) {
            m_ObjectData.resize(id + 1);
        }

        /*
         * Update object data with position and attitude.
         *
         * Note that we reorder the rotation angles we get from the Vicon system
         * so that they are in the order of yaw, pitch and roll (which seems to
         * be standard).
         */
        using namespace units::length;
        using namespace units::angle;
        m_ObjectData[id].update(name, frameNumber,
                                millimeter_t(position[0]),
                                millimeter_t(position[1]),
                                millimeter_t(position[2]),
                                radian_t(attitude[2]),
                                radian_t(attitude[0]),
                                radian_t(attitude[1]));
    }

    void readThread(int socket)
    {
        // Create buffer for reading data
        // **NOTE** this is the maximum size supported by Vicon so will support all payload sizes
        uint8_t buffer[1024];

        // Loop until quit flag is set
        for(unsigned int f = 0; !m_ShouldQuit; f++) {
            // Read datagram
            const ssize_t bytesReceived = recvfrom(socket, &buffer[0], 1024,
                                                   0, nullptr, nullptr);

            // If there was an error
            if(bytesReceived == -1) {
                // If this was a timeout, continue
                if(errno == EAGAIN || errno == EINTR) {
                    continue;
                }
                // Otherwise, display error and stop
                else {
                    throw OS::Net::NetworkError("Cannot read datagram");
                }
            }
            // Otherwise, if data was received
            else {
                // Read frame number
                uint32_t frameNumber;
                memcpy(&frameNumber, &buffer[0], sizeof(uint32_t));

                // Read items in block
                const unsigned int itemsInBlock = (unsigned int)buffer[4];

                // Loop through items in blcok
                unsigned int itemOffset = 5;
                for(unsigned int i = 0; i < itemsInBlock; i++) {
                    // Read object ID
                    const unsigned int objectID = (unsigned int)buffer[itemOffset];

                    // Read size of item
                    uint16_t itemDataSize;
                    memcpy(&itemDataSize, &buffer[itemOffset + 1], sizeof(uint16_t));
                    BOB_ASSERT(itemDataSize == 72);

                    // Read object name and check it is NULL-terminated
                    char objectName[24];
                    memcpy(&objectName[0], &buffer[itemOffset + 3], 24);
                    BOB_ASSERT(objectName[23] == '\0');

                    // Read object position
                    Vector3<double> position;
                    memcpy(&position[0], &buffer[itemOffset + 27], 3 * sizeof(double));

                    // Read object attitude
                    Vector3<double> attitude;
                    memcpy(&attitude[0], &buffer[itemOffset + 51], 3 * sizeof(double));

                    // Update item
                    updateObjectData(objectID, objectName, frameNumber, position, attitude);

                    // Update offset for next offet
                    itemOffset += itemDataSize;
                }
            }
        }

        // Close socket
        close(socket);
    }

    //----------------------------------------------------------------------------
    // Members
    //----------------------------------------------------------------------------
    std::atomic<bool> m_ShouldQuit;
    std::thread m_ReadThread;
    void *m_ReadUserData;

    std::mutex m_ObjectDataMutex;
    std::vector<ObjectDataType> m_ObjectData;
};
} // namespace Vicon
} // BoBRobotics
