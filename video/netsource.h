#pragma once

// C++ includes
#include <mutex>
#include <string>
#include <vector>

// OpenCV
#include <opencv2/opencv.hpp>

// GeNN robotics includes
#include "../common/semaphore.h"
#include "../net/node.h"

// local includes
#include "input.h"

namespace GeNNRobotics {
namespace Video {
class NetSource : public Input
{
public:
    NetSource()
    {}

    NetSource(Net::Node &node)
    {
        // handle incoming IMG commands
        node.addCommandHandler("IMG", [this] (Net::Node &node, const Net::Command &command) {
            onCommandReceived(node, command);
        });

        // when connected, send command to start streaming
        node.addConnectedHandler([this] (Net::Node &node) {
            node.getSocket()->send("IMG START\n");
        });
    }

    const std::string getCameraName() const override
    {
        return m_CameraName;
    }

    cv::Size getOutputSize() const override
    {
        return m_CameraResolution;
    }

    const bool needsUnwrapping() override
    {
        m_ParamsSemaphore.waitOnce();
        return Input::needsUnwrapping();
    }

    bool readFrame(cv::Mat &frame) override
    {
        m_ReadSemaphore.wait();
        std::lock_guard<std::mutex> guard(m_BufferMutex);
        cv::imdecode(m_Buffer, cv::IMREAD_UNCHANGED, &frame);
        return true;
    }

private:
    std::string m_CameraName = DefaultCameraName;
    cv::Size m_CameraResolution;
    std::vector<uchar> m_Buffer;
    std::mutex m_BufferMutex;
    Semaphore m_ParamsSemaphore, m_ReadSemaphore;

    void onCommandReceived(Net::Node &node, const Net::Command &command)
    {
        if (command[1] == "PARAMS") {
            m_CameraResolution.width = stoi(command[2]);
            m_CameraResolution.height = stoi(command[3]);
            m_CameraName = command[4];
            m_ParamsSemaphore.notify();
        } else if (command[1] == "FRAME") {
            std::lock_guard<std::mutex> guard(m_BufferMutex);
            size_t nbytes = stoi(command[2]);
            m_Buffer.resize(nbytes);
            node.getSocket()->read(m_Buffer.data(), nbytes);
            m_ReadSemaphore.notify();
        } else {
            throw Net::bad_command_error();
        }
    }
}; // NetSource
} // Video
} // GeNNRobotics
