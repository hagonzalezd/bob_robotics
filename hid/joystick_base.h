#pragma once

// C includes
#include <cmath>
#include <cstdint>

// C++ includes
#include <array>
#include <functional>
#include <limits>
#include <string>
#include <vector>

// GeNN robotics includes
#include "../common/threadable.h"

namespace GeNNRobotics {
namespace HID {
#define DefaultDeadZone 0.25f

// helper macros
#define toIndex(value) static_cast<size_t>(value)
#define toAxis(value) static_cast<JAxis>(value)
#define toButton(value) static_cast<JButton>(value)

// maximum values
#define int16_maxf static_cast<float>(std::numeric_limits<int16_t>::max())
#define int16_absminf -static_cast<float>(std::numeric_limits<int16_t>::min())

/*
 * Controller axes, including thumbsticks, triggers and D-pad.
 */
enum class JAxis
{
    LeftStickHorizontal = 0,
    LeftStickVertical = 1,
    RightStickHorizontal = 3,
    RightStickVertical = 4,
    LeftTrigger = 2,
    RightTrigger = 5,
    DpadHorizontal = 6,
    DpadVertical = 7,
    LENGTH
};

enum ButtonState
{
    StateDown = (1 << 0),
    StatePressed = (1 << 1),
    StateReleased = (1 << 2)
};

template<typename JButton>
class JoystickBase : public Threadable
{
    using ButtonHandler = std::function<bool(JButton button, bool pressed)>;
    using AxisHandler = std::function<bool(JAxis axis, float value)>;

private:
    std::vector<ButtonHandler> m_ButtonHandlers;
    std::vector<AxisHandler> m_AxisHandlers;
    std::array<float, toIndex(JAxis::LENGTH)> m_AxisState;
    float m_DeadZone;

public:
    // Virtual methods
    virtual float axisToFloat(JAxis axis, int16_t value) const = 0;
    virtual bool update() = 0;

    void addHandler(AxisHandler handler)
    {
        m_AxisHandlers.insert(m_AxisHandlers.begin(), handler);
    }

    void addHandler(ButtonHandler handler)
    {
        m_ButtonHandlers.insert(m_ButtonHandlers.begin(), handler);
    }

    float getState(JAxis axis) const
    {
        return m_AxisState[toIndex(axis)];
    }

    unsigned char getState(JButton button) const
    {
        return m_ButtonState[toIndex(button)];
    }

    bool isDown(JButton button) const
    {
        return getState(button) & StateDown;
    }

    bool isPressed(JButton button) const
    {
        return getState(button) & StatePressed;
    }

    bool isReleased(JButton button) const
    {
        return getState(button) & StateReleased;
    }

    static std::string getName(JAxis axis)
    {
        switch (axis) {
        case JAxis::LeftStickHorizontal:
            return "left stick horizontal";
        case JAxis::LeftStickVertical:
            return "left stick vertical";
        case JAxis::RightStickHorizontal:
            return "right stick horizontal";
        case JAxis::RightStickVertical:
            return "right stick vertical";
        case JAxis::LeftTrigger:
            return "left trigger";
        case JAxis::RightTrigger:
            return "right trigger";
        case JAxis::DpadHorizontal:
            return "D-pad horizontal";
        case JAxis::DpadVertical:
            return "D-pad vertical";
        default:
            return "(unknown)";
        }
    }

    static std::string getName(JButton button)
    {
        // these values should be defined before this header is included
        switch (button) {
        case JButton::A:
            return "A";
        case JButton::B:
            return "B";
        case JButton::X:
            return "X";
        case JButton::Y:
            return "Y";
        case JButton::LB:
            return "LB";
        case JButton::RB:
            return "RB";
        case JButton::Back:
            return "back";
        case JButton::Start:
            return "start";
#ifndef _WIN32
        // this button is not available in Windows
        case JButton::Xbox:
            return "Xbox";
#endif
        case JButton::LeftStick:
            return "left stick";
        case JButton::RightStick:
            return "right stick";
        default:
            return "(unknown)";
        }
    }

protected:
    std::array<unsigned char, toIndex(JButton::LENGTH)> m_ButtonState;

    JoystickBase(float deadZone)
      : m_DeadZone(deadZone)
    {}

    void raiseEvent(JButton button, bool pressed)
    {
        for (auto handler : m_ButtonHandlers) {
            if (handler(button, pressed)) {
                break;
            }
        }
    }

    void updateAxis(JAxis axis, float value, bool isInitial)
    {
        auto &s = m_AxisState[toIndex(axis)];
        if (s != value) {
            s = value;

            // run handlers
            if (!isInitial) {
                for (auto handler : m_AxisHandlers) {
                    if (handler(axis, value)) {
                        break;
                    }
                }
            }
        }
    }

    void updateAxis(JAxis axis, int16_t value, bool isInitial)
    {
        updateAxis(axis, axisToFloat(axis, value), isInitial);
    }

    void updateAxis(JAxis axisHorz, int16_t hvalue, int16_t vvalue, bool isInitial)
    {
        JAxis axisVert = toAxis(toIndex(axisHorz) + 1);
        float x = axisToFloat(axisHorz, hvalue);
        float y = axisToFloat(axisVert, vvalue);

        if (sqrt(x * x + y * y) < m_DeadZone) {
            updateAxis(axisHorz, 0.0f, isInitial);
            updateAxis(axisVert, 0.0f, isInitial);
        } else {
            updateAxis(axisHorz, x, isInitial);
            updateAxis(axisVert, y, isInitial);
        }
    }

}; // JoystickBase
} // HID
} // GeNNRobotics
