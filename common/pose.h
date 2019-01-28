#pragma once

// Third-party includes
#include "../third_party/units.h"

// Standard C++ includes
#include <array>
#include <tuple>

namespace BoBRobotics {

template<typename Derived>
class PoseBase
{
public:
    template<typename PoseType>
    bool operator==(const PoseType &pose) const
    {
        const auto derived = reinterpret_cast<const Derived *>(this);
        return derived->x() == pose.x() && derived->y() == pose.y() && derived->z() == pose.z()
                && derived->yaw() == pose.yaw() && derived->pitch() == pose.pitch() && derived->roll() == pose.roll();
    }

    template<typename PoseType>
    bool operator!=(const PoseType &pose) const
    {
        return !(*this == pose);
    }
};

//! Base class for vectors of length units
template<typename LengthUnit, size_t N>
class VectorBase
{
    static_assert(units::traits::is_length_unit<LengthUnit>::value,
                  "LengthUnit is not a unit of length");
    using radian_t = units::angle::radian_t;

public:
    VectorBase() = default;

    template<typename... Ts>
    VectorBase(Ts &&... args)
      : m_Array({ std::forward<Ts>(args)... })
    {}

    operator const std::array<LengthUnit, N> &() const
    {
        return m_Array;
    }

    LengthUnit &operator[](size_t i) { return m_Array[i]; }
    const LengthUnit &operator[](size_t i) const { return m_Array[i]; }
    static constexpr size_t size() { return N; }

    auto begin() { return m_Array.begin(); }
    auto begin() const { return m_Array.begin(); }
    auto end() { return m_Array.end(); }
    auto end() const { return m_Array.end(); }
    auto cbegin() const { return m_Array.cbegin(); }
    auto cend() const { return m_Array.end(); }

    static constexpr radian_t yaw() { return radian_t(0); }
    static constexpr radian_t pitch() { return radian_t(0); }
    static constexpr radian_t roll() { return radian_t(0); }

private:
    std::array<LengthUnit, N> m_Array;
};

template<typename LengthUnit>
class Vector3;

//! 2D length unit vector
template<typename LengthUnit>
class Vector2
  : public VectorBase<LengthUnit, 2>
  , public PoseBase<Vector2<LengthUnit>>
{
public:
    Vector2() = default;

    Vector2(LengthUnit x, LengthUnit y)
      : VectorBase<LengthUnit, 2>(x, y)
    {}

    Vector2(const std::array<LengthUnit, 2> &array)
      : Vector2(array[0], array[1])
    {}

    operator Vector3<LengthUnit>() const { return Vector3<LengthUnit>(x(), y(), z()); }

    LengthUnit &x() { return (*this)[0]; }
    const LengthUnit &x() const { return (*this)[0]; }
    LengthUnit &y() { return (*this)[1]; }
    const LengthUnit &y() const { return (*this)[1]; }
    static constexpr LengthUnit z() { return LengthUnit(0); }
};

//! 3D length unit vector
template<typename LengthUnit>
class Vector3
  : public VectorBase<LengthUnit, 3>
  , public PoseBase<Vector3<LengthUnit>>
{
public:
    Vector3() = default;

    Vector3(LengthUnit x, LengthUnit y, LengthUnit z)
      : VectorBase<LengthUnit, 3>(x, y, z)
    {}

    Vector3(const std::array<LengthUnit, 3> &array)
      : Vector3(array[0], array[1], array[2])
    {}

    operator Vector2<LengthUnit>() const { return Vector2<LengthUnit>(x(), y()); }

    LengthUnit &x() { return (*this)[0]; }
    const LengthUnit &x() const { return (*this)[0]; }
    LengthUnit &y() { return (*this)[1]; }
    const LengthUnit &y() const { return (*this)[1]; }
    LengthUnit &z() { return (*this)[2]; }
    const LengthUnit &z() const { return (*this)[2]; }
};

template<typename LengthUnit, typename AngleUnit>
class Pose3;

//! A two-dimensional pose
template<typename LengthUnit, typename AngleUnit>
class Pose2
  : public std::tuple<Vector2<LengthUnit>, AngleUnit>
  , public PoseBase<Pose2<LengthUnit, AngleUnit>>
{
    static_assert(units::traits::is_length_unit<LengthUnit>::value,
                  "LengthUnit is not a unit of length");
    static_assert(units::traits::is_angle_unit<AngleUnit>::value,
                  "AngleUnit is not a unit of angle");

public:
    Pose2() = default;

    Pose2(LengthUnit x, LengthUnit y, AngleUnit angle)
      : std::tuple<Vector2<LengthUnit>, AngleUnit>({ x, y }, angle)
    {}

    template<typename LengthUnit2, typename AngleUnit2>
    operator Pose2<LengthUnit2, AngleUnit2>() const
    {
        return Pose2<LengthUnit2, AngleUnit2>{ x(), y(), yaw() };
    }

    template<typename LengthUnit2, typename AngleUnit2>
    operator Pose3<LengthUnit2, AngleUnit2>() const
    {
        return Pose3<LengthUnit2, AngleUnit2>{ { x(), y(), z() }, { yaw(), pitch(), roll() } };
    }

    Vector2<LengthUnit> &position() { return std::get<0>(*this); }
    const Vector2<LengthUnit> &position() const { return std::get<0>(*this); }
    LengthUnit &x() { return std::get<0>(*this)[0]; }
    const LengthUnit &x() const { return std::get<0>(*this)[0]; }
    LengthUnit &y() { return std::get<0>(*this)[1]; }
    const LengthUnit &y() const { return std::get<0>(*this)[1]; }
    static constexpr LengthUnit z() { return LengthUnit(0); }

    std::array<AngleUnit, 3> attitude() const { return { yaw(), AngleUnit(0), AngleUnit(0) }; }
    AngleUnit &yaw() { return std::get<1>(*this); }
    const AngleUnit &yaw() const { return std::get<1>(*this); }
    static constexpr AngleUnit pitch() { return AngleUnit(0); }
    static constexpr AngleUnit roll() { return AngleUnit(0); }
};

//! A three-dimensional pose
template<typename LengthUnit, typename AngleUnit>
class Pose3
  : public std::tuple<Vector3<LengthUnit>, std::array<AngleUnit, 3>>
  , public PoseBase<Pose3<LengthUnit, AngleUnit>>
{
    static_assert(units::traits::is_length_unit<LengthUnit>::value,
                  "LengthUnit is not a unit of length");
    static_assert(units::traits::is_angle_unit<AngleUnit>::value,
                  "AngleUnit is not a unit of angle");

public:
    Pose3() = default;

    Pose3(const Vector3<LengthUnit> &position, const std::array<AngleUnit, 3> &attitude)
      : std::tuple<Vector3<LengthUnit>, std::array<AngleUnit, 3>>(position, attitude)
    {}

    template<typename LengthUnit2, typename AngleUnit2>
    operator Pose2<LengthUnit2, AngleUnit2>() const
    {
        return Pose2<LengthUnit2, AngleUnit2>{ x(), y(), yaw() };
    }

    template<typename LengthUnit2, typename AngleUnit2>
    operator Pose3<LengthUnit2, AngleUnit2>() const
    {
        return Pose3<LengthUnit2, AngleUnit2>{ { x(), y(), z() }, { yaw(), pitch(), roll() } };
    }

    Vector3<LengthUnit> &position() { return std::get<0>(*this); }
    const Vector3<LengthUnit> &position() const { return std::get<0>(*this); }
    LengthUnit &x() { return std::get<0>(*this)[0]; }
    const LengthUnit &x() const { return std::get<0>(*this)[0]; }
    LengthUnit &y() { return std::get<0>(*this)[1]; }
    const LengthUnit &y() const { return std::get<0>(*this)[1]; }
    LengthUnit &z() { return std::get<0>(*this)[2]; }
    const LengthUnit &z() const { return std::get<0>(*this)[2]; }

    std::array<AngleUnit, 3> &attitude() { return std::get<1>(*this); }
    const std::array<AngleUnit, 3> &attitude() const { return std::get<1>(*this); }
    AngleUnit &yaw() { return std::get<1>(*this)[0]; }
    const AngleUnit &yaw() const { return std::get<1>(*this)[0]; }
    AngleUnit &pitch() { return std::get<1>(*this)[1]; }
    const AngleUnit &pitch() const { return std::get<1>(*this)[1]; }
    AngleUnit &roll() { return std::get<1>(*this)[2]; }
    const AngleUnit &roll() const { return std::get<1>(*this)[2]; }
};

//! Converts the input array to a unit-type of OutputUnit
template<typename OutputUnit, typename ArrayType>
inline constexpr std::array<OutputUnit, 3>
convertUnitArray(const ArrayType &values)
{
    return { static_cast<OutputUnit>(values[0]),
             static_cast<OutputUnit>(values[1]),
             static_cast<OutputUnit>(values[2]) };
}
} // BoBRobotics
