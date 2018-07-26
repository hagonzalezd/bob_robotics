#pragma once

// Standard C++ includes
#include <array>

// Third-party includes
#include "../third_party/units.h"

namespace BoBRobotics {
//! A generic template for unit arrays
template<class T>
using Vector3 = std::array<T, 3>;

//! Returns a triple of unit-type objects.
template<class OutputUnit, class ArrayType>
inline constexpr Vector3<OutputUnit>
makeUnitArray(const ArrayType &values)
{
    return { static_cast<OutputUnit>(values[0]),
             static_cast<OutputUnit>(values[1]),
             static_cast<OutputUnit>(values[2]) };
}
} // BoBRobotics