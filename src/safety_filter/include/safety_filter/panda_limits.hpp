#pragma once
#include <array>
namespace safety_filter {
// FER/Panda, not FR3. Specification ceilings, not commissioning parameters.
inline constexpr std::array<double, 7> kJointLower{-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973};
inline constexpr std::array<double, 7> kJointUpper{2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973};
inline constexpr std::array<double, 7> kJointVelocity{2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61};
inline constexpr std::array<double, 7> kJointAcceleration{15, 7.5, 10, 12.5, 15, 20, 20};
inline constexpr std::array<double, 7> kJointTorque{87, 87, 87, 87, 12, 12, 12};
inline constexpr std::array<double, 7> kHome{0, -0.7854, 0, -2.3562, 0, 1.5708, 0.7854};
}
