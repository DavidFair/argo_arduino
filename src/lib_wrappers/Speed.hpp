#ifndef SPEED_HPP_
#define SPEED_HPP_

#include <stdint.h>

#include "Distance.hpp"
#include "Time.hpp"

namespace Libs {

class Speed {
public:
  constexpr Speed() : m_distance(), m_time(1_s) {}

  constexpr Speed(Distance length, Time time)
      : m_distance(length), m_time(time) {}

  constexpr bool operator==(Speed other) const {
    return m_distance == other.m_distance && m_time == other.m_time;
  }

  constexpr int32_t getMilliMetersPerSecond() const {
    return m_distance.getMilliMeters() / m_time.seconds();
  }

private:
  Distance m_distance;
  Time m_time;
};

} // namespace Libs

#endif // SPEED_HPP_