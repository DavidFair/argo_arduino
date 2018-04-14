#ifndef SPEED_HPP_
#define SPEED_HPP_

#include <stdint.h>

#include "Distance.hpp"
#include "Time.hpp"

namespace Libs {

class Speed {
public:
  constexpr Speed() : m_distance(), m_time(1_s), m_isOneSecond(true) {}

  constexpr Speed(Distance length, Time time)
      : m_distance(length), m_time(time), m_isOneSecond(m_time == 1_s) {}

  constexpr bool operator<=(const Speed &other) const {
    return (m_distance <= other.m_distance) && (m_time <= other.m_time);
  }

  constexpr Speed operator-(const Speed &other) const {
    // We have to do this all in one return statement for
    // c++-11 support.
    return Speed(getUnitDistance() - other.getUnitDistance(), 1_s);
  }

  constexpr bool operator==(const Speed &other) const {
    return m_distance == other.m_distance && m_time == other.m_time;
  }

  constexpr Distance getUnitDistance() const {
    // Have to do this in one return statement for C++11 constexpr support
    return m_isOneSecond ? m_distance
                         : Distance(m_distance.native() / m_time.seconds());
  }

private:
  Distance m_distance;
  Time m_time;
  bool m_isOneSecond;
};

} // namespace Libs

#endif // SPEED_HPP_