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

  constexpr Speed operator-(const Speed &other) const {
    /* we can show that (d1.t1^-1) - (d2.t2^-1) is
     * equal to (d1.t2 - d2.t1) / (t2.t1) thereby saving
     * a division by turning it into a multiplication */

    // We have to do this all in one return statement for
    // c++-11 support.
    return Speed(Distance(m_distance * other.m_time.native() -
                          other.m_distance * m_time.native()),
                 Time(m_time.native() * other.m_time.native()));
  }

  constexpr bool operator==(const Speed &other) const {
    return m_distance == other.m_distance && m_time == other.m_time;
  }

  constexpr Distance getUnitDistance() const {
    return Distance(m_distance.native() / m_time.seconds());
  }

private:
  Distance m_distance;
  Time m_time;
};

} // namespace Libs

#endif // SPEED_HPP_