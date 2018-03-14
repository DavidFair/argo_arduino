#ifndef SPEED_HPP_
#define SPEED_HPP_

#include <stdint.h>

#include "Length.hpp"

namespace Libs {

class Speed {
public:
  constexpr Speed(Length length, unsigned long millisTime)
      : m_distance(length), m_millis(millisTime) {}

  constexpr bool operator==(Speed other) const {
    return m_distance == other.m_distance && m_millis == other.m_millis;
  }

  constexpr int32_t getMilliMetersPerSecond() const {
    return m_distance.getMilliMeters() / (m_millis / (double)MILLIS_PER_SEC);
  }

private:
  static const int MILLIS_PER_SEC = 1000;

  Length m_distance;
  int32_t m_millis;
};

} // namespace Libs

#endif // SPEED_HPP_