#ifndef LENGTH_HPP_
#define LENGTH_HPP_

#include <stdint.h>

namespace Libs {

class Distance {
public:
  constexpr Distance() : m_millimeters(0) {}

  explicit constexpr Distance(int32_t millimeters)
      : m_millimeters(millimeters) {}

  explicit constexpr Distance(double meters, int32_t millimeters)
      : m_millimeters((meters * METER_TO_MILLI) + millimeters) {}

  constexpr bool operator<=(const Distance &other) const {
    return m_millimeters <= other.m_millimeters;
  }

  constexpr bool operator>=(const Distance &other) const {
    return m_millimeters >= other.m_millimeters;
  }

  constexpr bool operator==(const Distance &other) const {
    return m_millimeters == other.m_millimeters;
  }

  constexpr Distance operator+(const Distance &other) const {
    return Distance(m_millimeters + other.m_millimeters);
  }

  constexpr Distance operator-() const { return Distance(-m_millimeters); }

  constexpr Distance operator-(const Distance &other) const {
    return Distance(m_millimeters - other.m_millimeters);
  }

  constexpr Distance operator*(const int32_t multi) const {
    return Distance(m_millimeters * multi);
  }

  constexpr Distance operator/(const int32_t div) const {
    return Distance(m_millimeters / div);
  }

  constexpr int32_t millimeters() const { return m_millimeters; }

  constexpr float meters() const {
    return m_millimeters / (float)METER_TO_MILLI;
  }

  constexpr int32_t native() const { return m_millimeters; }

private:
  static const int32_t METER_TO_MILLI = 1e+03;

  int32_t m_millimeters{0};
}; // namespace Libs

constexpr Distance operator"" _m(unsigned long long meters) {
  return Distance(meters, 0);
}

constexpr Distance operator"" _m(long double meters) {
  // Has to be a long double as per the standard
  return Distance(meters, 0);
}

} // namespace Libs

#endif // LENGTH_HPP_