#ifndef LENGTH_HPP_
#define LENGTH_HPP_

#include <stdint.h>

namespace Libs {

class Length {
public:
  explicit constexpr Length(int32_t micrometers) : m_micrometers(micrometers) {}

  // Have to do this all in initialiser for avr-gcc 4.8.x
  explicit constexpr Length(double meters, int32_t millimeters)
      : m_micrometers((meters * METER_TO_MICRO) +
                      (millimeters * MILLI_TO_MICRO)) {}

  constexpr bool operator==(const Length &other) const {
    return m_micrometers == other.m_micrometers;
  }

  constexpr Length operator+(const Length &other) const {
    return Length(m_micrometers + other.m_micrometers);
  }

  constexpr Length operator*(const int32_t multi) const {
    return Length(m_micrometers * multi);
  }

  constexpr Length operator/(const int32_t div) const {
    return Length(m_micrometers / div);
  }

  constexpr int32_t getMicroMeters() const { return m_micrometers; }

  constexpr int32_t getMilliMeters() const {
    return m_micrometers / MILLI_TO_MICRO;
  }
  constexpr double getMeters() const {
    return m_micrometers / (double)METER_TO_MICRO;
  }

private:
  static const int32_t METER_TO_MICRO = 1e+06;
  static const int32_t MILLI_TO_MICRO = 1e+03;

  // Internally store this as micrometers so we don't lose precision
  int32_t m_micrometers{0};
}; // namespace Libs

constexpr Length operator"" _m(long double meters) {
  // Has to be a long double as per the standard
  return Length(meters, 0);
}

} // namespace Libs

#endif // LENGTH_HPP_