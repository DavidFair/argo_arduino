#ifndef LENGTH_HPP_
#define LENGTH_HPP_

#include <stdint.h>

namespace Libs {

class Length {
public:
  explicit constexpr Length(double meters, int32_t millimeters)
      : m_millimeters(millimeters) {
    m_millimeters += convertMeterToMilliMeter(meters);
  }

  constexpr Length operator+(const Length &other) {
    int32_t total = other.m_millimeters + m_millimeters;
    return Length(0, total);
  }

  constexpr int32_t getMilliMeters() const { return m_millimeters; }

private:
  constexpr int32_t convertMeterToMilliMeter(double meters) {
    return meters * 1000;
  }

  int32_t m_millimeters{0};
};

constexpr Length operator"" _m(long double meters) {
  // Has to be a long double as per the standard
  return Length(meters, 0);
}

constexpr Length operator"" _mm(unsigned long long int millimeters) {
  return Length(0, millimeters);
}

} // namespace Libs

#endif // LENGTH_HPP_