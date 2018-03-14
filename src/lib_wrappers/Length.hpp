#ifndef LENGTH_HPP_
#define LENGTH_HPP_

#include <stdint.h>

namespace Libs {

class Length {
public:
  explicit constexpr Length(int32_t nanometers) : m_nanometers(nanometers) {}

  explicit constexpr Length(double meters, int32_t millimeters)
      : m_nanometers(meters * meterToNanoMeter) {
    m_nanometers += millimeters * millimeterToNanoMeter;
  }

  constexpr Length operator+(const Length &other) {
    int32_t totalMillis =
        (other.m_nanometers + m_nanometers) / millimeterToNanoMeter;
    return Length(0, totalMillis);
  }

  constexpr int32_t getNanoMeters() const { return m_nanometers; }

  constexpr int32_t getMilliMeters() const {
    return m_nanometers / millimeterToNanoMeter;
  }
  constexpr double getMeters() const {
    double val = m_nanometers / (double)meterToNanoMeter;
    return val;
  }

private:
  const int32_t meterToNanoMeter = 1e+09;
  const int32_t millimeterToNanoMeter = 1e+06;

  // Internally store this as micro meters as we have the space
  int32_t m_nanometers{0};
}; // namespace Libs

constexpr Length operator"" _m(long double meters) {
  // Has to be a long double as per the standard
  return Length(meters, 0);
}

constexpr Length operator"" _mm(unsigned long long int millimeters) {
  return Length(0, millimeters);
}

} // namespace Libs

#endif // LENGTH_HPP_