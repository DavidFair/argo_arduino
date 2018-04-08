#ifndef LENGTH_HPP_
#define LENGTH_HPP_

#include <stdint.h>

namespace Libs
{

class Distance
{
public:
  constexpr Distance() : m_micrometers(0) {}

  explicit constexpr Distance(int32_t micrometers)
      : m_micrometers(micrometers) {}

  // Have to do this all in initialiser for avr-gcc 4.8.x
  explicit constexpr Distance(double meters, int32_t millimeters)
      : m_micrometers((meters * METER_TO_MICRO) +
                      (millimeters * MILLI_TO_MICRO)) {}

  constexpr bool operator==(const Distance &other) const
  {
    return m_micrometers == other.m_micrometers;
  }

  constexpr Distance operator+(const Distance &other) const
  {
    return Distance(m_micrometers + other.m_micrometers);
  }

  constexpr Distance operator-(const Distance &other) const
  {
    return Distance(m_micrometers - other.m_micrometers);
  }

  constexpr Distance operator*(const int32_t multi) const
  {
    return Distance(m_micrometers * multi);
  }

  constexpr Distance operator/(const int32_t div) const
  {
    return Distance(m_micrometers / div);
  }

  constexpr int32_t micrometers() const { return m_micrometers; }

  constexpr int32_t millimeters() const
  {
    return m_micrometers / MILLI_TO_MICRO;
  }
  constexpr float meters() const
  {
    return m_micrometers / (float)METER_TO_MICRO;
  }

  constexpr int32_t native() const { return m_micrometers; }

private:
  static const int32_t METER_TO_MICRO = 1e+06;
  static const int32_t MILLI_TO_MICRO = 1e+03;

  // Internally store this as micrometers so we don't lose precision
  int32_t m_micrometers{0};
}; // namespace Libs

constexpr Distance operator"" _m(unsigned long long meters)
{
  return Distance(meters, 0);
}

constexpr Distance operator"" _m(long double meters)
{
  // Has to be a long double as per the standard
  return Distance(meters, 0);
}

} // namespace Libs

#endif // LENGTH_HPP_