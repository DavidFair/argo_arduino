#ifndef TIME_HPP_
#define TIME_HPP_

#include <stdint.h>

namespace Libs {

class Time {
public:
  constexpr Time() : m_millis(0) {}

  explicit constexpr Time(int32_t milliseconds) : m_millis(milliseconds) {}

  constexpr bool operator==(const Time &other) const {
    return m_millis == other.m_millis;
  }

  constexpr Time operator+(const Time &other) const {
    return Time(m_millis + other.m_millis);
  }

  constexpr double seconds() const { return m_millis / (double)MILLIS_PER_SEC; }

  constexpr int32_t millis() const { return m_millis; }

private:
  static const int16_t MILLIS_PER_SEC = 1000;

  int32_t m_millis;
};

constexpr Time operator"" _s(unsigned long long seconds) {
  return Time(seconds * 1000);
}

constexpr Time operator"" _s(long double seconds) {
  return Time(seconds * 1000);
}

constexpr Time operator"" _milliS(unsigned long long milliseconds) {
  return Time(milliseconds);
}

} // namespace Libs

#endif // TIME_HPP_