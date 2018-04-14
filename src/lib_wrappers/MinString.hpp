#ifndef MIN_STRING_HPP_
#define MIN_STRING_HPP_

#include "cstring_wrapper.hpp"

namespace Libs {

class MinString {
public:
  constexpr MinString(const char *string)
      : m_string(string), m_length(strlen(string)) {}

  constexpr bool equalsCString(const char *cStr) const {
    return (strncmp(cStr, m_string, m_length) == 0);
  }

  constexpr const char *str() const { return m_string; }
  constexpr uint8_t length() const { return m_length; }

private:
  const char *m_string;
  uint8_t m_length;
};
} // namespace Libs

#endif // MIN_STRING_HPP_