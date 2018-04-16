#ifndef MIN_STRING_HPP_
#define MIN_STRING_HPP_

#include <stdint.h>

#include "cstring_wrapper.hpp"

namespace Libs {

// Takes a pointer to a string literal to wrap some functions in
// better named functions
class MinString {
public:
  MinString(const char *string) : m_string(string), m_length(strlen(string)) {}

  bool isPresentInString(const char *cStr) const {
    return (strncmp(cStr, m_string, m_length) == 0);
  }

  const char *str() const { return m_string; }
  uint8_t length() const { return m_length; }

private:
  const char *m_string;
  uint8_t m_length;
};
} // namespace Libs

#endif // MIN_STRING_HPP_