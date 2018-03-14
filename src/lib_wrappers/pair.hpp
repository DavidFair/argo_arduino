#ifndef PAIR_HPP_
#define PAIR_HPP_

namespace Libs {

template <class T1, class T2> struct pair {
  T1 first;
  T2 second;

  pair(const T1 &inFirst, const T2 &inSecond)
      : first(inFirst), second(inSecond) {}

  pair(pair &&other) : first(other.first), second(other.second) {}
};

} // namespace Libs

#endif // PAIR_HPP_