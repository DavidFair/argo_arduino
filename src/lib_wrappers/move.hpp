#ifndef MOVE_HPP_
#define MOVE_HPP_

namespace Libs {

// --------------------------------------
// Minimal remove reference by Jonathan Wakely
// See https://stackoverflow.com/a/38524997 (Accessed 2017-03-04)
template <class T> struct remove_reference { typedef T type; };

template <class T> struct remove_reference<T &> { typedef T type; };

template <class T> struct remove_reference<T &&> { typedef T type; };

// -------------------------------------

// Now we can use templating with remove_reference to implement move
template <typename T>
constexpr typename remove_reference<T>::type &&move(T &&obj) {
  return static_cast<typename remove_reference<T>::type &&>(obj);
}

} // namespace Libs

#endif // MOVE_HPP_