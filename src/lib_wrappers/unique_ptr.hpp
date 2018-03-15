#ifndef UNIQUE_PTR_HPP_
#define UNIQUE_PTR_HPP_

namespace Libs {

template <typename T> class unique_ptr {
public:
  constexpr unique_ptr(T *passedPtr) : ptr(passedPtr) {}
  ~unique_ptr() { delete ptr; }

  // Copy
  unique_ptr(const unique_ptr &) = delete;
  unique_ptr &operator=(const unique_ptr &a) = delete;

  // Move
  constexpr unique_ptr(unique_ptr &&other) {
    ptr = other.ptr;
    other.ptr = nullptr;
  }

  unique_ptr &operator=(unique_ptr &&other) {
    if (this != &other) {
      ptr = other.ptr;
      other.ptr = nullptr;
    }
    return *this;
  }

  constexpr T *get() const { return ptr; }

  constexpr void reset(T *newPtr) {
    delete ptr;
    ptr = newPtr;
  }

  constexpr T *release() {
    T *ptrToReturn = ptr;
    ptr = nullptr;
    return ptrToReturn;
  }

  constexpr T &operator*() { return *ptr; }

  constexpr T *operator->() { return ptr; }

private:
  T *ptr;
};

} // namespace Libs

#endif // UNIQUE_PTR_HPP_