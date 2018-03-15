#ifndef UNIQUE_PTR_HPP_
#define UNIQUE_PTR_HPP_

namespace Libs {

template <typename T> class unique_ptr {
public:
  unique_ptr(T *passedPtr) : ptr(passedPtr) {}
  ~unique_ptr() { delete ptr; }

  // Copy
  unique_ptr(const unique_ptr &) = delete;
  unique_ptr &operator=(const unique_ptr &a) = delete;

  // Move
  unique_ptr(unique_ptr &&other) {
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

  T *get() const { return ptr; }

  void reset(T *newPtr) {
    delete ptr;
    ptr = newPtr;
  }

  T *release() {
    T *ptrToReturn = ptr;
    ptr = nullptr;
    return ptrToReturn;
  }

  T &operator*() { return *ptr; }

  T *operator->() { return ptr; }

private:
  T *ptr;
};

} // namespace Libs

#endif // UNIQUE_PTR_HPP_