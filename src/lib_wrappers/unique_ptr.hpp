#ifndef UNIQUE_PTR_HPP_
#define UNIQUE_PTR_HPP_

namespace Argo {

template <typename T> class unique_ptr {
public:
  unique_ptr(T *passedPtr) : ptr(passedPtr) {}
  ~unique_ptr() { delete ptr; }

  unique_ptr(const unique_ptr &) = delete;
  unique_ptr &operator=(const unique_ptr &a) = delete;

  unique_ptr(unique_ptr &&other) {
    ptr = other.ptr;
    other.ptr = nullptr;
  }

  T &operator*() { return ptr; }

  T *operator->() { return ptr; }

private:
  T *ptr;
};

} // namespace Argo

#endif // UNIQUE_PTR_HPP_