/// @file unique_ptr.h
/// @data 06/03/2014 22:12:53
/// @author Ambroise Leclerc
/// @brief A unique_ptr smart pointer.
//
// Copyright (c) 2014, Ambroise Leclerc
//   All rights reserved.
//
//   Redistribution and use in source and binary forms, with or without
//   modification, are permitted provided that the following conditions are met:
//
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in
//     the documentation and/or other materials provided with the
//     distribution.
//   * Neither the name of the copyright holders nor the names of
//     contributors may be used to endorse or promote products derived
//     from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 'AS IS'
//  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
//  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
//  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
//  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
//  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
//  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
//  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//  POSSIBILITY OF SUCH DAMAGE.

#ifndef ETL_LIBSTD_UNIQUE_PTR_H
#define ETL_LIBSTD_UNIQUE_PTR_H
#include <utility>
#include <cstddef>
#include <h/default_delete.h>

namespace std {
  
template <class T, class D = default_delete<T>>
class unique_ptr {
 public:
  typedef T* pointer;
  typedef T element_type;
  typedef D deleter_type;

  pointer pointer_;
  deleter_type deleter_;
  
  /// Constructs an empty std::unique_ptr.
  constexpr unique_ptr() noexcept : pointer_(), deleter_() { }
  
  /// Constructs an emply std::unique_ptr.
  constexpr unique_ptr(decltype(nullptr)) noexcept : unique_ptr() { }
  
  /// Constructs a std::unique_ptr which owns p.
  /// @param p pointer
  explicit unique_ptr(pointer p) noexcept : pointer_(p), deleter_(deleter_type()) { }
  
  /// Constructs a unique_ptr by transferring ownership from source to *this.
  /// @param[in] source source pointer
  unique_ptr(unique_ptr&& source) noexcept
   : pointer_(source.release(), std::forward<deleter_type>(source.get_deleter())) { }
  
  /// Converting constructor from another type
  template <class T2, class D2>
  unique_ptr(unique_ptr<T2, D2>&& source) noexcept
   : pointer_(source.release(), std::forward<D2>(source.get_deleter())) { }

  // Disable copy from lvalue.
  unique_ptr(const unique_ptr&) = delete;
  unique_ptr& operator=(const unique_ptr&) = delete;
  
  /// Move assignment operator.
  /// @param[in] u the object to transfer ownership from.
  unique_ptr& operator=(unique_ptr&& source) noexcept {
    reset(source.release());
    get_deleter() = std::forward<deleter_type>(source.get_deleter());
    return *this;
  }

  /// Destructs the managed object.
  ~unique_ptr() noexcept {
    if (nullptr != pointer_) {
      get_deleter()(pointer_);
    }
    pointer_ = pointer();
  }

  
  /// Assignment from another type.
  /// @param[in] u the object to transfer ownership from.
  template <class T2, class D2>
  unique_ptr& operator=(unique_ptr<T2, D2>&& source) noexcept {
    reset(source.release());
    get_deleter() = std::forward<D2>(source.get_deleter());
  }
  
  /// Assignment to nullptr : same effect as calling reset().
  unique_ptr& operator=(nullptr_t) noexcept {
    reset();
    return *this;
  }

  /// Provides access to the managed object.
  /// @return the object owned by *this, i.e. *get().
  typename std::add_lvalue_reference<T>::type operator*() const {
    return *get();
  }
  
  /// Provides access to the managed object.
  /// @return the object owned by *this, i.e. get().
  pointer operator->() const noexcept {
    return get();
  }
  
  /// Returns a pointer to the managed object.
  /// @return pointer to the managed object or nullptr is no object is owned.
  pointer get() const noexcept {
    return pointer_;
  }
  
  /// Returns a reference to the stored deleter.
  /// @return The stored deleter object.
  deleter_type& get_deleter() noexcept {
    return deleter_;
  }
  
  /// Returns a reference to the stored deleter.
  /// @return The stored deleter object.
  const deleter_type& get_deleter() const noexcept {
    return deleter_;
  }
  
  /// Checks if an object is owned.
  /// @return true if an object is owned, false otherwise.
  explicit operator bool() const noexcept {
    return get() != pointer();
  }

  /// Returns a pointer to the managed object and releases the ownership.
  /// @return pointer to the managed object.
  pointer release() noexcept {
    pointer old_ptr = get();
    pointer_ = pointer();
    return old_ptr;
  }
  
  /// Replaces the managed object.
  void reset(pointer p = pointer()) noexcept {
    pointer old_ptr = pointer_;
    pointer_ = p;
    if (old_ptr != pointer()) {
      get_deleter()(old_ptr);
    }
  }
  
  /// Swaps the managed objects and associated deleters with another std::unique_ptr object.
  /// @param[in] other another std::unique_ptr to swap the managed object and the deleter with
  void swap(unique_ptr& other) noexcept {
    pointer old_ptr = pointer_;
    pointer_ = other.pointer_;
    other.pointer_ = old_ptr;
    deleter_type old_deleter = deleter_;
    deleter_ = other.deleter_;
    other.deleter_ = old_deleter;
  }
};

} // namespace std
#endif // ETL_LIBSTD_UNIQUE_PTR_H