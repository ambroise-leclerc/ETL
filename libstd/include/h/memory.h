/// @file memory.h
/// @data 05/03/2014 18:23:53
/// @author Ambroise Leclerc
/// @brief
//
// Embedded Template Library
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

#ifndef ETL_LIBSTD_MEMORY_H_
#define ETL_LIBSTD_MEMORY_H_
#include <type_traits>
#include <utility>

namespace std {
namespace etlHelper {
	// addressof helpers taken from Boost library
	template<class T>
	struct addressof_ref {
	  T & v_; 
	  addressof_ref( T & v ): v_( v ) {}
	  operator T& () const { return v_; }

	 private:
	  addressof_ref & operator=(const addressof_ref &);
	};

	template<class T>
	struct addressof_impl
	{
	  static T * f( T & v, long ) {
		return reinterpret_cast<T*>(
		  &const_cast<char&>(reinterpret_cast<const volatile char &>(v)));
	  }

	  static T * f( T * v, int ) { return v; }
	};
} // namespace etlHelper 

  
template<typename T>
class allocator {
 public:
  using value_type      = T;
  using pointer         = T*;
  using const_pointer   = const T*;
  using reference       = T&;
  using const_reference = const T&;
  using size_type       = std::size_t;
  using difference_type = std::ptrdiff_t;
  using propagate_on_container_move_assignment = std::true_type;
  
  template<typename T2> struct rebind { typedef allocator<T2> other; };
  
  allocator() noexcept {}
  allocator(const allocator&) noexcept {};
  template<typename T2> allocator(const allocator<T2>&) noexcept {};
  
  /// Constructs an object of type T in allocated uninitialized storage using placement new.
  template<typename T2, typename... Args>
  static void construct(T2* p, Args&&... value) {
    ::new((void*)p) T2(std::forward<Args>(value)...);
  }    

  // destroy helper to invoke destructor explicitly.
  static void destroy(const_reference t) {
    t.~T(); // T must support non-throwing destructor
  }
};

template<typename T>
T* addressof(T& arg) {
  return etlHelper::addressof_impl<T>::f(etlHelper::addressof_ref<T>(arg), 0);
}  

} // namespace std
#endif // ETL_LIBSTD_MEMORY_H_