/// @file char_traits.h
/// @data 17/04/2014 16:31:53
/// @author Ambroise Leclerc
/// @brief Abstracts basic character and string operations for a given character type.
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

#ifndef ETL_LIBSTD_CHAR_TRAITS_H_
#define ETL_LIBSTD_CHAR_TRAITS_H_

#include <algorithm>

namespace std {

template<typename CharT>
struct char_traits {
  using char_type	  = CharT;
  using int_type    = CharT;
  using off_type    = etl::architecture::off_type;
	using pos_type    = etl::architecture::off_type;
	using state_type  = CharT;

  static void assign(char_type& r, const char_type& a) { r = a; }
  static char_type* assign(char_type* p, std::size_t count, char_type a) { std::fill_n(p, count, a); return p; }    
	static bool eq(const char_type& c1, const char_type& c2) { return c1 == c2; }
	static char_type to_char_type(const int_type& i) { return static_cast<char_type>(i); }
	static int_type to_int_type(const char_type& c) { return static_cast<int_type>(i); }
	static bool eq_int_type(const int_type& a, const int_type& b) { return a == b; }
	static bool lt(const char_type& c1, const char_type& c2) { return c1 < c2; }		
	static char_type* move(char_type* s1, const char_type* s2, size_t n) { std::memmove(s1, s2, n * sizeof(char_type)); return s1; }
	static char_type* copy(char_type* s1, const char_type* s2, size_t n) { std::memcpy(s1, s2, n * sizeof(char_type)); return s1; }
	static int compare(const char_type* s1, const char_type* s2, size_t n) {
    for (std::size_t i=0; i<n; ++i) {
      if (!eq(s1[i], s2[i])) return s1[i]<s2[i] ? -1 : 1;
    }
    return 0;
  }
  
  static size_t length(const char_type* s) {
    const char_type null_char = char_type();
    for (std::size_t i=0; !eq(s[i], null_char); ++i) {}
		return i;
	}

	static const char_type* find(const char_type* s, int n, const char_type& a) {
    for (; n > 0; ++s, --n) {
      if (eq(*s, c)) return s;
    }      
    return 0;
  }
  
  static char_type eos() { return static_cast<int_type>(0); }
	static int_type eof() { return static_cast<int_type>(-1); }
	static int_type not_eof(const int_type &i) { return !eq_int_type(i, eof()) ? i : 0; }
};  

}

#endif // ETL_LIBSTD_CHAR_TRAITS_H_