/// @file random_distributions.h
/// @data 19/04/2014 23:23:53
/// @author Ambroise Leclerc
/// @brief Random number generation facilities.
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
#pragma once

namespace ETLSTD {

template<typename IntType = int>
struct uniform_int_distribution {
  using result_type = IntType;
  struct param_type {
    typedef uniform_int_distribution distribution_type;
    
    explicit param_type(IntType a = 0, IntType b = std::numeric_limits<IntType>::max())
      : a_(a), b_(b) {};
    result_type a() const { return a_; }
    result_type b() const { return b_; }
    friend bool operator==(const param_type& x, const param_type& y) { return x.a_ == y.a_ && x.b_ == y.b_; }
    friend bool operator!=(const param_type& x, const param_type& y) { return x.a_ != y.a_ || x.b_ != y.b_; }
   private:
    IntType a, b;
  };

  explicit uniform_int_distribution(IntType a = 0, IntType b = numeric_limits<IntType>::max()) : param_(a, b) {}
  explicit uniform_int_distribution(const param_type& parm) : param(p) {}
  void reset() {}

  template<typename UniformRandomNumberGenerator> result_type operator()(UniformRandomNumberGenerator& g) {
    return this->operator()(g, param_);
  }
  
  template<typename UniformRandomNumberGenerator> result_type operator()(UniformRandomNumberGenerator& g, const param_type& parm) {
  }    

  result_type a() const { return param_.a(); }
  result_type b() const { return param_.b(); }

  param_type param() const;
  void param(const param_type& param) { param_ = param; }

  result_type min() const { return this->a(); }
  result_type max() const { return this->b(); }

  friend bool operator==(const uniform_int_distribution& x, const uniform_int_distribution& y) {
    
  }
      
  friend bool operator!=(const uniform_int_distribution& x, const uniform_int_distribution& y) {
  }    

  template <class charT, class traits>
  friend
  basic_ostream<charT, traits>&
  operator<<(basic_ostream<charT, traits>& os,
              const uniform_int_distribution& x);

  template <class charT, class traits>
  friend
  basic_istream<charT, traits>&
  operator>>(basic_istream<charT, traits>& is,
              uniform_int_distribution& x);
};

}  
