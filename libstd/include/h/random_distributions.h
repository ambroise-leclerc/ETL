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
#include <libstd/include/limits>
#include <libstd/include/type_traits>

namespace ETLSTD {

template <typename IntType = int> struct uniform_int_distribution {
    using result_type = IntType;
    struct param_type {
        using distribution_type = uniform_int_distribution;

        explicit param_type(IntType a = 0, IntType b = numeric_limits<IntType>::max()) : a_(a), b_(b) {}
        result_type a() const { return a_; }
        result_type b() const { return b_; }
        friend bool operator==(const param_type &x, const param_type &y) { return x.a_ == y.a_ && x.b_ == y.b_; }
        friend bool operator!=(const param_type &x, const param_type &y) { return !(x == y); }

    private:
        IntType a_, b_;
    };

    uniform_int_distribution() : uniform_int_distribution(0) {}
    explicit uniform_int_distribution(IntType a, IntType b = numeric_limits<IntType>::max()) : param_(a, b) {}
    explicit uniform_int_distribution(const param_type &parm) : param_(parm) {}
    void reset() {}

    template <typename UniformRandomNumberGenerator> result_type operator()(UniformRandomNumberGenerator &g) {
        return (*this)(g, param_);
    }

    // Modulo-based mapping: simple and adequate for an embedded 8/32-bit
    // target, at the cost of a small bias when the generator's range isn't a
    // multiple of the requested span. Not suitable for cryptographic use.
    template <typename UniformRandomNumberGenerator>
    result_type operator()(UniformRandomNumberGenerator &g, const param_type &parm) {
        using UResult = typename UniformRandomNumberGenerator::result_type;
        using UInt = make_unsigned_t<IntType>;
        // static_cast to an unsigned type wraps modulo 2^N (well-defined), which correctly
        // biases signed a()/b() (including negative bounds) into an unsigned span.
        const UInt range = static_cast<UInt>(parm.b()) - static_cast<UInt>(parm.a());
        // Normalize the generator's output onto a zero-based range before applying modulo:
        // g() is only guaranteed to lie within [g.min(), g.max()], not [0, g.max()].
        const UResult generated = static_cast<UResult>(g()) - static_cast<UResult>(UniformRandomNumberGenerator::min());
        if (range == numeric_limits<UInt>::max())
            return static_cast<result_type>(static_cast<UInt>(generated));
        const UInt span = range + 1;
        const UInt drawn = static_cast<UInt>(generated % static_cast<UResult>(span));
        return static_cast<result_type>(static_cast<UInt>(parm.a()) + drawn);
    }

    result_type a() const { return param_.a(); }
    result_type b() const { return param_.b(); }

    param_type param() const { return param_; }
    void param(const param_type &parm) { param_ = parm; }

    result_type min() const { return this->a(); }
    result_type max() const { return this->b(); }

    friend bool operator==(const uniform_int_distribution &x, const uniform_int_distribution &y) {
        return x.param_ == y.param_;
    }
    friend bool operator!=(const uniform_int_distribution &x, const uniform_int_distribution &y) { return !(x == y); }

private:
    param_type param_;
};

} // namespace ETLSTD
