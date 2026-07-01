/// @file random_generators.h
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
#include <libstd/include/cstdint>
#include <libstd/include/limits>

namespace ETLSTD {

/// xorshift32 pseudo-random generator. Not a std::-conforming engine (no
/// standard algorithm/state-size guarantees), but satisfies the
/// UniformRandomBitGenerator interface used by libstd distributions and is
/// cheap enough for 8-bit targets.
class xorshift32_engine {
public:
    using result_type = uint32_t;

    explicit xorshift32_engine(result_type seed = 1u) : state_(seed ? seed : 1u) {}

    void seed(result_type s) { state_ = s ? s : 1u; }

    result_type operator()() {
        state_ ^= state_ << 13;
        state_ ^= state_ >> 17;
        state_ ^= state_ << 5;
        return state_;
    }

    static constexpr result_type min() { return 0u; }
    static constexpr result_type max() { return numeric_limits<result_type>::max(); }

private:
    result_type state_;
};

using default_random_engine = xorshift32_engine;

} // namespace ETLSTD
