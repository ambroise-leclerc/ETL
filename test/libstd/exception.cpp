/// @file test/libstd/exception.cpp
/// @date 01/07/2026
/// @author Ambroise Leclerc
/// @brief Tests for <exception> and <stdexcept>
//
// Copyright (c) 2026, Ambroise Leclerc
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
#include <catch.hpp>
#include <cstring>

#define __Mock_Mock__
#define ETLSTD etlstd

#include <libstd/include/stdexcept>

using namespace ETLSTD;

SCENARIO("std::exception diagnostics", "[libstd][exception]") {
    exception base;
    REQUIRE(base.what() != nullptr);
    REQUIRE(base.what()[0] == '\0');

    bad_exception bad;
    REQUIRE(bad.what() != nullptr);

    logic_error logic("logic");
    REQUIRE(std::strcmp(logic.what(), "logic") == 0);

    out_of_range range("range");
    REQUIRE(std::strcmp(range.what(), "range") == 0);

    runtime_error runtime("runtime");
    REQUIRE(std::strcmp(runtime.what(), "runtime") == 0);
}
