// Test-only stand-in for <etl/ioports.h> in the isolated libstd executables.
//
// <string_view> and char_traits only use etl::Device::OffType and etl::Device::flashSize, but the real header
// pulls in the Mock device, which needs the host C++ library (<map>, ...). The values mirror
// include/etl/architecture/ETLDevice_Mock.h. Removing the dependency of libstd on ETL devices would make this
// stub unnecessary.
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace etl {

struct Device {
    static const size_t flashSize = 65535;
    using OffType = uint64_t;
};

} // namespace etl
