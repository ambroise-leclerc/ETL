#include <array>
#include <cstdint>
#include <string_view>
#include <type_traits>
#include <utility>

static_assert(std::is_integral_v<unsigned char>);
volatile unsigned char input;
volatile unsigned char result;

int main() {
    std::array<unsigned char, 2> values{{input, 7}};
    std::swap(values[0], values[1]);
    constexpr std::string_view name("minimal");
    result = values[1] + name.size();
    return values[0] == 7 ? 0 : 1;
}
