#include <array>
#include <utility>

extern "C" int c_value(void);

int main() {
    std::array<int, 2> values{{1, 2}};
    std::swap(values[0], values[1]);
    return values[0] == c_value() ? 0 : 1;
}
