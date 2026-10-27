#include <array>

static_assert(std::array<int, 3>{}.size() == 3);

int main() {
    std::array<int, 3> values{1, 2, 3};
    return values[1] == 2 ? 0 : 1;
}
