#include <array>
#include <utility>

int main() {
    std::array<int, 2> values{{1, 2}};
    std::swap(values[0], values[1]);
    return values[0] == 2 ? 0 : 1;
}
