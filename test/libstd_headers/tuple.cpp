#include <tuple>

int main() {
    auto values = std::make_tuple(1, 2.0);
    return std::get<0>(values) == 1 ? 0 : 1;
}
