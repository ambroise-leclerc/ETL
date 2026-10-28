#include <memory>

int main() {
    auto value = std::make_unique<int>(42);
    return *value == 42 ? 0 : 1;
}
