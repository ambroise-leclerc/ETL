// The current allocation implementation uses ETL's device-specific FreeStore.
#include <cstdint>
#include <etl.h>
#include <memory>
#include <new>

volatile unsigned char input;
unsigned char *volatile allocated;
volatile unsigned char result;

int main() {
    std::unique_ptr<unsigned char> value(new unsigned char(input));
    allocated = value.get();
    result = *value;
    value.reset();
    allocated = nullptr;
    return 0;
}
