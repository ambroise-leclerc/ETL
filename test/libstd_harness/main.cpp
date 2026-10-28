#include "harness.h"

// extern "C": with -ffreestanding Clang does not treat main specially and would mangle it
extern "C" int main() { return etl_harness::run(); }
