// Second translation unit of the multi-TU smoke target (see multi_tu_main.cpp).
#include <etl.h>

struct Sample {
    long value;
    unsigned char tag;
};

Sample* makeSample(long value) {
    return new Sample{value, 0};
}
