// ETL included from two translation units (see multi_tu_other.cpp): the program must link
// without duplicate definitions, and both units share one free store.
#include <etl.h>

struct Sample {
    long value;
    unsigned char tag;
};

Sample* makeSample(long value);  // allocates in the other translation unit

int main() {
    Sample* local = new Sample{1, 2};
    Sample* remote = makeSample(3);
    int* values = new int[4];
    values[0] = static_cast<int>(etl::FreeStore::GetMemorySize());

    alignas(Sample) unsigned char storage[sizeof(Sample)];
    Sample* placed = new (storage) Sample{4, 5};

    long result = local->value + remote->value + placed->value + values[0];
    delete local;   // sized delete
    delete remote;  // allocated by the other translation unit
    delete[] values;
    return result != 0 ? 0 : 1;
}
