// User replacement of the global allocation functions. It lives in its own translation unit:
// a translation unit including etl.h already contains ETL's (weak) definitions.
#include <stddef.h>

static unsigned char arena[16];
static unsigned char used;

void* operator new(size_t size) {
    void* block = &arena[used];
    used += static_cast<unsigned char>(size);
    return block;
}

void operator delete(void*) noexcept {}
void operator delete(void*, size_t) noexcept {}
