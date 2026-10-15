// A program replacing the scalar allocation functions (see custom_new_allocator.cpp): its
// definitions must take precedence over the weak ones provided by ETL, while the array forms
// it does not replace keep allocating from, and releasing to, the ETL free store.
#include <etl.h>

struct Tracked {
    ~Tracked() { ++destroyed; }  // non-trivial: delete[] uses the sized form
    static inline unsigned char destroyed = 0;
};

// volatile: keeps the compiler from eliding the allocations ([expr.new])
long* volatile value;
long* volatile values;
Tracked* volatile tracked;

int main() {
    value = new long{42};
    values = new long[3];
    tracked = new Tracked[2];
    long result = *value + values[0] * 0;
    delete value;
    delete[] values;
    delete[] tracked;
    return result == 42 && Tracked::destroyed == 2 ? 0 : 1;
}
