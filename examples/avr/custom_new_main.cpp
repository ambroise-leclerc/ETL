// A program replacing the global allocation functions (see custom_new_allocator.cpp): its
// definitions must take precedence over the weak ones provided by ETL.
#include <etl.h>

// volatile: keeps the compiler from eliding the allocation ([expr.new])
long* volatile value;

int main() {
    value = new long{42};
    long result = *value;
    delete value;
    return result == 42 ? 0 : 1;
}
