// Compile only: these operations need exception/RTTI ABI symbols when linked.
// The exported full profile must accept them without claiming to provide that runtime.
int full_exception_probe(int value) {
    try {
        if (value) {
            throw value;
        }
        return 0;
    } catch (int caught) { return caught; }
}

struct FullProbeBase {
    virtual ~FullProbeBase() = default;
};

struct FullProbeDerived : FullProbeBase {};

FullProbeDerived *full_rtti_probe(FullProbeBase *value) { return dynamic_cast<FullProbeDerived *>(value); }
