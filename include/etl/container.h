
namespace etl {
    template<typename T, uint8_t N>
    class CircularBuffer {
    public:
        CircularBuffer() : readIndex(0), writeIndex(0) {
        }

        void push_back(T elem) {
            buffer[writeIndex] = elem;
            writeIndex = (writeIndex + 1) % N;

        }

        T pop_front() {
            T value = buffer[readIndex];
            readIndex = (readIndex + 1) % N;
            return value;
        }

        T front() {
            T value = buffer[readIndex];
            return value;
        }

        T back() {
            auto index = 0;
            if (writeIndex == 0) {
                index = N - 1;
            }
            else {
                index = writeIndex - 1;
            }
            T value = buffer[index];
            return value;
        }

        uint8_t size() {
            return writeIndex >= readIndex ? writeIndex - readIndex : writeIndex + N - readIndex;
        }

        template<typename... Args>
        void emplace_back(Args&& ... args) {
            std::allocator<T> all;
            all.construct(&buffer[writeIndex], std::forward<Args>(args)...);
            writeIndex = (writeIndex + 1) % N;
        }


    private:
        std::array<T, N> buffer;
        uint8_t readIndex, writeIndex;
    };

}