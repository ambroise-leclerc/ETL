/*
 * etl.h
 *
 * Created: 21/02/2014 23:06:23
 *  Author: Ambroise Leclerc
 */ 


#ifndef ETL_H_
#define ETL_H_


#include <stdlib.h>
#include <metautils.h>

namespace etl {

class FreeStore {
  public:
  /// Allocates size bytes of uninitialized storage.
  /// @param[in] size number of bytes to allocate
  /// @return Pointer to the beginning of allocated memory. The pointer must be
  /// deallocated with FreeStore::Deallocate.
  static void* Allocate(std::size_t size) {
    auto chunk_parser = reinterpret_cast<Chunk*>(free_chunk_);
    
    // 0 Setup the start of allocatable area (data_) to the first unallocated block
    while (chunk_parser->IsAllocated()) {
      chunk_parser = chunk_parser->NextChunk();
      free_chunk_ = reinterpret_cast<uint8_t*>(chunk_parser);
    }
    auto second_choice = chunk_parser;
    uint16_t second_choice_size = 0b1111111111111u;
    
    // 1 Find a free chunk with the same size as the requested allocation size.
    while (!chunk_parser->IsLastChunk()) {
      if (!chunk_parser->IsAllocated()) {
        if (chunk_parser->ChunkSize() == size) {
          return chunk_parser->AllocateData(size);
        }
        if (chunk_parser->ChunkSize() > (size+1)) {
          if (chunk_parser->ChunkSize() < second_choice_size) {
            second_choice = chunk_parser;
            second_choice_size = chunk_parser->ChunkSize();
          }
        }
      }
      chunk_parser = chunk_parser->NextChunk();
    }

    // 2 Free chunk with exact same size has not been found : We allocate the second choice
    if (second_choice->ChunkSize() < (size+2)) {
      second_choice = chunk_parser;
    }
    return second_choice->AllocateData(size);
  }

  /// Deallocates the memory space pointed to by ptr, which must have been
  /// returned by a previous call to FreeStore::Allocate(). Otherwise undefined
  /// behaviors occurs.
  /// @param[in] pointer to the memory to deallocate
  /// @return
  static void Deallocate(void* ptr) {
    auto deallocatable = reinterpret_cast<uint8_t*>(ptr);
    deallocatable -= CompileTimeOffsetOf(&Chunk::data);          // Point to chunk beginning
    if (deallocatable < free_chunk_) {                            // if chunk to deallocate is before the current first free chunk
      free_chunk_ = reinterpret_cast<uint8_t*>(deallocatable);    // then we define it's address as the new first free chunk
    }
    reinterpret_cast<Chunk*>(deallocatable)->Deallocate();
  }
  
  /// Returns 
  static std::size_t GetMemorySize() {
    return GetChunksSize(false);
  }

  static uint8_t GetMemoryFragmentation() {
    uint16_t holessize = GetChunksSize(true);
    uint16_t memsize = GetMemorySize();
    if (0 == memsize) return 0;
    return 100-((100 * memsize) / (memsize + holessize));
  }

  static std::size_t GetFreeMemory() {
    uint16_t stack_limit = 0;
    auto mem_spread = reinterpret_cast<std::size_t>(&stack_limit);
    Chunk* chunk_parser = reinterpret_cast<Chunk*>(free_chunk_);
    while (!chunk_parser->IsLastChunk()) {
      chunk_parser = chunk_parser->NextChunk();
    }
    mem_spread -= reinterpret_cast<std::size_t>(chunk_parser);
    return mem_spread;
  }


 private:
  struct Chunk {
    enum Bits : uint16_t { AllocatedChunk = 1u<<15, LastChunk = 1u<<14 };

    uint16_t descriptor;    // bit0-12:size  bit15:allocated bit14:Last
    uint8_t data;

    void SetChunkSize(uint16_t size) { descriptor = (descriptor & 0b1100000000000000) | size; }
    uint16_t constexpr ChunkSize() const  { return descriptor & 0b1111111111111; }

    bool constexpr IsAllocated() const    { return (descriptor & AllocatedChunk) == AllocatedChunk; }
    void SetAllocated()                   { descriptor |= AllocatedChunk; }
    void ResetAllocated()                 { descriptor &= ~AllocatedChunk; }

    bool constexpr IsLastChunk() const    { return (descriptor & LastChunk) == LastChunk; }
    void SetLastChunk()                   { descriptor |= LastChunk; }
    void ResetLastChunk()                 { descriptor &= ~LastChunk; }
    
    Chunk* NextChunk() { return reinterpret_cast<Chunk*>((&data) + ChunkSize()); }

    void* AllocateData(uint16_t size) {
      uint16_t next_size = ChunkSize() - size;
      SetChunkSize(size);
      descriptor |= AllocatedChunk;
      if (next_size != 0) {
        NextChunk()->descriptor = next_size;   // Set next chunk size and clear descriptor's high bits
        if (IsLastChunk()) {
          NextChunk()->SetLastChunk();
          ResetLastChunk();
        }
      }
      return &data;
    }

    void Deallocate() {
      // Deallocation of this chunk :
      //  - if next chunk is unallocated then we have to merge both chunks
      //  - if not, we simply mark this chunk has unallocated
      if (!NextChunk()->IsAllocated()) {
        SetChunkSize(ChunkSize() + NextChunk()->ChunkSize());
      }
      ResetAllocated();
    }
  };

  static std::size_t GetChunksSize(bool allocated) {
    Chunk* chunk_parser = reinterpret_cast<Chunk*>(__malloc_heap_start);
    uint16_t size = 0;
      
    while (allocated == chunk_parser->IsLastChunk()) {
      if (chunk_parser->IsAllocated()) {
        size += chunk_parser->ChunkSize();
      }
      chunk_parser = chunk_parser->NextChunk();
    }
    return size;
  }

  //private:
 public:
  friend class HardwareInitializer;
  static bool Initialize() {
    reinterpret_cast<Chunk*>(free_chunk_)->descriptor = ~(0);
    reinterpret_cast<Chunk*>(free_chunk_)->SetLastChunk();
    reinterpret_cast<Chunk*>(free_chunk_)->ResetAllocated();
    return true;
  }
  static uint8_t* free_chunk_;
};

uint8_t* FreeStore::free_chunk_ = reinterpret_cast<uint8_t*>(__malloc_heap_start);


class HardwareInitializer {
  static bool freestore_initialized_;
};
bool HardwareInitializer::freestore_initialized_ = FreeStore::Initialize();

} // namespace etl

template <typename T>
class allocator {
 public:
  typedef T* pointer;
  typedef const T* const_pointer;
  typedef T value_type;
  typedef T& reference;
  typedef const T& const_reference;
  // construct helper using placement new:
  static void construct(reference p, const_reference value) {
    new (&p) T(value); // T must support copy-constructor
  }

  // destroy helper to invoke destructor explicitly.
  static void destroy(const_reference t) {
    t.~T(); // T must support non-throwing destructor
  }
};

/// Allocates requested number of bytes.
/// @param[in] size number of bytes to allocate
/// @return pointer to allocated memory
void* operator new(std::size_t size) {
  return etl::FreeStore::Allocate(size);
}

/// Deallocates memory space previously allocated by a matching operator new.
/// @param[in] pointer to the memory to deallocate
/// @return
void operator delete(void* ptr) {
  etl::FreeStore::Deallocate(ptr);
}

/// Allocates requested number of bytes.
/// @param[in] size number of bytes to allocate
/// @return pointer to allocated memory
void* operator new[](size_t size) {
  return etl::FreeStore::Allocate(size);
}

/// Deallocates memory space previously allocated by a matching operator new.
/// @param[in] pointer to the memory to deallocate
/// @return
void operator delete[](void* ptr) {
  etl::FreeStore::Deallocate(ptr);
}

/// Placement new for allocating the object inside a given memory buffer.
/// @param[in] ptr pointer to a memory area to initialize the object at
void* operator new(size_t, void* const buf) {
  return buf;
}

/// Placement delete called automatically on "placement new" failure.
void operator delete(void*, void* const){
 }




#endif /* ETL_H_ */