/// @file freestore.h
/// @data 05/03/2014 23:02:55
/// @author Ambroise Leclerc
/// @brief
//
// Copyright (c) 2014, Ambroise Leclerc
//   All rights reserved.
//
//   Redistribution and use in source and binary forms, with or without
//   modification, are permitted provided that the following conditions are met:
//
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in
//     the documentation and/or other materials provided with the
//     distribution.
//   * Neither the name of the copyright holders nor the names of
//     contributors may be used to endorse or promote products derived
//     from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 'AS IS'
//  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
//  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
//  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
//  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
//  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
//  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
//  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//  POSSIBILITY OF SUCH DAMAGE.

#pragma once
#define ETL_FREESTORE_H_

#include <libstd/include/cstddef>

namespace etl {

class FreeStore : public FreeStoreTracePolicy {
 public:
  /// Allocates size bytes of uninitialized storage.
  /// @param[in] size number of bytes to allocate
  /// @return Pointer to the beginning of allocated memory. The pointer must be
  /// deallocated with FreeStore::Deallocate.
  static void* Allocate(ETLSTD::size_t size) noexcept {
    auto chunk_parser = FirstFreeChunk();
      
    // 0) Setup the start of allocatable area (data_) to the first unallocated block
    while (chunk_parser->IsAllocated()) {
      chunk_parser = chunk_parser->NextChunk();
      free_chunk_ = chunk_parser;
    }
    auto second_choice = chunk_parser;
    uint16_t second_choice_size = 0b1111111111111u;
      
    // 1) Find a free chunk with the same size as the requested allocation size.
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

    // 2) Free chunk with exact same size has not been found : We allocate the second choice
    if (second_choice->ChunkSize() < (size+2)) {
      second_choice = chunk_parser;
    }
    FreeStoreTracePolicy::Log(FreeStoreTracePolicy::Allocation, second_choice);
    return second_choice->AllocateData(size);
  }

  /// Deallocates the memory space pointed to by ptr, which must have been
  /// returned by a previous call to FreeStore::Allocate(). Otherwise undefined
  /// behaviors occurs.
  /// @param[in] pointer to the memory to deallocate
  /// @return
  // noinline: keeps the weak operator delete copies emitted by each translation unit tiny
  [[gnu::noinline]] static void Deallocate(void* ptr) noexcept {
    auto deallocatable = reinterpret_cast<Chunk*>(
      reinterpret_cast<decltype(Chunk::descriptor)*>(ptr)-1);     // Point to chunk beginning
    FreeStoreTracePolicy::Log(FreeStoreTracePolicy::DeallocationRequest, ptr);
    if (deallocatable < free_chunk_) {                            // if chunk to deallocate is before the current first free chunk
      free_chunk_ = deallocatable;                                // then we define it's address as the new first free chunk
    }
    deallocatable->Deallocate();
  }
    
  /// Returns how many bytes have been allocated.
  /// @return allocated bytes
  static ETLSTD::size_t GetMemorySize() noexcept {
    return GetChunksSize(true);
  }

  /// Computes storage space overhead caused by memory fragmentation. A 0%
  /// fragmentation level means that memory is allocated optimally. A 50%
  /// level indicates that fragmentation doubles the memory space needs : 50%
  /// of memory is used by fragmentation holes.
  /// @return Fragmentation level in percents
  static uint8_t GetMemoryFragmentation() noexcept {
    uint16_t holessize = GetChunksSize(false);
    uint16_t memsize = GetMemorySize();
    if (0 == memsize) return 0;
    return 100-((100 * memsize) / (memsize + holessize));
  }

  /// Computes available RAM (i.e. total micro-controller on-board SRAM
  /// minus global variables, stack and free store consumption).
  /// @return Available RAM in bytes
  static ETLSTD::size_t GetFreeMemory() noexcept {
    uint16_t stack_limit = 0;

    auto chunk_parser = FirstFreeChunk();
    while (!chunk_parser->IsLastChunk()) {
      chunk_parser = chunk_parser->NextChunk();
    }
    ETLSTD::size_t mem_spread = reinterpret_cast<uint8_t*>(&stack_limit) - reinterpret_cast<uint8_t*>(chunk_parser);
    return mem_spread;
  }


 private:
  struct Chunk {
    enum Bits : uint16_t { AllocatedChunk = 1u<<15, LastChunk = 1u<<14 };

    uint16_t descriptor;    // bit0-12:size  bit15:allocated bit14:Last
    uint8_t data;

    void SetChunkSize(uint16_t size) { descriptor = (descriptor & 0b1100000000000000) | size; }
    uint16_t ChunkSize() const       { return descriptor & 0b1111111111111; }

    bool IsAllocated() const         { return (descriptor & AllocatedChunk) == AllocatedChunk; }
    void SetAllocated()              { descriptor |= AllocatedChunk; }
    void ResetAllocated()            { descriptor &= ~AllocatedChunk; }

    bool IsLastChunk() const         { return (descriptor & LastChunk) == LastChunk; }
    void SetLastChunk()              { descriptor |= LastChunk; }
    void ResetLastChunk()            { descriptor &= ~LastChunk; }
      
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
      FreeStoreTracePolicy::Log(FreeStoreTracePolicy::DeallocateChunk, this);
      if (!IsLastChunk()) {
        // Deallocation of this chunk :
        //  - if next chunk is unallocated then we have to merge both chunks
        //  - if not, we simply mark this chunk has unallocated
        if (!NextChunk()->IsAllocated()) {
          if (NextChunk()->IsLastChunk()) {
            SetLastChunk();
          }            
          SetChunkSize(ChunkSize() + NextChunk()->ChunkSize());
        }
      }        
      ResetAllocated();
    }
  };

  static ETLSTD::size_t GetChunksSize(bool allocated) __attribute__((noinline)) {
    FirstFreeChunk();
    auto chunk_parser = reinterpret_cast<Chunk*>(__malloc_heap_start);
    ETLSTD::size_t size = 0;
      
    while (!chunk_parser->IsLastChunk()) {
      if (allocated == chunk_parser->IsAllocated()) {
        size += chunk_parser->ChunkSize();
      }
      chunk_parser = chunk_parser->NextChunk();
    }
    return size;
  }

 private:
  /// Returns the first free chunk, setting up the heap as a single free chunk on first use.
  /// The free store is initialized lazily, so a program that does not use it pays nothing,
  /// and no static initializer has to be ordered before the first allocation.
  static Chunk* FirstFreeChunk() noexcept {
    if (free_chunk_ == nullptr) {
      free_chunk_ = reinterpret_cast<Chunk*>(__malloc_heap_start);
      free_chunk_->descriptor = ~(0);
      free_chunk_->SetLastChunk();
      free_chunk_->ResetAllocated();
    }
    return free_chunk_;
  }

  inline static Chunk* free_chunk_ = nullptr;
};

} // namespace etl

