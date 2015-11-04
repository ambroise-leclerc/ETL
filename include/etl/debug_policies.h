/// @file debug_policies.h
/// @date 04/04/2014 16:04:23
/// @author Ambroise Leclerc
/// @brief Embedded Template Library debug policies
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
//  POSSIBILITY OF SUCH DAMAGE./*



#ifndef ETL_DEBUG_H_
#define ETL_DEBUG_H_

namespace etl {
    
using std::uint8_t;
  
class FreeStoreNoDebug {
 public: 
  enum operation { Allocation = 'A', DeallocationRequest = 'R', DeallocateChunk = 'C' };
  static void Log(operation op, void* address) { };
};  

template <uint8_t LOGSIZE>
class FreeStoreDebugTrace {
 public:
  enum operation { Allocation = 'A', DeallocationRequest = 'R', DeallocateChunk = 'C' };
  static void Log(operation op, void* address) {
    LogOperations[LogCounter] = op;
    LogAdress[LogCounter] = address;
    LogCounter++;
    if (LogCounter == LOGSIZE)
      LogCounter = 0;
  }
    
  static uint8_t LogCounter;
  static operation LogOperations[LOGSIZE];
  static void* LogAdress[LOGSIZE];
};

template <uint8_t LOGSIZE> uint8_t FreeStoreDebugTrace<LOGSIZE>::LogCounter = 0;
template <uint8_t LOGSIZE> typename FreeStoreDebugTrace<LOGSIZE>::operation FreeStoreDebugTrace<LOGSIZE>::LogOperations[LOGSIZE];
template <uint8_t LOGSIZE> void* FreeStoreDebugTrace<LOGSIZE>::LogAdress[LOGSIZE];

#ifndef ETL_FREESTORE_LOG_DEPTH
using FreeStoreTracePolicy = FreeStoreNoDebug;
#else
using FreeStoreTracePolicy = FreeStoreDebugTrace<ETL_FREESTORE_LOG_DEPTH>;
#endif
  
} // namespace etl

#endif /* ETL_DEBUG_H_ */