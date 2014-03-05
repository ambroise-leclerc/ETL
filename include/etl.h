/*
 * etl.h
 *
 * Created: 21/02/2014 23:06:23
 *  Author: Ambroise Leclerc
 */ 


#ifndef ETL_H_
#define ETL_H_


#include <stdlib.h>
#include <etl/metautils.h>
#include <memory>
#include <new>

namespace etl {
  
class HardwareInitializer {
  static bool freestore_initialized_;
};
bool HardwareInitializer::freestore_initialized_ = FreeStore::Initialize();

} // namespace etl

#endif /* ETL_H_ */