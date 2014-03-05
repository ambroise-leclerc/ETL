/*
 * memory.h
 *
 * Created: 05/03/2014 18:23:53
 *  Author: Ambroise Leclerc
 */ 


#ifndef MEMORY_H_
#define MEMORY_H_


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

#endif /* MEMORY_H_ */