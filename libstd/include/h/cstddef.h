
/*
* cstddef.h
*
* Created: 05/03/2014 18:23:53
*  Author: Ambroise Leclerc
*/


#ifndef CSTDDEF_H_
#define CSTDDEF_H_

typedef decltype(nullptr) nullptr_t;
typedef ::size_t size_t;
typedef ::ptrdiff_t ptrdiff_t;
typedef ::max_align_t max_align_t;

#endif // CSTDDEF_H_