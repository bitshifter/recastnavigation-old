#ifndef DETOURALLOCATOR_H
#define DETOURALLOCATOR_H

#include <stddef.h>

// Memory allocator overrides, taken from Bullet's implementation but without alignement

enum dtAllocHint { DT_ALLOC_PERM, DT_ALLOC_TEMP };

void* dtAlloc(size_t size, dtAllocHint hint);
void  dtFree(void* ptr);

typedef void* (dtAllocFunc)(size_t size, dtAllocHint hint);
typedef void  (dtFreeFunc)(void *memblock);

// The developer can let all Detour memory allocations go through a custom memory allocator, using dtAllocSetCustom
void dtAllocSetCustom(dtAllocFunc *allocFunc, dtFreeFunc *freeFunc);

// Macro for defining class new/delete overrides
#define DT_DECLARE_OBJECT_ALLOCATOR() \
	inline void* operator new(size_t sizeInBytes)   { return dtAlloc(sizeInBytes); }   \
	inline void  operator delete(void* ptr)         { dtFree(ptr); }   \
	inline void* operator new(size_t, void* ptr)    { return ptr; }   \
	inline void  operator delete(void*, void*)      { }   \
	inline void* operator new[](size_t sizeInBytes) { return dtAlloc(sizeInBytes); }   \
	inline void  operator delete[](void* ptr)       { dtFree(ptr); }   \
	inline void* operator new[](size_t, void* ptr)  { return ptr; }   \
	inline void  operator delete[](void*, void*)    { }   \

#endif // DETOURALLOCATOR_H
