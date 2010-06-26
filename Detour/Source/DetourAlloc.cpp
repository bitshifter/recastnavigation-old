#include <stdlib.h>
#include "DetourAlloc.h"

static void *dtAllocDefault(size_t size)
{
	return malloc(size);
}

static void dtFreeDefault(void *ptr)
{
	free(ptr);
}

static dtAllocFunc *sAllocFunc = dtAllocDefault;
static dtFreeFunc *sFreeFunc = dtFreeDefault;

void dtAllocSetCustom(dtAllocFunc *allocFunc, dtFreeFunc *freeFunc)
{
	sAllocFunc = allocFunc ? allocFunc : dtAllocDefault;
	sFreeFunc = freeFunc ? freeFunc : dtFreeDefault;
}

void* dtAlloc(size_t size)
{
	return sAllocFunc(size);
}

void dtFree(void* ptr)
{
	sFreeFunc(ptr);
}
