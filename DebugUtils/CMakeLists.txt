CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(debugutils_SRCS 
	Source/DebugDraw.cpp 
	Source/DetourDebugDraw.cpp 
	Source/RecastDebugDraw.cpp 
	Source/RecastDump.cpp)

SET(debugutils_HDRS
	Include/DebugDraw.h
	Include/DetourDebugDraw.h
	Include/RecastDebugDraw.h
	Include/RecastDump.h
)

IF(WIN32)
	SET(debugutils_SRCS ${debugutils_SRCS} ${debugutils_HDRS})
ENDIF(WIN32)

INCLUDE_DIRECTORIES(Include ../Detour/Include ../Recast/Include)

ADD_LIBRARY(DebugUtils ${debugutils_SRCS})
