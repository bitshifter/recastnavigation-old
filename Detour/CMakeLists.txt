CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(detour_SRCS 
	Source/DetourCommon.cpp
	Source/DetourNavMeshBuilder.cpp
	Source/DetourNavMesh.cpp
	Source/DetourNode.cpp)

SET(detour_HDRS
	Include/DetourCommon.h
	Include/DetourNavMeshBuilder.h
	Include/DetourNavMesh.h
	Include/DetourNode.h
)

INCLUDE_DIRECTORIES(Include)

ADD_LIBRARY(Detour ${detour_SRCS} ${detour_HDRS})

