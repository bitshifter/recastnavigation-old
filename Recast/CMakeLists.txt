CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(recast_SRCS
	Source/Recast.cpp
	Source/RecastArea.cpp
	Source/RecastAlloc.cpp
	Source/RecastContour.cpp
	Source/RecastFilter.cpp
	Source/RecastLog.cpp
	Source/RecastMesh.cpp
	Source/RecastMeshDetail.cpp
	Source/RecastRasterization.cpp
	Source/RecastRegion.cpp
	Source/RecastTimer.cpp)

SET(recast_HDRS
	Include/Recast.h
	Include/RecastAlloc.h
	Include/RecastLog.h
	Include/RecastTimer.h
)

INCLUDE_DIRECTORIES(Include)

ADD_LIBRARY(Recast ${recast_SRCS} ${recast_HDRS})
