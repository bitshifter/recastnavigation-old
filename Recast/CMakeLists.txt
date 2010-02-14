CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(recast_SRCS
	Source/Recast.cpp
	Source/RecastArea.cpp
	Source/RecastContour.cpp
	Source/RecastFilter.cpp
	Source/RecastLog.cpp
	Source/RecastMesh.cpp
	Source/RecastMeshDetail.cpp
	Source/RecastRasterization.cpp
	Source/RecastRegion.cpp
	Source/RecastTimer.cpp)

SET(recast_HDRS
	Include/RecastLog.h
	Include/RecastTimer.h
	Include/Recast.h
)

IF(WIN32)
	SET(recast_SRCS ${recast_SRCS} ${recast_HDRS})
ENDIF(WIN32)

INCLUDE_DIRECTORIES(Include)

ADD_LIBRARY(Recast ${recast_SRCS})
