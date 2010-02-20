CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

PROJECT(RecastDemo)
#SET(RECAST_VERSION r129)

SET(CMAKE_SUPPRESS_REGENERATION  1)
SET(CMAKE_USE_RELATIVE_PATHS 1)

IF(NOT CMAKE_BUILD_TYPE)
#	SET(CMAKE_BUILD_TYPE "Debug")
	SET(CMAKE_BUILD_TYPE "Release")
ENDIF(NOT CMAKE_BUILD_TYPE)

IF(MSVC)
	OPTION(USE_MSVC_FAST_FLOATINGPOINT "Use MSVC /fp:fast option" ON)
	IF(USE_MSVC_FAST_FLOATINGPOINT)
		ADD_DEFINITIONS(/fp:fast)
	ENDIF(USE_MSVC_FAST_FLOATINGPOINT)
	
	ADD_DEFINITIONS(/D _CRT_SECURE_NO_WARNINGS)
ENDIF(MSVC)

OPTION(INTERNAL_CREATE_RELATIVE_PATH_PROJECTFILES "Create project files with relative paths" ON)
IF(INTERNAL_CREATE_RELATIVE_PATH_PROJECTFILES)
	SET(CMAKE_SUPPRESS_REGENERATION  1)
	SET(CMAKE_USE_RELATIVE_PATHS 1)
ENDIF(INTERNAL_CREATE_RELATIVE_PATH_PROJECTFILES)

# RecastDemo sources
SET(recastdemo_SRCS 
	../Source/ChunkyTriMesh.cpp 
	../Source/ConvexVolumeTool.cpp 
	../Source/imgui.cpp 
	../Source/imguiRenderGL.cpp 
	../Source/InputGeom.cpp 
	../Source/main.cpp 
	../Source/MeshLoaderObj.cpp 
	../Source/NavMeshTesterTool.cpp 
	../Source/OffMeshConnectionTool.cpp
	../Source/Sample.cpp 
	../Source/Sample_SoloMeshSimple.cpp 
	../Source/Sample_SoloMeshTiled.cpp 
	../Source/Sample_TileMesh.cpp
	../Source/TestCase.cpp)

SET(recastdemo_HDRS
	../Include/ChunkyTriMesh.h
	../Include/ConvexVolumeTool.h
	../Include/imgui.h
	../Include/imguiRenderGL.h
	../Include/InputGeom.h
	../Include/MeshLoaderObj.h
	../Include/NavMeshTesterTool.h
	../Include/OffMeshConnectionTool.h
	../Include/Sample.h
	../Include/Sample_SoloMeshSimple.h
	../Include/Sample_SoloMeshTiled.h
	../Include/Sample_TileMesh.h
	../Include/SDLMain.h
	../Include/TestCase.h
)

# DebugUtils sources
SET(debugutils_SRCS 
	../../DebugUtils/Source/DebugDraw.cpp 
	../../DebugUtils/Source/DetourDebugDraw.cpp 
	../../DebugUtils/Source/RecastDebugDraw.cpp 
	../../DebugUtils/Source/RecastDump.cpp)

SET(debugutils_HDRS
	../../DebugUtils/Include/DebugDraw.h
	../../DebugUtils/Include/DetourDebugDraw.h
	../../DebugUtils/Include/RecastDebugDraw.h
	../../DebugUtils/Include/RecastDump.h
)

# Detour sources
SET(detour_SRCS 
	../../Detour/Source/DetourCommon.cpp
	../../Detour/Source/DetourNavMeshBuilder.cpp
	../../Detour/Source/DetourNavMesh.cpp
	../../Detour/Source/DetourNode.cpp)

SET(detour_HDRS
	../../Detour/Include/DetourCommon.h
	../../Detour/Include/DetourNavMeshBuilder.h
	../../Detour/Include/DetourNavMesh.h
	../../Detour/Include/DetourNode.h
)

# Recast Sources
SET(recast_SRCS
	../../Recast/Source/Recast.cpp
	../../Recast/Source/RecastArea.cpp
	../../Recast/Source/RecastContour.cpp
	../../Recast/Source/RecastFilter.cpp
	../../Recast/Source/RecastLog.cpp
	../../Recast/Source/RecastMesh.cpp
	../../Recast/Source/RecastMeshDetail.cpp
	../../Recast/Source/RecastRasterization.cpp
	../../Recast/Source/RecastRegion.cpp
	../../Recast/Source/RecastTimer.cpp)

SET(recast_HDRS
	../../Recast/Include/RecastLog.h
	../../Recast/Include/RecastTimer.h
	../../Recast/Include/Recast.h
)

# Visual Studio project folders
IF(MSVC)
	SOURCE_GROUP("DebugUtils\\Include" FILES ${debugutils_HDRS})
	SOURCE_GROUP("DebugUtils\\Source" FILES ${debugutils_SRCS})
	SET(debugutils_SRCS ${debugutils_SRCS} ${debugutils_HDRS})
	
	SOURCE_GROUP("Detour\\Include" FILES ${detour_HDRS})
	SOURCE_GROUP("Detour\\Source" FILES ${detour_SRCS})
	SET(detour_SRCS ${detour_SRCS} ${detour_HDRS})

	SOURCE_GROUP("Recast\\Include" FILES ${recast_HDRS})
	SOURCE_GROUP("Recast\\Source" FILES ${recast_SRCS})
	SET(recast_SRCS ${recast_SRCS} ${recast_HDRS})
	
	SOURCE_GROUP("Demo\\Include" FILES ${recastdemo_HDRS})
	SOURCE_GROUP("Demo\\Source" FILES ${recastdemo_SRCS})
	SET(recastdemo_SRCS ${recastdemo_SRCS} ${recastdemo_HDRS})
ENDIF(MSVC)

FIND_PACKAGE(OpenGL REQUIRED)
IF(MSVC)
	FIND_LIBRARY(SDL_LIBRARY NAME SDL PATHS ../Contrib/SDL/lib)
	FIND_LIBRARY(SDLMAIN_LIBRARY NAME SDLmain PATHS ../Contrib/SDL/lib)
	SET(SDL_INCLUDE_DIR ../Contrib/SDL/include)
ELSE(MSVC)
	FIND_PACKAGE(SDL REQUIRED)
ENDIF(MSVC)

INCLUDE_DIRECTORIES(../Include ../Contrib ../../DebugUtils/Include ../../Detour/Include ../../Recast/Include ${SDL_INCLUDE_DIR})

ADD_EXECUTABLE(RecastDemo WIN32 ${debugutils_SRCS} ${detour_SRCS} ${recast_SRCS} ${recastdemo_SRCS})
TARGET_LINK_LIBRARIES(RecastDemo ${SDL_LIBRARY} ${SDLMAIN_LIBRARY} ${OPENGL_LIBRARIES})

IF(MSVC)
	# Enable some linker optimisations
	SET_TARGET_PROPERTIES(RecastDemo PROPERTIES LINK_FLAGS_RELEASE "/OPT:REF /OPT:ICF")
	SET_TARGET_PROPERTIES(RecastDemo PROPERTIES LINK_FLAGS_MINSIZEREL "/OPT:REF /OPT:ICF")
	SET_TARGET_PROPERTIES(RecastDemo PROPERTIES LINK_FLAGS_RELWITHDEBINFO "/OPT:REF /OPT:ICF")
	
	# Copy the built executable to the Bin directory
	ADD_CUSTOM_COMMAND(TARGET RecastDemo POST_BUILD
		COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different "$(TargetPath)" "${RecastDemo_SOURCE_DIR}/../Bin/")
ENDIF(MSVC)

