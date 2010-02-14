CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(recastdemo_SRCS 
	Source/ChunkyTriMesh.cpp 
	Source/ConvexVolumeTool.cpp 
	Source/imgui.cpp 
	Source/imguiRenderGL.cpp 
	Source/InputGeom.cpp 
	Source/main.cpp 
	Source/MeshLoaderObj.cpp 
	Source/NavMeshTesterTool.cpp 
	Source/OffMeshConnectionTool.cpp
	Source/Sample.cpp 
	Source/Sample_SoloMeshSimple.cpp 
	Source/Sample_SoloMeshTiled.cpp 
	Source/Sample_TileMesh.cpp
	Source/TestCase.cpp)

SET(recastdemo_HDRS
	Include/ChunkyTriMesh.h
	Include/ConvexVolumeTool.h
	Include/imgui.h
	Include/imguiRenderGL.h
	Include/InputGeom.h
	Include/MeshLoaderObj.h
	Include/NavMeshTesterTool.h
	Include/OffMeshConnectionTool.h
	Include/Sample.h
	Include/Sample_SoloMeshSimple.h
	Include/Sample_SoloMeshTiled.h
	Include/Sample_TileMesh.h
	Include/SDLMain.h
	Include/TestCase.h
)

FIND_PACKAGE(OpenGL REQUIRED)
IF(WIN32)
	FIND_LIBRARY(SDL_LIBRARY NAME SDL PATHS Contrib/SDL/lib)
	FIND_LIBRARY(SDLMAIN_LIBRARY NAME SDLmain PATHS Contrib/SDL/lib)
	SET(SDL_INCLUDE_DIR Contrib/SDL/include)
	SET(recastdemo_SRCS ${recastdemo_SRCS} ${recastdemo_HDRS})
ELSE(WIN32)
	FIND_PACKAGE(SDL REQUIRED)
ENDIF(WIN32)

INCLUDE_DIRECTORIES(Include Contrib ../DebugUtils/Include ../Detour/Include ../Recast/Include ${SDL_INCLUDE_DIR})

ADD_EXECUTABLE(RecastDemo ${recastdemo_SRCS})
TARGET_LINK_LIBRARIES(RecastDemo DebugUtils Detour Recast ${SDL_LIBRARY} ${SDLMAIN_LIBRARY} ${OPENGL_LIBRARIES})


