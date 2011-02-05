NAME = DetourCrowd

SOURCES = \
	DetourCrowd.cpp \
	DetourLocalBoundary.cpp \
	DetourObstacleAvoidance.cpp \
	DetourPathCorridor.cpp \
	DetourPathQueue.cpp \
	DetourProximityGrid.cpp

HEADERS = \
	DetourCrowd.h \
	DetourLocalBoundary.h \
	DetourObstacleAvoidance.h \
	DetourPathCorridor.h \
	DetourPathQueue.h \
	DetourProximityGrid.h

CPPFLAGS = \
	-I Detour/Include

include $(BUILD)/HelperLibrary.mk
