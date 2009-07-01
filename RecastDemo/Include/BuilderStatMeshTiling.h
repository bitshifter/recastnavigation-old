#ifndef RECASTBUILDERSTATMESHTILING_H
#define RECASTBUILDERSTATMESHTILING_H

#include "BuilderStatMesh.h"
#include "DetourStatNavMesh.h"
#include "Recast.h"
#include "RecastLog.h"
#include "ChunkyTriMesh.h"

class BuilderStatMeshTiling : public BuilderStatMesh
{
protected:

	struct Tile
	{
		inline Tile() : chf(0), cset(0), solid(0), buildTime(0) {}
		inline ~Tile() { delete chf; delete cset; delete solid; }
		rcCompactHeightfield* chf;
		rcHeightfield* solid;
		rcContourSet* cset;
		int buildTime;
	};
	
	struct TileSet
	{
		inline TileSet() : width(0), height(0), tiles(0) {}
		inline ~TileSet() { delete [] tiles; }
		int width, height;
		float bmin[3], bmax[3];
		float cs, ch;
		Tile* tiles;
	};
	
	bool m_keepInterResults;
	float m_tileSize;
	rcBuildTimes m_buildTimes; 
	
	rcChunkyTriMesh* m_chunkyMesh;
	rcPolyMesh* m_polyMesh;
	rcConfig m_cfg;	
	TileSet* m_tileSet;
	
	
	enum DrawMode
	{
		DRAWMODE_NAVMESH,
		DRAWMODE_NAVMESH_TRANS,
		DRAWMODE_NAVMESH_BVTREE,
		DRAWMODE_NAVMESH_INVIS,
		DRAWMODE_MESH,
		DRAWMODE_VOXELS,
		DRAWMODE_VOXELS_WALKABLE,
		DRAWMODE_COMPACT,
		DRAWMODE_COMPACT_DISTANCE,
		DRAWMODE_COMPACT_REGIONS,
		DRAWMODE_REGION_CONNECTIONS,
		DRAWMODE_RAW_CONTOURS,
		DRAWMODE_BOTH_CONTOURS,
		DRAWMODE_CONTOURS,
		DRAWMODE_POLYMESH,
		MAX_DRAWMODE
	};
	
	DrawMode m_drawMode;
	
	void cleanup();
	
public:
	BuilderStatMeshTiling();
	virtual ~BuilderStatMeshTiling();
	
	virtual void handleSettings();
	virtual void handleDebugMode();
	
	virtual void handleRender();
	virtual void handleRenderOverlay(class GLFont* font, double* proj, double* model, int* view);
	virtual void handleMeshChanged(const float* verts, int nverts,
								   const int* tris, const float* trinorms, int ntris,
								   const float* bmin, const float* bmax);
	virtual bool handleBuild();
};


#endif // RECASTBUILDERSTATMESHTILING_H
