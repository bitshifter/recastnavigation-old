#ifndef PARALLELNAVMESHTESTERTOOL_H
#define PARALLELNAVMESHTESTERTOOL_H

#include "Sample.h"

class ParallelNavMeshTesterTool : public SampleTool
{
	Sample* m_sample;
	dtNavMesh* m_navMesh;
	dtQueryFilter m_filter;

	enum ToolMode
	{
		TOOLMODE_PATHFIND_ITER,
		TOOLMODE_PATHFIND_STRAIGHT,
		TOOLMODE_RAYCAST,
		TOOLMODE_DISTANCE_TO_WALL,
		TOOLMODE_FIND_POLYS_AROUND,
	};

	ToolMode m_toolMode;

	bool m_tposSet;
	float m_tpos[3];
	float m_polyPickExt[3];

	static const int MAX_AGENTS = 32;
	static const int MAX_POLYS = 256;
	static const int MAX_SMOOTH = 2048;
	static const int MAX_STEER_POINTS = 10;

	struct Agent
	{
		dtPolyRef m_startRef;
		dtPolyRef m_endRef;
		dtPolyRef m_polys[MAX_POLYS];
		dtPolyRef m_parent[MAX_POLYS];
		int m_npolys;
		float m_straightPath[MAX_POLYS*3];
		unsigned char m_straightPathFlags[MAX_POLYS];
		dtPolyRef m_straightPathPolys[MAX_POLYS];
		int m_nstraightPath;
		float m_smoothPath[MAX_SMOOTH*3];
		int m_nsmoothPath;

		float m_spos[3];
		float m_epos[3];
		float m_hitPos[3];
		float m_hitNormal[3];
		bool m_hitResult;
		float m_distanceToWall;

		int m_pathIterNum;
		const dtPolyRef* m_pathIterPolys;
		int m_pathIterPolyCount;
		float m_prevIterPos[3], m_iterPos[3], m_steerPos[3], m_targetPos[3];

		float m_steerPoints[MAX_STEER_POINTS*3];
		int m_steerPointCount;

		// these could be per worker thread instead of per agent
		class dtNodePool* m_nodePool;
		class dtNodeQueue* m_openList;

		Agent();
		~Agent();

		void reset();
		void render(const ToolMode toolMode, DebugDrawGL& dd, const dtNavMesh& navMesh, const float agentRadius, const float agentHeight, const float agentClimb) const;
		void renderOverlay(double* proj, double* model, int* view, int agentId) const;
		void recalc(const ToolMode toolMode, const dtNavMesh& navMesh, const dtQueryFilter& filter, const float polyPickExt[3]);
		void step(const dtNavMesh& navMesh, const dtQueryFilter& filter);
	};

	Agent m_agents[MAX_AGENTS];
	int m_nagents;

	int m_nthreads;

	void recalc();

public:
	ParallelNavMeshTesterTool();
	virtual ~ParallelNavMeshTesterTool() {}
	virtual int type() { return TOOL_PARALLEL_TESTER; }
	virtual void init(class Sample* sample);
	virtual void reset();
	virtual void handleMenu();
	virtual void handleClick(const float* p, bool shift);
	virtual void handleRender();
	virtual void handleRenderOverlay(double* proj, double* model, int* view);
	virtual void handleStep();
};

#endif // PARALLELNAVMESHTESTERTOOL_H
