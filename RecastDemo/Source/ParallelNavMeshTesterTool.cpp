#include <cstring>
#include <cmath>
#include "SDL.h"
#include "SDL_opengl.h"
#include "Recast.h"
#include "DetourDebugDraw.h"
#include "imgui.h"
#include "ParallelNavMeshTesterTool.h"

// Uncomment this to dump all the requests in stdout.
#define DUMP_REQS

inline bool inRange(const float* v1, const float* v2, const float r, const float h)
{
	const float dx = v2[0] - v1[0];
	const float dy = v2[1] - v1[1];
	const float dz = v2[2] - v1[2];
	return (dx*dx + dz*dz) < r*r && fabsf(dy) < h;
}

static bool getSteerTarget(const dtNavMesh* navMesh, const float* startPos, const float* endPos,
						   const float minTargetDist,
						   const dtPolyRef* path, const int pathSize,
						   float* steerPos, unsigned char& steerPosFlag, dtPolyRef& steerPosRef,
						   float* outPoints = 0, int* outPointCount = 0)
{
	// Find steer target.
	static const int MAX_STEER_POINTS = 3;
	float steerPath[MAX_STEER_POINTS*3];
	unsigned char steerPathFlags[MAX_STEER_POINTS];
	dtPolyRef steerPathPolys[MAX_STEER_POINTS];
	int nsteerPath = navMesh->findStraightPath(startPos, endPos, path, pathSize,
											   steerPath, steerPathFlags, steerPathPolys, MAX_STEER_POINTS);
	if (!nsteerPath)
		return false;

	if (outPoints && outPointCount)
	{
		*outPointCount = nsteerPath;
		for (int i = 0; i < nsteerPath; ++i)
			rcVcopy(&outPoints[i*3], &steerPath[i*3]);
	}


	// Find vertex far enough to steer to.
	int ns = 0;
	while (ns < nsteerPath)
	{
		// Stop at Off-Mesh link or when point is further than slop away.
		if ((steerPathFlags[ns] & DT_STRAIGHTPATH_OFFMESH_CONNECTION) ||
			!inRange(&steerPath[ns*3], startPos, minTargetDist, 1000.0f))
			break;
		ns++;
	}
	// Failed to find good point to steer to.
	if (ns >= nsteerPath)
		return false;

	rcVcopy(steerPos, &steerPath[ns*3]);
	steerPosFlag = steerPathFlags[ns];
	steerPosRef = steerPathPolys[ns];

	return true;
}

static void drawAgent(DebugDrawGL& dd, const float* pos, float r, float h, float c, const unsigned int col)
{
	dd.depthMask(false);

	// Agent dimensions.
	duDebugDrawCylinderWire(&dd, pos[0]-r, pos[1]+0.02f, pos[2]-r, pos[0]+r, pos[1]+h, pos[2]+r, col, 2.0f);

	duDebugDrawCircle(&dd, pos[0],pos[1]+c,pos[2],r,duRGBA(0,0,0,64),1.0f);

	unsigned int colb = duRGBA(0,0,0,196);
	dd.begin(DU_DRAW_LINES);
	dd.vertex(pos[0], pos[1]-c, pos[2], colb);
	dd.vertex(pos[0], pos[1]+c, pos[2], colb);
	dd.vertex(pos[0]-r/2, pos[1]+0.02f, pos[2], colb);
	dd.vertex(pos[0]+r/2, pos[1]+0.02f, pos[2], colb);
	dd.vertex(pos[0], pos[1]+0.02f, pos[2]-r/2, colb);
	dd.vertex(pos[0], pos[1]+0.02f, pos[2]+r/2, colb);
	dd.end();

	dd.depthMask(true);
}

static void getPolyCenter(const dtNavMesh& navMesh, dtPolyRef ref, float* center)
{
	const dtPoly* p = navMesh.getPolyByRef(ref);
	if (!p) return;
	const float* verts = navMesh.getPolyVertsByRef(ref);
	center[0] = 0;
	center[1] = 0;
	center[2] = 0;
	for (int i = 0; i < (int)p->vertCount; ++i)
	{
		const float* v = &verts[p->verts[i]*3];
		center[0] += v[0];
		center[1] += v[1];
		center[2] += v[2];
	}
	const float s = 1.0f / p->vertCount;
	center[0] *= s;
	center[1] *= s;
	center[2] *= s;
}

void ParallelNavMeshTesterTool::Agent::reset()
{
	m_startRef = 0;
	m_endRef = 0;
	m_npolys = 0;
	m_nstraightPath = 0;
	m_nsmoothPath = 0;
	memset(m_hitPos, 0, sizeof(m_hitPos));
	memset(m_hitNormal, 0, sizeof(m_hitNormal));
	m_distanceToWall = 0;
}

void ParallelNavMeshTesterTool::Agent::recalc(const ToolMode toolMode, const dtNavMesh& navMesh, const dtQueryFilter& filter, const float polyPickExt[3])
{
	m_startRef = navMesh.findNearestPoly(m_spos, polyPickExt, &filter, 0);
	m_endRef = navMesh.findNearestPoly(m_epos,polyPickExt, &filter, 0);

	if (toolMode == TOOLMODE_PATHFIND_ITER)
	{
		m_pathIterNum = 0;
		if (m_startRef && m_endRef)
		{
#ifdef DUMP_REQS
			printf("pi  %f %f %f  %f %f %f  0x%x 0x%x\n",
				   m_spos[0],m_spos[1],m_spos[2], m_epos[0],m_epos[1],m_epos[2],
				   filter.includeFlags, filter.excludeFlags);
#endif

			m_npolys = navMesh.findPath(m_startRef, m_endRef, m_spos, m_epos, &filter, m_polys, MAX_POLYS);

			m_nsmoothPath = 0;

			if (m_npolys)
			{
				// Iterate over the path to find smooth path on the detail mesh surface.
				const dtPolyRef* polys = m_polys;
				int npolys = m_npolys;

				float iterPos[3], targetPos[3];
				navMesh.closestPointOnPolyBoundary(m_startRef, m_spos, iterPos);
				navMesh.closestPointOnPolyBoundary(polys[npolys-1], m_epos, targetPos);

				static const float STEP_SIZE = 0.5f;
				static const float SLOP = 0.01f;

				m_nsmoothPath = 0;

				rcVcopy(&m_smoothPath[m_nsmoothPath*3], iterPos);
				m_nsmoothPath++;

				// Move towards target a small advancement at a time until target reached or
				// when ran out of memory to store the path.
				while (npolys && m_nsmoothPath < MAX_SMOOTH)
				{
					// Find location to steer towards.
					float steerPos[3];
					unsigned char steerPosFlag;
					dtPolyRef steerPosRef;

					if (!getSteerTarget(&navMesh, iterPos, targetPos, SLOP,
										polys, npolys, steerPos, steerPosFlag, steerPosRef))
						break;

					bool endOfPath = (steerPosFlag & DT_STRAIGHTPATH_END) ? true : false;
					bool offMeshConnection = (steerPosFlag & DT_STRAIGHTPATH_OFFMESH_CONNECTION) ? true : false;

					// Find movement delta.
					float delta[3], len;
					rcVsub(delta, steerPos, iterPos);
					len = sqrtf(rcVdot(delta,delta));
					// If the steer target is end of path or off-mesh link, do not move past the location.
					if ((endOfPath || offMeshConnection) && len < STEP_SIZE)
						len = 1;
					else
						len = STEP_SIZE / len;
					float moveTgt[3];
					rcVmad(moveTgt, iterPos, delta, len);

					// Move
					float result[3];
					int n = navMesh.moveAlongPathCorridor(iterPos, moveTgt, result, polys, npolys);
					float h = 0;
					navMesh.getPolyHeight(polys[n], result, &h);
					result[1] = h;
					// Shrink path corridor if advanced.
					if (n)
					{
						polys += n;
						npolys -= n;
					}
					// Update position.
					rcVcopy(iterPos, result);

					// Handle end of path and off-mesh links when close enough.
					if (endOfPath && inRange(iterPos, steerPos, SLOP, 1.0f))
					{
						// Reached end of path.
						rcVcopy(iterPos, targetPos);
						if (m_nsmoothPath < MAX_SMOOTH)
						{
							rcVcopy(&m_smoothPath[m_nsmoothPath*3], iterPos);
							m_nsmoothPath++;
						}
						break;
					}
					else if (offMeshConnection && inRange(iterPos, steerPos, SLOP, 1.0f))
					{
						// Reached off-mesh connection.
						float startPos[3], endPos[3];

						// Advance the path up to and over the off-mesh connection.
						dtPolyRef prevRef = 0, polyRef = polys[0];
						while (npolys && polyRef != steerPosRef)
						{
							prevRef = polyRef;
							polyRef = polys[0];
							polys++;
							npolys--;
						}

						// Handle the connection.
						if (navMesh.getOffMeshConnectionPolyEndPoints(prevRef, polyRef, startPos, endPos))
						{
							if (m_nsmoothPath < MAX_SMOOTH)
							{
								rcVcopy(&m_smoothPath[m_nsmoothPath*3], startPos);
								m_nsmoothPath++;
								// Hack to make the dotted path not visible during off-mesh connection.
								if (m_nsmoothPath & 1)
								{
									rcVcopy(&m_smoothPath[m_nsmoothPath*3], startPos);
									m_nsmoothPath++;
								}
							}
							// Move position at the other side of the off-mesh link.
							rcVcopy(iterPos, endPos);
							float h;
							navMesh.getPolyHeight(polys[0], iterPos, &h);
							iterPos[1] = h;
						}
					}

					// Store results.
					if (m_nsmoothPath < MAX_SMOOTH)
					{
						rcVcopy(&m_smoothPath[m_nsmoothPath*3], iterPos);
						m_nsmoothPath++;
					}
				}
			}

		}
		else
		{
			m_npolys = 0;
			m_nsmoothPath = 0;
		}
	}
	else if (toolMode == TOOLMODE_PATHFIND_STRAIGHT)
	{
		if (m_startRef && m_endRef)
		{
#ifdef DUMP_REQS
			printf("ps  %f %f %f  %f %f %f  0x%x 0x%x\n",
				   m_spos[0],m_spos[1],m_spos[2], m_epos[0],m_epos[1],m_epos[2],
				   filter.includeFlags, filter.excludeFlags);
#endif
			m_npolys = navMesh.findPath(m_startRef, m_endRef, m_spos, m_epos, &filter, m_polys, MAX_POLYS);
			m_nstraightPath = 0;
			if (m_npolys)
			{
				// In case of partial path, make sure the end point is clamped to the last polygon.
				float epos[3];
				rcVcopy(epos, m_epos);
				if (m_polys[m_npolys-1] != m_endRef)
					navMesh.closestPointOnPoly(m_polys[m_npolys-1], m_epos, epos);

				m_nstraightPath = navMesh.findStraightPath(m_spos, epos, m_polys, m_npolys,
															  m_straightPath, m_straightPathFlags,
															  m_straightPathPolys, MAX_POLYS);
			}
		}
		else
		{
			m_npolys = 0;
			m_nstraightPath = 0;
		}
	}
	else if (toolMode == TOOLMODE_RAYCAST)
	{
		m_nstraightPath = 0;
		if (m_startRef)
		{
#ifdef DUMP_REQS
			printf("rc  %f %f %f  %f %f %f  0x%x 0x%x\n",
				   m_spos[0],m_spos[1],m_spos[2], m_epos[0],m_epos[1],m_epos[2],
				   filter.includeFlags, filter.excludeFlags);
#endif
			float t = 0;
			m_npolys = 0;
			m_nstraightPath = 2;
			m_straightPath[0] = m_spos[0];
			m_straightPath[1] = m_spos[1];
			m_straightPath[2] = m_spos[2];
			m_npolys = navMesh.raycast(m_startRef, m_spos, m_epos, &filter, t, m_hitNormal, m_polys, MAX_POLYS);
			if (t > 1)
			{
				// No hit
				rcVcopy(m_hitPos, m_epos);
				m_hitResult = false;
			}
			else
			{
				// Hit
				m_hitPos[0] = m_spos[0] + (m_epos[0] - m_spos[0]) * t;
				m_hitPos[1] = m_spos[1] + (m_epos[1] - m_spos[1]) * t;
				m_hitPos[2] = m_spos[2] + (m_epos[2] - m_spos[2]) * t;
				if (m_npolys)
				{
					float h = 0;
					navMesh.getPolyHeight(m_polys[m_npolys-1], m_hitPos, &h);
					m_hitPos[1] = h;
				}
				m_hitResult = true;
			}
			rcVcopy(&m_straightPath[3], m_hitPos);
		}
	}
	else if (toolMode == TOOLMODE_DISTANCE_TO_WALL)
	{
		m_distanceToWall = 0;
		if (m_startRef)
		{
#ifdef DUMP_REQS
			printf("dw  %f %f %f  %f  0x%x 0x%x\n",
				   m_spos[0],m_spos[1],m_spos[2], 100.0f,
				   filter.includeFlags, filter.excludeFlags);
#endif
			m_distanceToWall = navMesh.findDistanceToWall(m_startRef, m_spos, 100.0f, &filter, m_hitPos, m_hitNormal);
		}
	}
	else if (toolMode == TOOLMODE_FIND_POLYS_AROUND)
	{
		if (m_startRef)
		{
			const float dx = m_epos[0] - m_spos[0];
			const float dz = m_epos[2] - m_spos[2];
			float dist = sqrtf(dx*dx + dz*dz);
#ifdef DUMP_REQS
			printf("fp  %f %f %f  %f  0x%x 0x%x\n",
				   m_spos[0],m_spos[1],m_spos[2], dist,
				   filter.includeFlags, filter.excludeFlags);
#endif
			m_npolys = navMesh.findPolysAround(m_startRef, m_spos, dist, &filter, m_polys, m_parent, 0, MAX_POLYS);
		}
	}

}

void ParallelNavMeshTesterTool::Agent::step(const dtNavMesh &navMesh, const dtQueryFilter& filter)
{
	if (!m_startRef || !m_endRef)
		return;

	static const float STEP_SIZE = 0.5f;
	static const float SLOP = 0.01f;

	if (m_pathIterNum == 0)
	{
		m_npolys = navMesh.findPath(m_startRef, m_endRef, m_spos, m_epos, &filter, m_polys, MAX_POLYS);
		m_nsmoothPath = 0;

		m_pathIterPolys = m_polys;
		m_pathIterPolyCount = m_npolys;

		if (m_pathIterPolyCount)
		{
			// Iterate over the path to find smooth path on the detail mesh surface.

			navMesh.closestPointOnPolyBoundary(m_startRef, m_spos, m_iterPos);
			navMesh.closestPointOnPolyBoundary(m_pathIterPolys[m_pathIterPolyCount-1], m_epos, m_targetPos);

			m_nsmoothPath = 0;

			rcVcopy(&m_smoothPath[m_nsmoothPath*3], m_iterPos);
			m_nsmoothPath++;
		}
	}

	rcVcopy(m_prevIterPos, m_iterPos);

	m_pathIterNum++;

	if (!m_pathIterPolyCount)
		return;

	if (m_nsmoothPath >= MAX_SMOOTH)
		return;

	// Move towards target a small advancement at a time until target reached or
	// when ran out of memory to store the path.

	// Find location to steer towards.
	float steerPos[3];
	unsigned char steerPosFlag;
	dtPolyRef steerPosRef;

	if (!getSteerTarget(&navMesh, m_iterPos, m_targetPos, SLOP,
						m_pathIterPolys, m_pathIterPolyCount, steerPos, steerPosFlag, steerPosRef,
						m_steerPoints, &m_steerPointCount))
		return;

	rcVcopy(m_steerPos, steerPos);

	bool endOfPath = (steerPosFlag & DT_STRAIGHTPATH_END) ? true : false;
	bool offMeshConnection = (steerPosFlag & DT_STRAIGHTPATH_OFFMESH_CONNECTION) ? true : false;

	// Find movement delta.
	float delta[3], len;
	rcVsub(delta, steerPos, m_iterPos);
	len = sqrtf(rcVdot(delta,delta));
	// If the steer target is end of path or off-mesh link, do not move past the location.
	if ((endOfPath || offMeshConnection) && len < STEP_SIZE)
		len = 1;
	else
		len = STEP_SIZE / len;
	float moveTgt[3];
	rcVmad(moveTgt, m_iterPos, delta, len);

	// Move
	float result[3];
	int n = navMesh.moveAlongPathCorridor(m_iterPos, moveTgt, result, m_pathIterPolys, m_pathIterPolyCount);
	float h = 0;
	navMesh.getPolyHeight(m_pathIterPolys[n], result, &h);
	result[1] = h;
	// Shrink path corridor if advanced.
	if (n)
	{
		m_pathIterPolys += n;
		m_pathIterPolyCount -= n;
	}
	// Update position.
	rcVcopy(m_iterPos, result);

	// Handle end of path and off-mesh links when close enough.
	if (endOfPath && inRange(m_iterPos, steerPos, SLOP, 1.0f))
	{
		// Reached end of path.
		rcVcopy(m_iterPos, m_targetPos);
		if (m_nsmoothPath < MAX_SMOOTH)
		{
			rcVcopy(&m_smoothPath[m_nsmoothPath*3], m_iterPos);
			m_nsmoothPath++;
		}
		return;
	}
	else if (offMeshConnection && inRange(m_iterPos, steerPos, SLOP, 1.0f))
	{
		// Reached off-mesh connection.
		float startPos[3], endPos[3];

		// Advance the path up to and over the off-mesh connection.
		dtPolyRef prevRef = 0, polyRef = m_pathIterPolys[0];
		while (m_pathIterPolyCount && polyRef != steerPosRef)
		{
			prevRef = polyRef;
			polyRef = m_pathIterPolys[0];
			m_pathIterPolys++;
			m_pathIterPolyCount--;
		}

		// Handle the connection.
		if (navMesh.getOffMeshConnectionPolyEndPoints(prevRef, polyRef, startPos, endPos))
		{
			if (m_nsmoothPath < MAX_SMOOTH)
			{
				rcVcopy(&m_smoothPath[m_nsmoothPath*3], startPos);
				m_nsmoothPath++;
				// Hack to make the dotted path not visible during off-mesh connection.
				if (m_nsmoothPath & 1)
				{
					rcVcopy(&m_smoothPath[m_nsmoothPath*3], startPos);
					m_nsmoothPath++;
				}
			}
			// Move position at the other side of the off-mesh link.
			rcVcopy(m_iterPos, endPos);
			float h;
			navMesh.getPolyHeight(m_pathIterPolys[0], m_iterPos, &h);
			m_iterPos[1] = h;
		}
	}

	// Store results.
	if (m_nsmoothPath < MAX_SMOOTH)
	{
		rcVcopy(&m_smoothPath[m_nsmoothPath*3], m_iterPos);
		m_nsmoothPath++;
	}

}

void ParallelNavMeshTesterTool::Agent::render(const ToolMode toolMode, DebugDrawGL& dd, const dtNavMesh& navMesh, const float agentRadius, const float agentHeight, const float agentClimb) const
{
	static const unsigned int startCol = duRGBA(128,25,0,192);
	static const unsigned int endCol = duRGBA(51,102,0,129);
	static const unsigned int pathCol = duRGBA(0,0,0,64);

	dd.depthMask(false);
	drawAgent(dd, m_spos, agentRadius, agentHeight, agentClimb, startCol);
	drawAgent(dd, m_epos, agentRadius, agentHeight, agentClimb, endCol);
	dd.depthMask(true);

	if (toolMode == TOOLMODE_PATHFIND_ITER)
	{
		duDebugDrawNavMeshPoly(&dd, navMesh, m_startRef, startCol);
		duDebugDrawNavMeshPoly(&dd, navMesh, m_endRef, endCol);

		if (m_npolys)
		{
			for (int i = 1; i < m_npolys-1; ++i)
				duDebugDrawNavMeshPoly(&dd, navMesh, m_polys[i], pathCol);
		}

		if (m_nsmoothPath)
		{
			dd.depthMask(false);
			const unsigned int pathCol = duRGBA(0,0,0,220);
			dd.begin(DU_DRAW_LINES, 3.0f);
			for (int i = 0; i < m_nsmoothPath; ++i)
				dd.vertex(m_smoothPath[i*3], m_smoothPath[i*3+1]+0.1f, m_smoothPath[i*3+2], pathCol);
			dd.end();
			dd.depthMask(true);
		}

		if (m_pathIterNum)
		{
			duDebugDrawNavMeshPoly(&dd, navMesh, m_pathIterPolys[0], duRGBA(255,255,255,128));

			dd.depthMask(false);
			dd.begin(DU_DRAW_LINES, 1.0f);

			const unsigned int prevCol = duRGBA(255,192,0,220);
			const unsigned int curCol = duRGBA(255,255,255,220);
			const unsigned int steerCol = duRGBA(0,192,255,220);

			dd.vertex(m_prevIterPos[0],m_prevIterPos[1]-0.3f,m_prevIterPos[2], prevCol);
			dd.vertex(m_prevIterPos[0],m_prevIterPos[1]+0.3f,m_prevIterPos[2], prevCol);

			dd.vertex(m_iterPos[0],m_iterPos[1]-0.3f,m_iterPos[2], curCol);
			dd.vertex(m_iterPos[0],m_iterPos[1]+0.3f,m_iterPos[2], curCol);

			dd.vertex(m_prevIterPos[0],m_prevIterPos[1]+0.3f,m_prevIterPos[2], prevCol);
			dd.vertex(m_iterPos[0],m_iterPos[1]+0.3f,m_iterPos[2], prevCol);

			dd.vertex(m_prevIterPos[0],m_prevIterPos[1]+0.3f,m_prevIterPos[2], steerCol);
			dd.vertex(m_steerPos[0],m_steerPos[1]+0.3f,m_steerPos[2], steerCol);

			for (int i = 0; i < m_steerPointCount-1; ++i)
			{
				dd.vertex(m_steerPoints[i*3+0],m_steerPoints[i*3+1]+0.2f,m_steerPoints[i*3+2], duDarkenColor(steerCol));
				dd.vertex(m_steerPoints[(i+1)*3+0],m_steerPoints[(i+1)*3+1]+0.2f,m_steerPoints[(i+1)*3+2], duDarkenColor(steerCol));
			}

			dd.end();
			dd.depthMask(true);
		}
	}
	else if (toolMode == TOOLMODE_PATHFIND_STRAIGHT)
	{
		duDebugDrawNavMeshPoly(&dd, navMesh, m_startRef, startCol);
		duDebugDrawNavMeshPoly(&dd, navMesh, m_endRef, endCol);

		if (m_npolys)
		{
			for (int i = 1; i < m_npolys-1; ++i)
				duDebugDrawNavMeshPoly(&dd, navMesh, m_polys[i], pathCol);
		}

		if (m_nstraightPath)
		{
			dd.depthMask(false);
			const unsigned int pathCol = duRGBA(64,16,0,220);
			const unsigned int offMeshCol = duRGBA(128,96,0,220);
			dd.begin(DU_DRAW_LINES, 2.0f);
			for (int i = 0; i < m_nstraightPath-1; ++i)
			{
				unsigned int col = 0;
				if (m_straightPathFlags[i] & DT_STRAIGHTPATH_OFFMESH_CONNECTION)
					col = offMeshCol;
				else
					col = pathCol;

				dd.vertex(m_straightPath[i*3], m_straightPath[i*3+1]+0.4f, m_straightPath[i*3+2], col);
				dd.vertex(m_straightPath[(i+1)*3], m_straightPath[(i+1)*3+1]+0.4f, m_straightPath[(i+1)*3+2], col);
			}
			dd.end();
			dd.begin(DU_DRAW_POINTS, 6.0f);
			for (int i = 0; i < m_nstraightPath; ++i)
			{
				unsigned int col = 0;
				if (m_straightPathFlags[i] & DT_STRAIGHTPATH_START)
					col = startCol;
				else if (m_straightPathFlags[i] & DT_STRAIGHTPATH_START)
					col = endCol;
				else if (m_straightPathFlags[i] & DT_STRAIGHTPATH_OFFMESH_CONNECTION)
					col = offMeshCol;
				else
					col = pathCol;
				dd.vertex(m_straightPath[i*3], m_straightPath[i*3+1]+0.4f, m_straightPath[i*3+2], pathCol);
			}
			dd.end();
			dd.depthMask(true);
		}
	}
	else if (toolMode == TOOLMODE_RAYCAST)
	{
		duDebugDrawNavMeshPoly(&dd, navMesh, m_startRef, startCol);

		if (m_nstraightPath)
		{
			for (int i = 1; i < m_npolys; ++i)
				duDebugDrawNavMeshPoly(&dd, navMesh, m_polys[i], pathCol);

			dd.depthMask(false);
			const unsigned int pathCol = m_hitResult ? duRGBA(64,16,0,220) : duRGBA(240,240,240,220);
			dd.begin(DU_DRAW_LINES, 2.0f);
			for (int i = 0; i < m_nstraightPath-1; ++i)
			{
				dd.vertex(m_straightPath[i*3], m_straightPath[i*3+1]+0.4f, m_straightPath[i*3+2], pathCol);
				dd.vertex(m_straightPath[(i+1)*3], m_straightPath[(i+1)*3+1]+0.4f, m_straightPath[(i+1)*3+2], pathCol);
			}
			dd.end();
			dd.begin(DU_DRAW_POINTS, 4.0f);
			for (int i = 0; i < m_nstraightPath; ++i)
				dd.vertex(m_straightPath[i*3], m_straightPath[i*3+1]+0.4f, m_straightPath[i*3+2], pathCol);
			dd.end();

			if (m_hitResult)
			{
				const unsigned int hitCol = duRGBA(0,0,0,128);
				dd.begin(DU_DRAW_LINES, 2.0f);
				dd.vertex(m_hitPos[0], m_hitPos[1] + 0.4f, m_hitPos[2], hitCol);
				dd.vertex(m_hitPos[0] + m_hitNormal[0]*agentRadius,
						  m_hitPos[1] + 0.4f + m_hitNormal[1]*agentRadius,
						  m_hitPos[2] + m_hitNormal[2]*agentRadius, hitCol);
				dd.end();
			}
			dd.depthMask(true);
		}
	}
	else if (toolMode == TOOLMODE_DISTANCE_TO_WALL)
	{
		duDebugDrawNavMeshPoly(&dd, navMesh, m_startRef, startCol);
		dd.depthMask(false);
		duDebugDrawCircle(&dd, m_spos[0], m_spos[1]+agentHeight/2, m_spos[2], m_distanceToWall, duRGBA(64,16,0,220), 2.0f);
		dd.begin(DU_DRAW_LINES, 3.0f);
		dd.vertex(m_hitPos[0], m_hitPos[1] + 0.02f, m_hitPos[2], duRGBA(0,0,0,192));
		dd.vertex(m_hitPos[0], m_hitPos[1] + agentHeight, m_hitPos[2], duRGBA(0,0,0,192));
		dd.end();
		dd.depthMask(true);
	}
	else if (toolMode == TOOLMODE_FIND_POLYS_AROUND)
	{
		for (int i = 0; i < m_npolys; ++i)
		{
			duDebugDrawNavMeshPoly(&dd, navMesh, m_polys[i], pathCol);
			dd.depthMask(false);
			if (m_parent[i])
			{
				float p0[3], p1[3];
				dd.depthMask(false);
				getPolyCenter(navMesh, m_parent[i], p0);
				getPolyCenter(navMesh, m_polys[i], p1);
				duDebugDrawArc(&dd, p0[0],p0[1],p0[2], p1[0],p1[1],p1[2], 0.25f, 0.0f, 0.4f, duRGBA(0,0,0,128), 2.0f);
				dd.depthMask(true);
			}
			dd.depthMask(true);
		}

		dd.depthMask(false);
		const float dx = m_epos[0] - m_spos[0];
		const float dz = m_epos[2] - m_spos[2];
		const float dist = sqrtf(dx*dx + dz*dz);
		duDebugDrawCircle(&dd, m_spos[0], m_spos[1]+agentHeight/2, m_spos[2], dist, duRGBA(64,16,0,220), 2.0f);
		dd.depthMask(true);
	}
}

void ParallelNavMeshTesterTool::Agent::renderOverlay(double* proj, double* model, int* view, int agentId) const
{
	GLdouble x, y, z;
	char buffer[16];

	// Draw start and end point labels
	if (gluProject((GLdouble)m_spos[0], (GLdouble)m_spos[1], (GLdouble)m_spos[2],
								model, proj, view, &x, &y, &z))
	{
		snprintf(buffer, 15, "Start %d", agentId);
		imguiDrawText((int)x, (int)(y-25), IMGUI_ALIGN_CENTER, buffer, imguiRGBA(0,0,0,220));
	}
	if (gluProject((GLdouble)m_epos[0], (GLdouble)m_epos[1], (GLdouble)m_epos[2],
								model, proj, view, &x, &y, &z))
	{
		snprintf(buffer, 15, "End %d", agentId);
		imguiDrawText((int)x, (int)(y-25), IMGUI_ALIGN_CENTER, buffer, imguiRGBA(0,0,0,220));
	}
}

ParallelNavMeshTesterTool::ParallelNavMeshTesterTool() :
		m_sample(0),
		m_navMesh(0),
		m_toolMode(TOOLMODE_PATHFIND_ITER),
		m_tposSet(false),
		m_nagents(0)
{
	m_filter.includeFlags = SAMPLE_POLYFLAGS_ALL;
	m_filter.excludeFlags = 0;

	m_polyPickExt[0] = 2;
	m_polyPickExt[1] = 4;
	m_polyPickExt[2] = 2;
}

void ParallelNavMeshTesterTool::init(class Sample* sample)
{
	m_sample = sample;
	m_navMesh = sample->getNavMesh();
	recalc();

	if (m_navMesh)
	{
		// Change costs.
		m_navMesh->setAreaCost(SAMPLE_POLYAREA_GROUND, 1.0f);
		m_navMesh->setAreaCost(SAMPLE_POLYAREA_WATER, 10.0f);
		m_navMesh->setAreaCost(SAMPLE_POLYAREA_ROAD, 1.0f);
		m_navMesh->setAreaCost(SAMPLE_POLYAREA_DOOR, 1.0f);
		m_navMesh->setAreaCost(SAMPLE_POLYAREA_GRASS, 2.0f);
		m_navMesh->setAreaCost(SAMPLE_POLYAREA_JUMP, 1.5f);
	}

	if (m_toolMode == TOOLMODE_PATHFIND_ITER || m_toolMode == TOOLMODE_PATHFIND_STRAIGHT)
	{
		unsigned char flags = DU_DRAWNAVMESH_CLOSEDLIST;
		if (m_navMesh)
			flags |= DU_DRAWNAVMESH_OFFMESHCONS;
		m_sample->setNavMeshDrawFlags(flags);
	}

}

void ParallelNavMeshTesterTool::reset()
{
	m_nagents = 0;
	m_tposSet = false;
}

void ParallelNavMeshTesterTool::handleMenu()
{
	if (imguiCheck("Pathfind Iter", m_toolMode == TOOLMODE_PATHFIND_ITER))
	{
		m_toolMode = TOOLMODE_PATHFIND_ITER;
		recalc();
	}
	if (imguiCheck("Pathfind Straight", m_toolMode == TOOLMODE_PATHFIND_STRAIGHT))
	{
		m_toolMode = TOOLMODE_PATHFIND_STRAIGHT;
		recalc();
	}
	if (imguiCheck("Distance to Wall", m_toolMode == TOOLMODE_DISTANCE_TO_WALL))
	{
		m_toolMode = TOOLMODE_DISTANCE_TO_WALL;
		recalc();
	}
	if (imguiCheck("Raycast", m_toolMode == TOOLMODE_RAYCAST))
	{
		m_toolMode = TOOLMODE_RAYCAST;
		recalc();
	}
	if (imguiCheck("Find Polys Around", m_toolMode == TOOLMODE_FIND_POLYS_AROUND))
	{
		m_toolMode = TOOLMODE_FIND_POLYS_AROUND;
		recalc();
	}

	imguiSeparator();

	if (imguiButton("Reset"))
	{
		reset();
	}

	imguiSeparator();

	if (m_toolMode == TOOLMODE_PATHFIND_ITER || m_toolMode == TOOLMODE_PATHFIND_STRAIGHT)
	{
		unsigned char flags = DU_DRAWNAVMESH_CLOSEDLIST;
		if (m_navMesh)
			flags |= DU_DRAWNAVMESH_OFFMESHCONS;
		m_sample->setNavMeshDrawFlags(flags);
	}
	else
	{
		unsigned char flags = 0;
		if (m_navMesh)
			flags |= DU_DRAWNAVMESH_OFFMESHCONS;
		m_sample->setNavMeshDrawFlags(flags);
	}

}

void ParallelNavMeshTesterTool::handleClick(const float* p, bool shift)
{
	if (m_nagents == MAX_AGENTS)
		return;

	if (!m_tposSet)
	{
		m_tposSet = true;
		rcVcopy(m_tpos, p);
	}
	else
	{
		// create new agent
		Agent* agent = &m_agents[m_nagents];
		agent->reset();
		rcVcopy(agent->m_spos, m_tpos);
		rcVcopy(agent->m_epos, p);
		++m_nagents;
		m_tposSet = false;
		recalc();
	}
}

void ParallelNavMeshTesterTool::handleRender()
{
	DebugDrawGL dd;

	static const unsigned int tempCol = duRGBA(128,25,0,192);

	const float agentRadius = m_sample->getAgentRadius();
	const float agentHeight = m_sample->getAgentHeight();
	const float agentClimb = m_sample->getAgentClimb();

	dd.depthMask(false);
	if (m_tposSet)
		drawAgent(dd, m_tpos, agentRadius, agentHeight, agentClimb, tempCol);
	dd.depthMask(true);

	if (!m_navMesh)
		return;

	for (int i=0; i<m_nagents; ++i)
	{
		m_agents[i].render(m_toolMode, dd, *m_navMesh, agentRadius, agentHeight, agentClimb);
	}

}

void ParallelNavMeshTesterTool::handleRenderOverlay(double* proj, double* model, int* view)
{
	for (int i=0; i<m_nagents; ++i)
	{
		m_agents[i].renderOverlay(proj, model, view, i);
	}
}

void ParallelNavMeshTesterTool::handleStep()
{
	if (m_toolMode != TOOLMODE_PATHFIND_ITER)
		return;

	if (!m_navMesh)
		return;

	for (int i=0; i<m_nagents; ++i)
	{
		m_agents[i].step(*m_navMesh, m_filter);
	}
}

void ParallelNavMeshTesterTool::recalc()
{
	if (!m_navMesh)
		return;

	for (int i=0; i<m_nagents; ++i)
	{
		m_agents[i].recalc(m_toolMode, *m_navMesh, m_filter, m_polyPickExt);
	}
}
