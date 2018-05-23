//
// Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
//
// This software is provided 'as-is', without any express or implied
// warranty.  In no event will the authors be held liable for any damages
// arising from the use of this software.
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software
//    in a product, an acknowledgment in the product documentation would be
//    appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be
//    misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.
//

#include <sstream>
#include <sys/stat.h>
#include <cstdio>
#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <fstream>
#include <string>

#include "SDL.h"
#include "SDL_opengl.h"
#ifdef __APPLE__
#	include <OpenGL/glu.h>
#else
#	include <GL/glu.h>
#endif

#include <vector>
#include <string>

#include "imgui.h"
#include "imguiRenderGL.h"

#include "Recast.h"
#include "RecastDebugDraw.h"
#include "InputGeom.h"
#include "TestCase.h"
#include "Filelist.h"
#include "Sample_SoloMesh.h"
#include "Sample_TileMesh.h"
#include "Sample_TempObstacles.h"
#include "Sample_Debug.h"
#include "DetourCommon.h"
#include "DetourNavMeshQuery.h"
#include "DetourNavMesh.h"
#include "DetourStatus.h"
#include "NavMeshTesterTool.h"


inline bool exists (const std::string& name) {
    struct stat buffer;
    return (stat (name.c_str(), &buffer) == 0);
}

#ifdef WIN32
#	define snprintf _snprintf
#	define putenv _putenv
#endif

using std::string;
using std::vector;

struct SampleItem
{
	Sample* (*create)();
	const string name;
};
Sample* createSolo() { return new Sample_SoloMesh(); }
Sample* createTile() { return new Sample_TileMesh(); }
Sample* createTempObstacle() { return new Sample_TempObstacles(); }
Sample* createDebug() { return new Sample_Debug(); }
static SampleItem g_samples[] =
{
	{ createSolo, "Solo Mesh" },
	{ createTile, "Tile Mesh" },
	{ createTempObstacle, "Temp Obstacles" },
};
static const int g_nsamples = sizeof(g_samples) / sizeof(SampleItem);

const int MAX_PATH_LEN = 1024;
const int MAX_SMOOTH_LEN = 4096;
const float ZERO_POS[3] = {0.0f, 0.0f, 0.0f};

long filesize(std::string filepath)
{
    std::ifstream in(filepath.c_str(), std::ifstream::ate | std::ifstream::binary);
    return (long)in.tellg();
}

std::string toString(const float* pos, int len = 3) {
    std::ostringstream ss;
    ss << "[";
    for (int i = 0; i < len; i++) {
        if (0 != i) {
            ss << ", ";
        }
        ss << pos[i];
    }
    ss << "]";
    return ss.str();
}

void objToNavmeshBin(std::string& inDir,
                 std::string& outDir,
                 std::string& filename)
{
    // determine input file
    std::string objPath;
    objPath.append(inDir);
    objPath.append("/");
    objPath.append(filename);
    if (!exists(objPath)) return;
    
    BuildContext ctx;
    
    // init sample
//    Sample* sample = g_samples[0].create();  // solo mesh
    Sample* sample = g_samples[1].create();  // tile mesh
    if (!sample) return;
    sample->setContext(&ctx);
    
    // init geom
    InputGeom* geom = new InputGeom;
    
    // load obj file
    if (!geom->load(&ctx, objPath)) return;
    sample->handleMeshChanged(geom);
    std::cout << "obj file loaded " << toString(geom->getMeshBoundsMin()) << " to " << toString(geom->getNavMeshBoundsMax()) << "] \n";
    
    // init build settings
    BuildSettings settings;
    memset(&settings, 0, sizeof(settings));
    rcVcopy(settings.navMeshBMin, geom->getNavMeshBoundsMin());
    rcVcopy(settings.navMeshBMax, geom->getNavMeshBoundsMax());
    // agent characteristics for a SOLDIER
    settings.agentHeight = 2.0f;
    settings.agentMaxClimb = 0.9f;
    settings.agentMaxSlope = 45.0f;
    settings.agentRadius = 0.4f;
    settings.cellHeight = 0.2f;
    settings.cellSize = 0.2f;
    // default values taken from the GUI interface
    settings.detailSampleDist = 6.0f;
    settings.detailSampleMaxError = 1.0f;
    settings.edgeMaxError = 1.3f;
    settings.edgeMaxLen = 12.0f;
    settings.vertsPerPoly = 6.0f;
    settings.partitionType = SAMPLE_PARTITION_WATERSHED;
    settings.regionMergeSize = 20.0f;
    settings.regionMinSize = 8.0f;
    settings.tileSize = 100.0f;
    sample->collectSettings(settings);
    sample->handleSettings();
    geom->saveGeomSet(&settings);

    // build navmesh
    std::cout << "building navmesh\n";
    if (!sample->handleBuild()) {
        std::cout << "failed building navmesh\n";
        return;
    }
    std::cout << "done building navmesh\n";
    
    // save to bin file
    std::string binPath;
    binPath.append(outDir);
    binPath.append("/");
    binPath.append(filename);
    binPath.append(".bin");
    std::cout << "saving bin file\n";
    sample->saveAll(binPath.c_str(), sample->m_navMesh);
    std::cout << "done saving bin file [" << binPath << "] (" << filesize(binPath) << ")\n";
    
    // delete stuff
    delete sample;
    delete geom;
}

void printStatusError(dtStatus status) {
    if (dtStatusSucceed(status)) {
        std::cout << "Success!";
    } else if (dtStatusFailed(status)) {
        if (dtStatusDetail(status, DT_WRONG_MAGIC)) {
            std::cerr << "DT_WRONG_MAGIC\n";
        } else if (dtStatusDetail(status, DT_WRONG_VERSION)) {
            std::cerr << "DT_WRONG_VERSION\n";
        } else if (dtStatusDetail(status, DT_OUT_OF_MEMORY)) {
            std::cerr << "DT_OUT_OF_MEMORY\n";
        } else if (dtStatusDetail(status, DT_INVALID_PARAM)) {
            std::cerr << "DT_INVALID_PARAM\n";
        } else if (dtStatusDetail(status, DT_BUFFER_TOO_SMALL)) {
            std::cerr << "DT_BUFFER_TOO_SMALL\n";
        } else if (dtStatusDetail(status, DT_OUT_OF_NODES)) {
            std::cerr << "DT_OUT_OF_NODES\n";
        } else if (dtStatusDetail(status, DT_PARTIAL_RESULT)) {
            std::cerr << "DT_PARTIAL_RESULT\n";
        } else if (dtStatusDetail(status, DT_ALREADY_OCCUPIED)) {
            std::cerr << "DT_ALREADY_OCCUPIED\n";
        } else {
            std::cerr << "Unknown badness [" << status << "]\n";
        }
    } else {
        std::cerr << "Unexpected status [" << status << "]\n";
    }
}

dtPolyRef getNearestPoly(const float* pos,
                         const float* halfExtents,
                         const dtQueryFilter* filter,
                         const dtNavMeshQuery* navmeshQuery,
                         const dtNavMesh* navmesh,
                         bool verbose = false)
{
    float nearestPt[3];
    dtPolyRef polyRef = -666;
    
    dtStatus status = navmeshQuery->findNearestPoly(pos, halfExtents, filter, &polyRef, nearestPt);
    if (!dtStatusSucceed(status)) {
        std::cerr << "navmeshQuery->findNearestPoly(" << toString(pos) << ", halfExtents, filter, &polyRef, nearestPt)\n";
        printStatusError(status);
        return -1;
    }
    if (!navmesh->isValidPolyRef(polyRef)) {
        std::cerr << "Invalid poly ref [" << polyRef <<"] found for " << toString(pos) << "\n";
        return -2;
    }
    
    if (verbose) {
        std::cout << "Valid poly ref [" << polyRef <<"] found for " << toString(pos) << "\n";
    }
    return polyRef;
}

void getSmoothPath() {
    
}

// copied from NavMeshTesterTool::recalc() "(m_toolMode == TOOLMODE_PATHFIND_FOLLOW)"
static void calcSmoothPath(float *endPos, const dtQueryFilter &filter, int &m_nsmoothPath, float *m_smoothPath, dtNavMesh *navMesh, dtNavMeshQuery &navQuery, dtPolyRef *path, int pathCount, float *startPos, dtPolyRef startRef) {
    {
        // Iterate over the path to find smooth path on the detail mesh surface.
        dtPolyRef polys[MAX_PATH_LEN];
        memcpy(polys, path, sizeof(dtPolyRef)*pathCount);
        int npolys = pathCount;
        
        float iterPos[3], targetPos[3];
        navQuery.closestPointOnPoly(startRef, startPos, iterPos, 0);
        navQuery.closestPointOnPoly(polys[npolys-1], endPos, targetPos, 0);
        
        static const float STEP_SIZE = 0.5f;
        static const float SLOP = 0.01f;
        
        dtVcopy(&m_smoothPath[m_nsmoothPath*3], iterPos);
        m_nsmoothPath++;
        
        // Move towards target a small advancement at a time until target reached or
        // when ran out of memory to store the path.
        while (npolys && m_nsmoothPath < MAX_SMOOTH_LEN)
        {
            // Find location to steer towards.
            float steerPos[3];
            unsigned char steerPosFlag;
            dtPolyRef steerPosRef;
            
            if (!nsNavMeshTesterTool::getSteerTarget(&navQuery, iterPos, targetPos, SLOP,
                                                   polys, npolys, steerPos, steerPosFlag, steerPosRef))
                break;
            
            bool endOfPath = (steerPosFlag & DT_STRAIGHTPATH_END) ? true : false;
            bool offMeshConnection = (steerPosFlag & DT_STRAIGHTPATH_OFFMESH_CONNECTION) ? true : false;
            
            // Find movement delta.
            float delta[3], len;
            dtVsub(delta, steerPos, iterPos);
            len = dtMathSqrtf(dtVdot(delta, delta));
            // If the steer target is end of path or off-mesh link, do not move past the location.
            if ((endOfPath || offMeshConnection) && len < STEP_SIZE)
                len = 1;
            else
                len = STEP_SIZE / len;
            float moveTgt[3];
            dtVmad(moveTgt, iterPos, delta, len);
            
            // Move
            float result[3];
            dtPolyRef visited[16];
            int nvisited = 0;
            navQuery.moveAlongSurface(polys[0], iterPos, moveTgt, &filter,
                                      result, visited, &nvisited, 16);
            
            npolys = nsNavMeshTesterTool::fixupCorridor(polys, npolys, MAX_PATH_LEN, visited, nvisited);
            npolys = nsNavMeshTesterTool::fixupShortcuts(polys, npolys, &navQuery);
            
            float h = 0;
            navQuery.getPolyHeight(polys[0], result, &h);
            result[1] = h;
            dtVcopy(iterPos, result);
            
            // Handle end of path and off-mesh links when close enough.
            if (endOfPath && nsNavMeshTesterTool::inRange(iterPos, steerPos, SLOP, 1.0f))
            {
                // Reached end of path.
                dtVcopy(iterPos, targetPos);
                if (m_nsmoothPath < MAX_SMOOTH_LEN)
                {
                    dtVcopy(&m_smoothPath[m_nsmoothPath*3], iterPos);
                    m_nsmoothPath++;
                }
                break;
            }
            else if (offMeshConnection && nsNavMeshTesterTool::inRange(iterPos, steerPos, SLOP, 1.0f))
            {
                // Reached off-mesh connection.
                float _startPos[3], _endPos[3];
                
                // Advance the path up to and over the off-mesh connection.
                dtPolyRef prevRef = 0, polyRef = polys[0];
                int npos = 0;
                while (npos < npolys && polyRef != steerPosRef)
                {
                    prevRef = polyRef;
                    polyRef = polys[npos];
                    npos++;
                }
                for (int i = npos; i < npolys; ++i)
                    polys[i-npos] = polys[i];
                npolys -= npos;
                
                // Handle the connection.
                dtStatus _status = navMesh->getOffMeshConnectionPolyEndPoints(prevRef, polyRef, _startPos, _endPos);
                if (dtStatusSucceed(_status))
                {
                    if (m_nsmoothPath < MAX_SMOOTH_LEN)
                    {
                        dtVcopy(&m_smoothPath[m_nsmoothPath*3], _startPos);
                        m_nsmoothPath++;
                        // Hack to make the dotted path not visible during off-mesh connection.
                        if (m_nsmoothPath & 1)
                        {
                            dtVcopy(&m_smoothPath[m_nsmoothPath*3], _startPos);
                            m_nsmoothPath++;
                        }
                    }
                    // Move position at the other side of the off-mesh link.
                    dtVcopy(iterPos, _endPos);
                    float eh = 0.0f;
                    navQuery.getPolyHeight(polys[0], iterPos, &eh);
                    iterPos[1] = eh;
                }
            }
            
            // Store results.
            if (m_nsmoothPath < MAX_SMOOTH_LEN)
            {
                dtVcopy(&m_smoothPath[m_nsmoothPath*3], iterPos);
                m_nsmoothPath++;
            }
        }
    }
}

static void getPath(float *startPos, float *endPos, const dtQueryFilter &filter, float *halfExtents,
                    dtPolyRef *path, int &pathLen,
                    float *m_smoothPath, int &m_nsmoothPath,
                    dtNavMesh *navMesh, dtNavMeshQuery &navQuery)
{
    dtPolyRef startRef = getNearestPoly(startPos, halfExtents, &filter, &navQuery, navMesh);
    dtPolyRef endRef = getNearestPoly(endPos, halfExtents, &filter, &navQuery, navMesh);
    if ((0 < startRef) && (0 < endRef)) {
        // find the path!
        std::cout << "Finding path between " << toString(startPos) << " (" << startRef <<") and " << toString(endPos) << " (" << endRef << ").\n";
        dtStatus status = navQuery.findPath(startRef, endRef, startPos, endPos, &filter, path, &pathLen, MAX_PATH_LEN);
        if (!dtStatusSucceed(status)) {
            std::cerr << "Failed to find a path.";
            printStatusError(status);
        } else {
            calcSmoothPath(endPos, filter, m_nsmoothPath, m_smoothPath, navMesh, navQuery, path, pathLen, startPos, startRef);
            if (0 >= m_nsmoothPath) {
                std::cerr << "Unable to calculate smooth path.\n";
            }
        }
    }
}

void printPolygonPath(dtPolyRef *path, int &pathLen, bool verbose = true) {
    std::cout << "Polygon path (" << pathLen << ") is...\n";
    if (verbose) {
        for (int i = 0; i < pathLen; i++) {
            std::cout << i << ": " << path[i] << "\n";
        }
    }
}

void printSmoothedPath(float *m_smoothPath, int &m_nsmoothPath, bool verbose = true) {
    std::cout << "Point path (" << m_nsmoothPath << ") is...\n";
    if (verbose) {
        for (int i = 0; i < m_nsmoothPath; i++) {
            std::cout << i << ": " << toString(&m_smoothPath[i]) << "\n";
        }
    }
}

bool findRandomPoint(const dtQueryFilter &filter, const dtNavMeshQuery &navQuery, float *randomPt)
{
    dtPolyRef randomRef;
    for (int i = 0; i < 100; i++) {
        dtStatus status = navQuery.findRandomPoint(&filter, nsNavMeshTesterTool::frand, &randomRef, randomPt);
        if (dtStatusSucceed(status)) {
            return true;
        }
    }
    
    dtVcopy(randomPt, ZERO_POS);
    return false;
}

void navmeshBinTestPaths(std::string binPath) {
    Sample* sample = g_samples[1].create();  // tile mesh
    if (!sample) return;
    
    // load navmesh bin file
    dtNavMesh* navMesh = sample->loadAll(binPath.c_str());
    
    // use navmesh for pathfinding
    dtNavMeshQuery navQuery;
    navQuery.init(navMesh, 65535);
    
    // output variables
    dtPolyRef path[MAX_PATH_LEN];
    int pathLen = 0;
    float smoothPath[MAX_SMOOTH_LEN*3];
    int smoothPathLen = 0;
    
    // initialize for a path finding query
    float halfExtents[3] = {0.1f, 0.1f, 0.1f};
    // set movment filter
    dtQueryFilter filter;
    filter.setIncludeFlags(SAMPLE_POLYFLAGS_WALK | SAMPLE_POLYFLAGS_DOOR | SAMPLE_POLYFLAGS_JUMP);
    filter.setAreaCost(SAMPLE_POLYAREA_GROUND, 1.0f);
    filter.setAreaCost(SAMPLE_POLYAREA_WATER, 10.0f);
    filter.setAreaCost(SAMPLE_POLYAREA_ROAD, 1.0f);
    filter.setAreaCost(SAMPLE_POLYAREA_DOOR, 1.0f);
    filter.setAreaCost(SAMPLE_POLYAREA_GRASS, 2.0f);
    filter.setAreaCost(SAMPLE_POLYAREA_JUMP, 1.5f);
    
    // determine starting/ending polygon
    float startPos[3] = {-315.5f, 99.0f, -48.1f};
    float endPos[3] = {100.0f, 0.0f, 10.0f};
    
    getPath(startPos, endPos, filter, halfExtents,
            path, pathLen,
            smoothPath, smoothPathLen,
            navMesh, navQuery);
    printPolygonPath(path, pathLen, false);
    printSmoothedPath(smoothPath, smoothPathLen, false);
    
    // run a bunch of random points
    int outerCount = 1;
    while (outerCount <= 1000) {
        if (findRandomPoint(filter, navQuery, startPos) && findRandomPoint(filter, navQuery, endPos)) {
            std::cout << "[" << outerCount << "] \n";
            getPath(startPos, endPos, filter, halfExtents,
                    path, pathLen,
                    smoothPath, smoothPathLen,
                    navMesh, navQuery);
            printPolygonPath(path, pathLen, false);
            printSmoothedPath(smoothPath, smoothPathLen, false);
            outerCount++;
        } else {
            std::cerr << outerCount << ": Failed to find valid random points.\n";
        }
    }
    
    // delete stuff
    delete sample;
}

std::string zeroPadNumber(int num, int width)
{
    std::stringstream ss;
    
    // the number is converted to string with the help of stringstream
    ss << num;
    std::string ret;
    ss >> ret;
    
    // Append zero chars
    unsigned long str_length = ret.length();
    for (unsigned long i = 0; i < (width - str_length); i++)
        ret = "0" + ret;
    return ret;
}

std::string getFilename(int col, int row, int level) {
    std::string filename;
    filename.append("Tile_+");
    filename.append(zeroPadNumber(col, 3));
    filename.append("_+");
    filename.append(zeroPadNumber(row, 3));
    filename.append("_L");
    filename.append(zeroPadNumber(level, 2));
    filename.append(".obj");
    return filename;
}

void mainAlbert(int cols, int rows)
{
    std::string inDir = "/Users/albertlaw/Downloads/Muscat 100m OBJ/Data/L19";
    std::string outDir = "/Users/albertlaw/code/recastnavigation/RecastDemo/Bin/Tile";
    for (int c = 0; c <= cols; c++) {
        for (int r = 0; r <= rows; r++) {
            std::string filename = getFilename(c, r, 19);
            objToNavmeshBin(inDir, outDir, filename);
        }
    }
}

void mainOneBigOne() {
    std::string inDir = "/Users/albertlaw/Downloads/Muscat 100m OBJ/Data/L19";
    std::string outDir = "/Users/albertlaw/code/recastnavigation/RecastDemo/Bin/Tile";
    std::string objFilename = "L19.obj";
    std::string binFilename = "L19.obj.bin";
//    objToNavmeshBin(inDir, outDir, objFilename);
    navmeshBinTestPaths(outDir + "/" + binFilename);
    std::cout << "exiting mainOneBigOne\n";
}

int main(int /*argc*/, char** /*argv*/)
{
//    mainAlbert(18, 18);
    mainOneBigOne();
    
	// Init SDL
	if (SDL_Init(SDL_INIT_EVERYTHING) != 0)
	{
		printf("Could not initialise SDL.\nError: %s\n", SDL_GetError());
		return -1;
	}

	// Enable depth buffer.
	SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
	SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
	
	// Set color channel depth.
	SDL_GL_SetAttribute(SDL_GL_RED_SIZE, 8);
	SDL_GL_SetAttribute(SDL_GL_GREEN_SIZE, 8);
	SDL_GL_SetAttribute(SDL_GL_BLUE_SIZE, 8);
	SDL_GL_SetAttribute(SDL_GL_ALPHA_SIZE, 8);
	
	// 4x MSAA.
	SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 1);
	SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, 4);

	SDL_DisplayMode displayMode;
	SDL_GetCurrentDisplayMode(0, &displayMode);

	bool presentationMode = false;
	Uint32 flags = SDL_WINDOW_OPENGL;
	int width;
	int height;
	if (presentationMode)
	{
		// Create a fullscreen window at the native resolution.
		width = displayMode.w;
		height = displayMode.h;
		flags |= SDL_WINDOW_FULLSCREEN;
	}
	else
	{
		float aspect = 16.0f / 9.0f;
		width = rcMin(displayMode.w, (int)(displayMode.h * aspect)) - 80;
		height = displayMode.h - 80;
	}
	
	SDL_Window* window;
	SDL_Renderer* renderer;
	int errorCode = SDL_CreateWindowAndRenderer(width, height, flags, &window, &renderer);

	if (errorCode != 0 || !window || !renderer)
	{
		printf("Could not initialise SDL opengl\nError: %s\n", SDL_GetError());
		return -1;
	}

	SDL_SetWindowPosition(window, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED);
	SDL_GL_CreateContext(window);

	if (!imguiRenderGLInit("DroidSans.ttf"))
	{
		printf("Could not init GUI renderer.\n");
		SDL_Quit();
		return -1;
	}
	
	float t = 0.0f;
	float timeAcc = 0.0f;
	Uint32 prevFrameTime = SDL_GetTicks();
	int mousePos[2] = {0, 0};
	int origMousePos[2] = {0, 0}; // Used to compute mouse movement totals across frames.
	
	float cameraEulers[] = {45, -45};
	float cameraPos[] = {0, 0, 0};
	float camr = 1000;
	float origCameraEulers[] = {0, 0}; // Used to compute rotational changes across frames.
	
	float moveFront = 0.0f, moveBack = 0.0f, moveLeft = 0.0f, moveRight = 0.0f, moveUp = 0.0f, moveDown = 0.0f;
	
	float scrollZoom = 0;
	bool rotate = false;
	bool movedDuringRotate = false;
	float rayStart[3];
	float rayEnd[3];
	bool mouseOverMenu = false;
	
	bool showMenu = !presentationMode;
	bool showLog = false;
	bool showTools = true;
	bool showLevels = false;
	bool showSample = false;
	bool showTestCases = false;

	// Window scroll positions.
	int propScroll = 0;
	int logScroll = 0;
	int toolsScroll = 0;
	
	string sampleName = "Choose Sample...";
	
	vector<string> files;
	const string meshesFolder = "Meshes";
	string meshName = "Choose Mesh...";
	
	float markerPosition[3] = {0, 0, 0};
	bool markerPositionSet = false;
	
	InputGeom* geom = 0;
	Sample* sample = 0;

	const string testCasesFolder = "TestCases";
	TestCase* test = 0;

	BuildContext ctx;
	
	// Fog.
	float fogColor[4] = { 0.32f, 0.31f, 0.30f, 1.0f };
	glEnable(GL_FOG);
	glFogi(GL_FOG_MODE, GL_LINEAR);
	glFogf(GL_FOG_START, camr * 0.1f);
	glFogf(GL_FOG_END, camr * 1.25f);
	glFogfv(GL_FOG_COLOR, fogColor);
	
	glEnable(GL_CULL_FACE);
	glDepthFunc(GL_LEQUAL);
	
	bool done = false;
	while(!done)
	{
		// Handle input events.
		int mouseScroll = 0;
		bool processHitTest = false;
		bool processHitTestShift = false;
		SDL_Event event;
		
		while (SDL_PollEvent(&event))
		{
			switch (event.type)
			{
				case SDL_KEYDOWN:
					// Handle any key presses here.
					if (event.key.keysym.sym == SDLK_ESCAPE)
					{
						done = true;
					}
					else if (event.key.keysym.sym == SDLK_t)
					{
						showLevels = false;
						showSample = false;
						showTestCases = true;
						scanDirectory(testCasesFolder, ".txt", files);
					}
					else if (event.key.keysym.sym == SDLK_TAB)
					{
						showMenu = !showMenu;
					}
					else if (event.key.keysym.sym == SDLK_SPACE)
					{
						if (sample)
							sample->handleToggle();
					}
					else if (event.key.keysym.sym == SDLK_1)
					{
						if (sample)
							sample->handleStep();
					}
					else if (event.key.keysym.sym == SDLK_9)
					{
						if (sample && geom)
						{
							string savePath = meshesFolder + "/";
							BuildSettings settings;
							memset(&settings, 0, sizeof(settings));

							rcVcopy(settings.navMeshBMin, geom->getNavMeshBoundsMin());
							rcVcopy(settings.navMeshBMax, geom->getNavMeshBoundsMax());

							sample->collectSettings(settings);

							geom->saveGeomSet(&settings);
						}
					}
					break;
				
				case SDL_MOUSEWHEEL:
					if (event.wheel.y < 0)
					{
						// wheel down
						if (mouseOverMenu)
						{
							mouseScroll++;
						}
						else
						{
							scrollZoom += 1.0f;
						}
					}
					else
					{
						if (mouseOverMenu)
						{
							mouseScroll--;
						}
						else
						{
							scrollZoom -= 1.0f;
						}
					}
					break;
				case SDL_MOUSEBUTTONDOWN:
					if (event.button.button == SDL_BUTTON_RIGHT)
					{
						if (!mouseOverMenu)
						{
							// Rotate view
							rotate = true;
							movedDuringRotate = false;
							origMousePos[0] = mousePos[0];
							origMousePos[1] = mousePos[1];
							origCameraEulers[0] = cameraEulers[0];
							origCameraEulers[1] = cameraEulers[1];
						}
					}
					break;
					
				case SDL_MOUSEBUTTONUP:
					// Handle mouse clicks here.
					if (event.button.button == SDL_BUTTON_RIGHT)
					{
						rotate = false;
						if (!mouseOverMenu)
						{
							if (!movedDuringRotate)
							{
								processHitTest = true;
								processHitTestShift = true;
							}
						}
					}
					else if (event.button.button == SDL_BUTTON_LEFT)
					{
						if (!mouseOverMenu)
						{
							processHitTest = true;
							processHitTestShift = (SDL_GetModState() & KMOD_SHIFT) ? true : false;
						}
					}
					
					break;
					
				case SDL_MOUSEMOTION:
					mousePos[0] = event.motion.x;
					mousePos[1] = height-1 - event.motion.y;
					
					if (rotate)
					{
						int dx = mousePos[0] - origMousePos[0];
						int dy = mousePos[1] - origMousePos[1];
						cameraEulers[0] = origCameraEulers[0] - dy * 0.25f;
						cameraEulers[1] = origCameraEulers[1] + dx * 0.25f;
						if (dx * dx + dy * dy > 3 * 3)
						{
							movedDuringRotate = true;
						}
					}
					break;
					
				case SDL_QUIT:
					done = true;
					break;
					
				default:
					break;
			}
		}

		unsigned char mouseButtonMask = 0;
		if (SDL_GetMouseState(0, 0) & SDL_BUTTON_LMASK)
			mouseButtonMask |= IMGUI_MBUT_LEFT;
		if (SDL_GetMouseState(0, 0) & SDL_BUTTON_RMASK)
			mouseButtonMask |= IMGUI_MBUT_RIGHT;
		
		Uint32 time = SDL_GetTicks();
		float dt = (time - prevFrameTime) / 1000.0f;
		prevFrameTime = time;
		
		t += dt;

		// Hit test mesh.
		if (processHitTest && geom && sample)
		{
			float hitTime;
			bool hit = geom->raycastMesh(rayStart, rayEnd, hitTime);
			
			if (hit)
			{
				if (SDL_GetModState() & KMOD_CTRL)
				{
					// Marker
					markerPositionSet = true;
					markerPosition[0] = rayStart[0] + (rayEnd[0] - rayStart[0]) * hitTime;
					markerPosition[1] = rayStart[1] + (rayEnd[1] - rayStart[1]) * hitTime;
					markerPosition[2] = rayStart[2] + (rayEnd[2] - rayStart[2]) * hitTime;
				}
				else
				{
					float pos[3];
					pos[0] = rayStart[0] + (rayEnd[0] - rayStart[0]) * hitTime;
					pos[1] = rayStart[1] + (rayEnd[1] - rayStart[1]) * hitTime;
					pos[2] = rayStart[2] + (rayEnd[2] - rayStart[2]) * hitTime;
					sample->handleClick(rayStart, pos, processHitTestShift);
				}
			}
			else
			{
				if (SDL_GetModState() & KMOD_CTRL)
				{
					// Marker
					markerPositionSet = false;
				}
			}
		}
		
		// Update sample simulation.
		const float SIM_RATE = 20;
		const float DELTA_TIME = 1.0f / SIM_RATE;
		timeAcc = rcClamp(timeAcc + dt, -1.0f, 1.0f);
		int simIter = 0;
		while (timeAcc > DELTA_TIME)
		{
			timeAcc -= DELTA_TIME;
			if (simIter < 5 && sample)
			{
				sample->handleUpdate(DELTA_TIME);
			}
			simIter++;
		}

		// Clamp the framerate so that we do not hog all the CPU.
		const float MIN_FRAME_TIME = 1.0f / 40.0f;
		if (dt < MIN_FRAME_TIME)
		{
			int ms = (int)((MIN_FRAME_TIME - dt) * 1000.0f);
			if (ms > 10) ms = 10;
			if (ms >= 0) SDL_Delay(ms);
		}
		
		// Set the viewport.
		glViewport(0, 0, width, height);
		GLint viewport[4];
		glGetIntegerv(GL_VIEWPORT, viewport);
		
		// Clear the screen
		glClearColor(0.3f, 0.3f, 0.32f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glDisable(GL_TEXTURE_2D);
		glEnable(GL_DEPTH_TEST);
		
		// Compute the projection matrix.
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		gluPerspective(50.0f, (float)width/(float)height, 1.0f, camr);
		GLdouble projectionMatrix[16];
		glGetDoublev(GL_PROJECTION_MATRIX, projectionMatrix);
		
		// Compute the modelview matrix.
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		glRotatef(cameraEulers[0], 1, 0, 0);
		glRotatef(cameraEulers[1], 0, 1, 0);
		glTranslatef(-cameraPos[0], -cameraPos[1], -cameraPos[2]);
		GLdouble modelviewMatrix[16];
		glGetDoublev(GL_MODELVIEW_MATRIX, modelviewMatrix);
		
		// Get hit ray position and direction.
		GLdouble x, y, z;
		gluUnProject(mousePos[0], mousePos[1], 0.0f, modelviewMatrix, projectionMatrix, viewport, &x, &y, &z);
		rayStart[0] = (float)x;
		rayStart[1] = (float)y;
		rayStart[2] = (float)z;
		gluUnProject(mousePos[0], mousePos[1], 1.0f, modelviewMatrix, projectionMatrix, viewport, &x, &y, &z);
		rayEnd[0] = (float)x;
		rayEnd[1] = (float)y;
		rayEnd[2] = (float)z;
		
		// Handle keyboard movement.
		const Uint8* keystate = SDL_GetKeyboardState(NULL);
		moveFront	= rcClamp(moveFront	+ dt * 4 * ((keystate[SDL_SCANCODE_W] || keystate[SDL_SCANCODE_UP		]) ? 1 : -1), 0.0f, 1.0f);
		moveLeft	= rcClamp(moveLeft	+ dt * 4 * ((keystate[SDL_SCANCODE_A] || keystate[SDL_SCANCODE_LEFT		]) ? 1 : -1), 0.0f, 1.0f);
		moveBack	= rcClamp(moveBack	+ dt * 4 * ((keystate[SDL_SCANCODE_S] || keystate[SDL_SCANCODE_DOWN		]) ? 1 : -1), 0.0f, 1.0f);
		moveRight	= rcClamp(moveRight	+ dt * 4 * ((keystate[SDL_SCANCODE_D] || keystate[SDL_SCANCODE_RIGHT	]) ? 1 : -1), 0.0f, 1.0f);
		moveUp		= rcClamp(moveUp	+ dt * 4 * ((keystate[SDL_SCANCODE_Q] || keystate[SDL_SCANCODE_PAGEUP	]) ? 1 : -1), 0.0f, 1.0f);
		moveDown	= rcClamp(moveDown	+ dt * 4 * ((keystate[SDL_SCANCODE_E] || keystate[SDL_SCANCODE_PAGEDOWN	]) ? 1 : -1), 0.0f, 1.0f);
		
		float keybSpeed = 22.0f;
		if (SDL_GetModState() & KMOD_SHIFT)
		{
			keybSpeed *= 4.0f;
		}
		
		float movex = (moveRight - moveLeft) * keybSpeed * dt;
		float movey = (moveBack - moveFront) * keybSpeed * dt + scrollZoom * 2.0f;
		scrollZoom = 0;
		
		cameraPos[0] += movex * (float)modelviewMatrix[0];
		cameraPos[1] += movex * (float)modelviewMatrix[4];
		cameraPos[2] += movex * (float)modelviewMatrix[8];
		
		cameraPos[0] += movey * (float)modelviewMatrix[2];
		cameraPos[1] += movey * (float)modelviewMatrix[6];
		cameraPos[2] += movey * (float)modelviewMatrix[10];

		cameraPos[1] += (moveUp - moveDown) * keybSpeed * dt;

		glEnable(GL_FOG);

		if (sample)
			sample->handleRender();
		if (test)
			test->handleRender();
		
		glDisable(GL_FOG);
		
		// Render GUI
		glDisable(GL_DEPTH_TEST);
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		gluOrtho2D(0, width, 0, height);
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		
		mouseOverMenu = false;
		
		imguiBeginFrame(mousePos[0], mousePos[1], mouseButtonMask, mouseScroll);
		
		if (sample)
		{
			sample->handleRenderOverlay((double*)projectionMatrix, (double*)modelviewMatrix, (int*)viewport);
		}
		if (test)
		{
			if (test->handleRenderOverlay((double*)projectionMatrix, (double*)modelviewMatrix, (int*)viewport))
				mouseOverMenu = true;
		}

		// Help text.
		if (showMenu)
		{
			const char msg[] = "W/S/A/D: Move  RMB: Rotate";
			imguiDrawText(280, height-20, IMGUI_ALIGN_LEFT, msg, imguiRGBA(255,255,255,128));
		}
		
		if (showMenu)
		{
			if (imguiBeginScrollArea("Properties", width-250-10, 10, 250, height-20, &propScroll))
				mouseOverMenu = true;

			if (imguiCheck("Show Log", showLog))
				showLog = !showLog;
			if (imguiCheck("Show Tools", showTools))
				showTools = !showTools;

			imguiSeparator();
			imguiLabel("Sample");
			if (imguiButton(sampleName.c_str()))
			{
				if (showSample)
				{
					showSample = false;
				}
				else
				{
					showSample = true;
					showLevels = false;
					showTestCases = false;
				}
			}
			
			imguiSeparator();
			imguiLabel("Input Mesh");
			if (imguiButton(meshName.c_str()))
			{
				if (showLevels)
				{
					showLevels = false;
				}
				else
				{
					showSample = false;
					showTestCases = false;
					showLevels = true;
					scanDirectory(meshesFolder, ".obj", files);
					scanDirectoryAppend(meshesFolder, ".gset", files);
				}
			}
			if (geom)
			{
				char text[64];
				snprintf(text, 64, "Verts: %.1fk  Tris: %.1fk",
						 geom->getMesh()->getVertCount()/1000.0f,
						 geom->getMesh()->getTriCount()/1000.0f);
				imguiValue(text);
			}
			imguiSeparator();

			if (geom && sample)
			{
				imguiSeparatorLine();
				
				sample->handleSettings();

				if (imguiButton("Build"))
				{
					ctx.resetLog();
					if (!sample->handleBuild())
					{
						showLog = true;
						logScroll = 0;
					}
					ctx.dumpLog("Build log %s:", meshName.c_str());
					
					// Clear test.
					delete test;
					test = 0;
				}

				imguiSeparator();
			}
			
			if (sample)
			{
				imguiSeparatorLine();
				sample->handleDebugMode();
			}

			imguiEndScrollArea();
		}
		
		// Sample selection dialog.
		if (showSample)
		{
			static int levelScroll = 0;
			if (imguiBeginScrollArea("Choose Sample", width-10-250-10-200, height-10-250, 200, 250, &levelScroll))
				mouseOverMenu = true;

			Sample* newSample = 0;
			for (int i = 0; i < g_nsamples; ++i)
			{
				if (imguiItem(g_samples[i].name.c_str()))
				{
					newSample = g_samples[i].create();
					if (newSample)
						sampleName = g_samples[i].name;
				}
			}
			if (newSample)
			{
				delete sample;
				sample = newSample;
				sample->setContext(&ctx);
				if (geom)
				{
					sample->handleMeshChanged(geom);
				}
				showSample = false;
			}

			if (geom || sample)
			{
				const float* bmin = 0;
				const float* bmax = 0;
				if (geom)
				{
					bmin = geom->getNavMeshBoundsMin();
					bmax = geom->getNavMeshBoundsMax();
				}
				// Reset camera and fog to match the mesh bounds.
				if (bmin && bmax)
				{
					camr = sqrtf(rcSqr(bmax[0]-bmin[0]) +
								 rcSqr(bmax[1]-bmin[1]) +
								 rcSqr(bmax[2]-bmin[2])) / 2;
					cameraPos[0] = (bmax[0] + bmin[0]) / 2 + camr;
					cameraPos[1] = (bmax[1] + bmin[1]) / 2 + camr;
					cameraPos[2] = (bmax[2] + bmin[2]) / 2 + camr;
					camr *= 3;
				}
				cameraEulers[0] = 45;
				cameraEulers[1] = -45;
				glFogf(GL_FOG_START, camr*0.1f);
				glFogf(GL_FOG_END, camr*1.25f);
			}
			
			imguiEndScrollArea();
		}
		
		// Level selection dialog.
		if (showLevels)
		{
			static int levelScroll = 0;
			if (imguiBeginScrollArea("Choose Level", width - 10 - 250 - 10 - 200, height - 10 - 450, 200, 450, &levelScroll))
				mouseOverMenu = true;
			
			vector<string>::const_iterator fileIter = files.begin();
			vector<string>::const_iterator filesEnd = files.end();
			vector<string>::const_iterator levelToLoad = filesEnd;
			for (; fileIter != filesEnd; ++fileIter)
			{
				if (imguiItem(fileIter->c_str()))
				{
					levelToLoad = fileIter;
				}
			}
			
			if (levelToLoad != filesEnd)
			{
				meshName = *levelToLoad;
				showLevels = false;
				
				delete geom;
				geom = 0;
				
				string path = meshesFolder + "/" + meshName;
				
				geom = new InputGeom;
//                std::string _path = "/Users/albertlaw/code/recastnavigation/RecastDemo/Bin/Meshes/Tile_+009_+008_L22.obj";
				if (!geom->load(&ctx, path))
				{
					delete geom;
					geom = 0;

					// Destroy the sample if it already had geometry loaded, as we've just deleted it!
					if (sample && sample->getInputGeom())
					{
						delete sample;
						sample = 0;
					}
					
					showLog = true;
					logScroll = 0;
					ctx.dumpLog("Geom load log %s:", meshName.c_str());
				}
				if (sample && geom)
				{
					sample->handleMeshChanged(geom);
				}

				if (geom || sample)
				{
					const float* bmin = 0;
					const float* bmax = 0;
					if (geom)
					{
						bmin = geom->getNavMeshBoundsMin();
						bmax = geom->getNavMeshBoundsMax();
					}
					// Reset camera and fog to match the mesh bounds.
					if (bmin && bmax)
					{
						camr = sqrtf(rcSqr(bmax[0]-bmin[0]) +
									 rcSqr(bmax[1]-bmin[1]) +
									 rcSqr(bmax[2]-bmin[2])) / 2;
						cameraPos[0] = (bmax[0] + bmin[0]) / 2 + camr;
						cameraPos[1] = (bmax[1] + bmin[1]) / 2 + camr;
						cameraPos[2] = (bmax[2] + bmin[2]) / 2 + camr;
						camr *= 3;
					}
					cameraEulers[0] = 45;
					cameraEulers[1] = -45;
					glFogf(GL_FOG_START, camr * 0.1f);
					glFogf(GL_FOG_END, camr * 1.25f);
				}
			}
			
			imguiEndScrollArea();
			
		}
		
		// Test cases
		if (showTestCases)
		{
			static int testScroll = 0;
			if (imguiBeginScrollArea("Choose Test To Run", width-10-250-10-200, height-10-450, 200, 450, &testScroll))
				mouseOverMenu = true;

			vector<string>::const_iterator fileIter = files.begin();
			vector<string>::const_iterator filesEnd = files.end();
			vector<string>::const_iterator testToLoad = filesEnd;
			for (; fileIter != filesEnd; ++fileIter)
			{
				if (imguiItem(fileIter->c_str()))
				{
					testToLoad = fileIter;
				}
			}
			
			if (testToLoad != filesEnd)
			{
				string path = testCasesFolder + "/" + *testToLoad;
				test = new TestCase;
				if (test)
				{
					// Load the test.
					if (!test->load(path))
					{
						delete test;
						test = 0;
					}

					// Create sample
					Sample* newSample = 0;
					for (int i = 0; i < g_nsamples; ++i)
					{
						if (g_samples[i].name == test->getSampleName())
						{
							newSample = g_samples[i].create();
							if (newSample)
								sampleName = g_samples[i].name;
						}
					}

					delete sample;
					sample = newSample;

					if (sample)
					{
						sample->setContext(&ctx);
						showSample = false;
					}

					// Load geom.
					meshName = test->getGeomFileName();
					
					
					path = meshesFolder + "/" + meshName;
					
					delete geom;
					geom = new InputGeom;
					if (!geom || !geom->load(&ctx, path))
					{
						delete geom;
						geom = 0;
						delete sample;
						sample = 0;
						showLog = true;
						logScroll = 0;
						ctx.dumpLog("Geom load log %s:", meshName.c_str());
					}
					if (sample && geom)
					{
						sample->handleMeshChanged(geom);
					}

					// This will ensure that tile & poly bits are updated in tiled sample.
					if (sample)
						sample->handleSettings();

					ctx.resetLog();
					if (sample && !sample->handleBuild())
					{
						ctx.dumpLog("Build log %s:", meshName.c_str());
					}
					
					if (geom || sample)
					{
						const float* bmin = 0;
						const float* bmax = 0;
						if (geom)
						{
							bmin = geom->getNavMeshBoundsMin();
							bmax = geom->getNavMeshBoundsMax();
						}
						// Reset camera and fog to match the mesh bounds.
						if (bmin && bmax)
						{
							camr = sqrtf(rcSqr(bmax[0] - bmin[0]) +
										 rcSqr(bmax[1] - bmin[1]) +
										 rcSqr(bmax[2] - bmin[2])) / 2;
							cameraPos[0] = (bmax[0] + bmin[0]) / 2 + camr;
							cameraPos[1] = (bmax[1] + bmin[1]) / 2 + camr;
							cameraPos[2] = (bmax[2] + bmin[2]) / 2 + camr;
							camr *= 3;
						}
						cameraEulers[0] = 45;
						cameraEulers[1] = -45;
						glFogf(GL_FOG_START, camr * 0.2f);
						glFogf(GL_FOG_END, camr * 1.25f);
					}
					
					// Do the tests.
					if (sample)
						test->doTests(sample->getNavMesh(), sample->getNavMeshQuery());
				}
			}				
				
			imguiEndScrollArea();
		}

		
		// Log
		if (showLog && showMenu)
		{
			if (imguiBeginScrollArea("Log", 250 + 20, 10, width - 300 - 250, 200, &logScroll))
				mouseOverMenu = true;
			for (int i = 0; i < ctx.getLogCount(); ++i)
				imguiLabel(ctx.getLogText(i));
			imguiEndScrollArea();
		}
		
		// Left column tools menu
		if (!showTestCases && showTools && showMenu) // && geom && sample)
		{
			if (imguiBeginScrollArea("Tools", 10, 10, 250, height - 20, &toolsScroll))
				mouseOverMenu = true;

			if (sample)
				sample->handleTools();
			
			imguiEndScrollArea();
		}
		
		// Marker
		if (markerPositionSet && gluProject((GLdouble)markerPosition[0], (GLdouble)markerPosition[1], (GLdouble)markerPosition[2],
								  modelviewMatrix, projectionMatrix, viewport, &x, &y, &z))
		{
			// Draw marker circle
			glLineWidth(5.0f);
			glColor4ub(240,220,0,196);
			glBegin(GL_LINE_LOOP);
			const float r = 25.0f;
			for (int i = 0; i < 20; ++i)
			{
				const float a = (float)i / 20.0f * RC_PI*2;
				const float fx = (float)x + cosf(a)*r;
				const float fy = (float)y + sinf(a)*r;
				glVertex2f(fx,fy);
			}
			glEnd();
			glLineWidth(1.0f);
		}
		
		imguiEndFrame();
		imguiRenderGLDraw();		
		
		glEnable(GL_DEPTH_TEST);
		SDL_GL_SwapWindow(window);
	}
	
	imguiRenderGLDestroy();
	
	SDL_Quit();
	
	delete sample;
	delete geom;
	
	return 0;
}
