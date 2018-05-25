//
//  NavMeshTesterTool_subset.hpp
//  RecastDemo
//

#ifndef NavMeshTesterTool_subset_hpp
#define NavMeshTesterTool_subset_hpp

#include "DetourCommon.h"
#include "DetourNavMeshQuery.h"

#endif /* NavMeshTesterTool_subset_hpp */

const int MAX_PATH_LEN = 1024;
const int MAX_SMOOTH_PATH_LEN = 4096;

struct FindPathResult {
    dtStatus status;
    dtPolyRef path[MAX_PATH_LEN];
    int pathCount;
};

struct SmoothPathResult {
    float path[3 * MAX_SMOOTH_PATH_LEN];
    int pathCount;
};

SmoothPathResult getSmoothPath(float* startPos, dtPolyRef startRef, float* endPos,
                               FindPathResult* path,
                               const dtQueryFilter* filter,
                               dtNavMesh* navMesh, dtNavMeshQuery* navQuery);
