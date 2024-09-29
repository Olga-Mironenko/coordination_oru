#include <cassert>
#include <iostream>

#include "ConditionsOccupancy.h"
#include "Footprint.h"
#include "PathFinder.h"

enum PLANNING_ALGORITHM { RRTConnect, RRTstar, TRRT, SST, LBTRRT, PRMstar, SPARS, pRRT, LazyRRT };

typedef struct PathPose {
  double x;
  double y;
  double theta;
} PathPose;

extern "C" void cleanupPath(PathPose* path) {
  std::cout << "Cleaning up memory.." << std::endl;
  free(path);
}

PathFinder finder;

extern "C" bool plan_multiple_circles(
  const uint8_t* occupancyMap, int mapWidth, int mapHeight, double mapResolution,
  double mapOriginX, double mapOriginY, double robotRadius,
  const double* xCoords, const double* yCoords, int numCoords,
  double startX, double startY, double startTheta,
  double goalX, double goalY, double goalTheta,
  PathPose** pathOut, int* pathOutLength,
  double distanceBetweenPathPoints, double turningRadius,
  double planningTimeInSecs, PLANNING_ALGORITHM algo
  ) {
  // Overview:
  // - Create a `Footprint`.
  // - Create a `ConditionsOccupancy`.
  // - Call `finder.constructIfNeeded` (if startX == startY == startTheta == goalX == goalY == goalTheta == -1).
  // - Call `finder.query`.

  assert(algo == PRMstar);
  const char* mapId = "grid"; // TODO: add as a parameter
  int numIterations = 100; // TODO: add as a parameter

  srand(1);

  std::shared_ptr<Footprint> footprint = std::make_shared<Footprint>(
    mapResolution, mapOriginX, mapOriginY, robotRadius,
    xCoords, yCoords, numCoords,
    true);
  std::shared_ptr<Conditions> conditions = std::make_shared<ConditionsOccupancy>(
    mapId, numIterations, turningRadius, footprint, occupancyMap, mapWidth, mapHeight);

  if (startX == -1 && startY == -1 && startTheta == -1 && goalX == -1 && goalY == -1 && goalTheta == -1) {
    for (int x = 0; x < mapWidth; x++) {
      int y = 0;
      // OMPL_DEBUG("x=%d, y=%d: occ=%d", x, y, conditions->isPixelOccupied(y, x));
    }

    finder.constructIfNeeded(conditions);
    return false;
  }

  std::shared_ptr<ompl::geometric::PathGeometric> path = finder.query(
    conditions, startX, startY, startTheta, goalX, goalY, goalTheta);

  if (path == nullptr) {
    return false;
  }

  // TODO: extract into a separate function
  double pLen = path->length();
  int numInterpolationPoints = pLen / distanceBetweenPathPoints;
  if (numInterpolationPoints > 0) path->interpolate(numInterpolationPoints);

  const std::vector<ob::State*>& states = path->getStates();
  std::vector<double> reals;

  *pathOutLength = states.size();
  *pathOut = static_cast<PathPose *>(malloc(sizeof(PathPose) * states.size()));
  memset(*pathOut, 0, sizeof(PathPose) * states.size());

  for (std::size_t i = 0; i < path->getStateCount(); ++i) {
      const auto state = path->getState(i)->as<ob::ReedsSheppStateSpace::StateType>();

      (*pathOut)[i].x = state->getX();
      (*pathOut)[i].y = state->getY();
      (*pathOut)[i].theta = state->getYaw();
  }
  return true;
}

extern "C" bool plan_multiple_circles_nomap(double* xCoords, double* yCoords, int numCoords, double startX, double startY, double startTheta, double goalX, double goalY, double goalTheta, PathPose** path, int* pathLength, double distanceBetweenPathPoints, double turningRadius, double planningTimeInSecs, PLANNING_ALGORITHM algo) {
  return false;
}