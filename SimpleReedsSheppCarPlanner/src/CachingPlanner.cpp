#include <cassert>
#include <iostream>

#include "ConditionsOccupancy.h"
#include "Footprint.h"
#include "PathFinder.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

struct PathPose {
  double x;
  double y;
  double theta;
};

void copyPath(
  const std::shared_ptr<og::PathGeometric> &path,
  PathPose** pathOut, int* pathOutLength)
{
  const std::vector<ob::State*>& states = path->getStates();
  std::vector<double> reals;

  *pathOutLength = states.size();
  *pathOut = new PathPose[states.size()];

  for (std::size_t i = 0; i < path->getStateCount(); ++i) {
    const auto state = path->getState(i)->as<ob::ReedsSheppStateSpace::StateType>();

    (*pathOut)[i].x = state->getX();
    (*pathOut)[i].y = state->getY();
    (*pathOut)[i].theta = state->getYaw();
  }
}

extern "C" void cleanupPath(const PathPose* path) {
  // std::cout << "Cleaning up memory.." << std::endl;
  delete[] path;
}

PathFinder finder;
bool isRNGSeeded = false;

extern "C" bool plan(
  const char* mapId, const uint8_t* occupancyMap, int mapWidth, int mapHeight, double mapResolution,
  double mapOriginX, double mapOriginY, double robotRadius,
  const double* xCoords, const double* yCoords, int numCoords,
  const double startX, const double startY, const double startTheta,
  const double goalX, const double goalY, const double goalTheta,
  PathPose** pathOut, int* pathOutLength,
  const double distanceBetweenPathPoints, double turningRadius,
  int numIterationsConstruction, int numIterationsSimplification
  ) {
  // Overview:
  // - Create a `Footprint`.
  // - Create a `ConditionsOccupancy`.
  // - Call `finder.constructIfNeeded` (if startX == startY == startTheta == goalX == goalY == goalTheta == -1).
  // - Call `finder.query`.

  ompl::msg::setLogLevel(ompl::msg::LOG_INFO);
  srand(1);
  if (! isRNGSeeded) {
    ompl::RNG::setSeed(1);
    isRNGSeeded = true;
  }

  auto footprint = std::make_shared<Footprint>(
    mapResolution, mapOriginX, mapOriginY, robotRadius,
    xCoords, yCoords, numCoords,
    true);
  const std::shared_ptr<Conditions> conditions = std::make_shared<ConditionsOccupancy>(
    mapId, numIterationsConstruction, numIterationsSimplification, turningRadius,
    footprint, occupancyMap, mapWidth, mapHeight);

  if (startX == -1 && startY == -1 && startTheta == -1 && goalX == -1 && goalY == -1 && goalTheta == -1) {
    for (int x = 0; x < mapWidth; x++) {
      int y = 0;
      // OMPL_DEBUG("x=%d, y=%d: occ=%d", x, y, conditions->isPixelOccupied(y, x));
    }

    finder.constructIfNeeded(conditions);
    return false;
  }

  const std::shared_ptr<og::PathGeometric> path = finder.query(
    conditions, startX, startY, startTheta, goalX, goalY, goalTheta);

  if (path == nullptr) {
    return false;
  }

  const double pLen = path->length();
  const int numInterpolationPoints = pLen / distanceBetweenPathPoints;
  if (numInterpolationPoints > 0) path->interpolate(numInterpolationPoints);

  copyPath(path, pathOut, pathOutLength);
  return true;
}