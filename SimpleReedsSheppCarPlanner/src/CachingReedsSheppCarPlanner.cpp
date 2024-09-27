#include <fstream>
#include <iostream>

#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

#include "MultipleCircleStateValidityChecker.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

enum PLANNING_ALGORITHM { RRTConnect, RRTstar, TRRT, SST, LBTRRT, PRMstar, SPARS, pRRT, LazyRRT };

typedef struct PathPose {
  double x;
  double y;
  double theta;
} PathPose;

int indexRun = 0;
og::SimpleSetupPtr ss_ = nullptr;
std::shared_ptr<og::PRMstar> planner_ = nullptr;

extern "C" void cleanupPath(PathPose* path) {
  std::cout << "Cleaning up memory.." << std::endl;
  free(path);
}

void dumpPlannerData(const char* filename) {
  ob::PlannerData pd(ss_->getSpaceInformation());
  ss_->getPlannerData(pd);

  std::ofstream file(filename);
  pd.printGraphML(file);
  file.close();
}

void initialize(
  uint8_t* occupancyMap, int mapWidth, int mapHeight, double mapResolution,
  double mapOriginX, double mapOriginY, double robotRadius,
  const double* xCoords, const double* yCoords, int numCoords,
  double turningRadius
) {
  ob::StateSpacePtr space(new ob::ReedsSheppStateSpace(turningRadius));

  std::cout << "Using " << mapWidth << "x" << mapHeight << " occupancy map for validity checking" << std::endl;

  ob::RealVectorBounds bounds(2);
  bounds.low[0] = mapOriginX;
  bounds.low[1] = mapOriginY;
  bounds.high[0] = mapOriginX+mapWidth*mapResolution;
  bounds.high[1] = mapOriginY+mapHeight*mapResolution;

  space->as<ob::SE2StateSpace>()->setBounds(bounds);
  std::cout << "Bounds are [(" << bounds.low[0] << "," << bounds.low[1] << "),(" << bounds.high[0] << "," << bounds.high[1] << ")]" << std::endl;

  // define a simple setup class
  ss_ = std::make_shared<og::SimpleSetup>(space);

  // set state validity checking for this space
  ob::SpaceInformationPtr si(ss_->getSpaceInformation());
  si->setStateValidityChecker(std::make_shared<MultipleCircleStateValidityChecker>(
    si, occupancyMap, mapWidth, mapHeight, mapResolution, mapOriginX, mapOriginY, robotRadius,
    xCoords, yCoords, numCoords));

  planner_ = std::make_shared<og::PRMstar>(si);
  ss_->setPlanner(planner_);

  ss_->getSpaceInformation()->setStateValidityCheckingResolution(mapResolution / space->getMaximumExtent()); // 1 pixel
  ss_->setup();
  ss_->print(); // this call is optional, but we put it in to get more output information
}

extern "C" bool plan_multiple_circles(
  // TODO: add `char* mapId`
  const uint8_t* occupancyMap, int mapWidth, int mapHeight, double mapResolution,
  double mapOriginX, double mapOriginY, double robotRadius,
  const double* xCoords, const double* yCoords, int numCoords,
  double startX, double startY, double startTheta,
  double goalX, double goalY, double goalTheta,
  PathPose** path, int* pathLength,
  double distanceBetweenPathPoints, double turningRadius,
  double planningTimeInSecs, PLANNING_ALGORITHM algo
) {
  assert(algo == PRMstar);

  // TODO:
  // - Create the finder if needed.
  // - Create a `ConditionsOccupancy`.
  // - Call `finder.constructIfNeeded` (if startX == startY == startTheta == goalX == goalY == goalTheta == -1).
  // - Call `finder.query`.

  indexRun++;

  if (ss_ == nullptr) {
    initialize(
      occupancyMap, mapWidth, mapHeight, mapResolution,
      mapOriginX, mapOriginY, robotRadius,
      xCoords, yCoords, numCoords,
      turningRadius);
  }

  ob::StateSpacePtr space = ss_->getStateSpace();
  ob::SpaceInformationPtr si = ss_->getSpaceInformation();

  // TODO: avoid this recreation (due to that `occupancyMap` is a new pointer)
  si->setStateValidityChecker(std::make_shared<MultipleCircleStateValidityChecker>(
    si, occupancyMap, mapWidth, mapHeight, mapResolution, mapOriginX, mapOriginY, robotRadius,
    xCoords, yCoords, numCoords));

  // set the start and goal states
  ob::ScopedState<> start(space), goal(space);
  start[0] = startX;
  start[1] = startY;
  start[2] = startTheta;
  goal[0] = goalX;
  goal[1] = goalY;
  goal[2] = goalTheta;
  ss_->setStartAndGoalStates(start, goal);

  // attempt to solve the problem within planningTimeInSecs seconds of planning time
  std::cout << "Planning time is to be " << planningTimeInSecs << " secs." << std::endl;
  planner_->clearQuery();
  ob::PlannerStatus solved = ss_->solve(planningTimeInSecs);

  std::ostringstream dumpFilenameStream;
  dumpFilenameStream << "SimpleReedsSheppCarPlanner/tmp/pd" << indexRun << ".graphml";
  dumpPlannerData(dumpFilenameStream.str().c_str());

  if (solved == ob::PlannerStatus::StatusType::EXACT_SOLUTION || solved == ob::PlannerStatus::StatusType::APPROXIMATE_SOLUTION) {
    std::cout << "Found solution:" << std::endl;
    ss_->simplifySolution();

    og::PathGeometric pth = ss_->getSolutionPath();
    double pLen = pth.length();
    int numInterpolationPoints = std::ceil(pLen / distanceBetweenPathPoints);
    if (numInterpolationPoints > 0) pth.interpolate(numInterpolationPoints);

    std::vector<ob::State*> states = pth.getStates();
    std::vector<double> reals;

    *pathLength = states.size();
    *path = static_cast<PathPose *>(malloc(sizeof(PathPose) * states.size()));

    for (unsigned i=0; i < states.size(); i++) {
      // TODO: getX(), etc.
      space->copyToReals(reals, states[i]);
      (*path)[i].x = reals[0];
      (*path)[i].y = reals[1];
      (*path)[i].theta = reals[2];
    }

    return true;
  }
  std::cout << "No solution found" << std::endl;
  return false;
}

extern "C" bool plan_multiple_circles_nomap(double* xCoords, double* yCoords, int numCoords, double startX, double startY, double startTheta, double goalX, double goalY, double goalTheta, PathPose** path, int* pathLength, double distanceBetweenPathPoints, double turningRadius, double planningTimeInSecs, PLANNING_ALGORITHM algo) {
  return false;
}