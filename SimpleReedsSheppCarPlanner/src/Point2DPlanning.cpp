/* Based on the demo from OMPL. Its author: Ioan Sucan */

#include <iostream>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/graph/astar_search.hpp>

#include <ompl/base/PlannerDataStorage.h>
#include <ompl/base/PlannerDataGraph.h>
#include <ompl/base/samplers/DeterministicStateSampler.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>

#include "ConditionsOccupancy.h"
#include "ConditionsPPM.h"
#include "PathFinder.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

// The same as in `coordination_oru`:
constexpr double thetaDown = -M_PI_2;
constexpr double thetaUp = M_PI_2;
constexpr double thetaRight = 0;
constexpr double thetaLeft = M_PI;

void testConditions(bool isPPM, bool isSinglePointFootprint, int numIterationsSimplification) {
    const boost::filesystem::path dirResources(TEST_RESOURCES_DIR);
    const std::string filenameFloor = (dirResources / "ppm/floor.ppm").string();
    const std::string filenameFloorWithObstacle = (dirResources / "ppm/floor_with_obstacle.ppm").string();

    const int numIterations = 1000;
    for (int iRun = numIterations; iRun <= numIterations; iRun += 500) {
        constexpr double turningRadius = 10;

        // Initialization

        srand(1);

        std::cout << "### " << (isPPM ? "PPM" : "Occupancy")
                  << " " << (isSinglePointFootprint ? "single" : "non-single")
                  << " simp" << numIterationsSimplification
                  << " RUN " << iRun << std::endl;
        PathFinder finder;

        std::shared_ptr<Footprint> footprint;
        std::shared_ptr<Conditions> conditions;

        if (isSinglePointFootprint) {
            footprint = std::make_shared<Footprint>();
        } else {
            // See `makeFootprint` in `AbstractVehicle.java`.
            double xLengthFront = 5;
            double xLengthBack = 5;
            double yLengthLeft = 2;
            double yLengthRight = 2;
            footprint = std::make_shared<Footprint>(1, 0, 0, 2,
                                                    new double[4] { -xLengthBack, xLengthFront, xLengthFront, -xLengthBack },
                                                    new double[4] { yLengthLeft, yLengthLeft, -yLengthRight, -yLengthRight },
                                                    4, false);
        }

        if (isPPM) {
            conditions = std::make_shared<ConditionsPPM>(
                "floor", numIterations, turningRadius, footprint, filenameFloor);
        } else {
            conditions = std::make_shared<ConditionsOccupancy>(
                "floor", numIterations, turningRadius, footprint, filenameFloor);
        }
        finder.constructIfNeeded(conditions);

        std::shared_ptr<ompl::geometric::PathGeometric> path;

        // Query 1:

        auto makeFilename = [&](int iQuery) -> std::string {
            return (
                boost::format("tmp/result_%s_%d_simp%s.ppm")
                % conditions->computeId() % iQuery % numIterationsSimplification).str();
        };

        srand(1);
        conditions->loadFile(filenameFloorWithObstacle);
        path = finder.query(
            conditions,
            10, 10, thetaRight,
            777, 1265, thetaDown,
            numIterationsSimplification);
        if (isPPM && path != nullptr) {
            finder.savePath(
                std::dynamic_pointer_cast<ConditionsPPM>(conditions),
                path,
                makeFilename(1));
        }

        // Query 2:

         srand(1);
         conditions->loadFile(filenameFloor);
         path = finder.query(
             conditions,
             20, 20, thetaRight,
             600, 1000, thetaDown,
             numIterationsSimplification);
         if (isPPM && path != nullptr) {
             finder.savePath(
                 std::dynamic_pointer_cast<ConditionsPPM>(conditions),
                 path,
                 makeFilename(2));
         }
    }
}

int main(int argc, char *argv[]) {
    // assert(argc == 2);
    // int seed = std::atoi(argv[1]);
    int seed = 107;

    std::cout << "Seed: " << seed << std::endl;
    ompl::RNG::setSeed(seed);

    ompl::msg::setLogLevel(ompl::msg::LOG_INFO);

    // testConditions(false, true);
    // testConditions(true, true);

    // testConditions(true, false, 0);
    // testConditions(true, false, 100);
    testConditions(true, false, 500);
    // testConditions(true, false, 1000);
    // testConditions(true, false);
}
