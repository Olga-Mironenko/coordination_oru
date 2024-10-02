/* Based on the demo from OMPL. Its author: Ioan Sucan */

#include <iostream>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/graph/astar_search.hpp>

#include <ompl/base/PlannerDataStorage.h>
#include <ompl/base/samplers/DeterministicStateSampler.h>
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

    for (int numIterationsConstruction = 2000; numIterationsConstruction <= 2000; numIterationsConstruction++) {
        constexpr double turningRadius = 10;

        // Initialization:

        srand(1);

        std::cout << "### " << (isPPM ? "PPM" : "Occupancy")
                  << " " << (isSinglePointFootprint ? "single" : "non-single")
                  << " con" << numIterationsConstruction
                  << " simp" << numIterationsSimplification
                  << std::endl;
        PathFinder finder;

        std::shared_ptr<Footprint> footprint;
        std::shared_ptr<Conditions> conditions;

        if (isSinglePointFootprint) {
            footprint = std::make_shared<Footprint>();
        } else {
            // See `makeFootprint` in `AbstractVehicle.java`.
            constexpr double xLengthFront = 5;
            constexpr double xLengthBack = 5;
            constexpr double yLengthLeft = 2;
            constexpr double yLengthRight = 2;
            footprint = std::make_shared<Footprint>(1, 0, 0, 2,
                                                    new double[4] { -xLengthBack, xLengthFront, xLengthFront, -xLengthBack },
                                                    new double[4] { yLengthLeft, yLengthLeft, -yLengthRight, -yLengthRight },
                                                    4, false);
        }

        if (isPPM) {
            conditions = std::make_shared<ConditionsPPM>(
                "floor", numIterationsConstruction, numIterationsSimplification, turningRadius, footprint,
                filenameFloor);
        } else {
            conditions = std::make_shared<ConditionsOccupancy>(
                "floor", numIterationsConstruction, numIterationsSimplification, turningRadius, footprint,
                filenameFloor);
        }
        finder.constructIfNeeded(conditions);

        std::shared_ptr<ompl::geometric::PathGeometric> path;

        // Query 1:

        auto makeFilename = [&](int iQuery) -> std::string {
            return (
                boost::format("tmp/result_%s_query%d_simp%s.ppm")
                % conditions->computeIdConstruction() % iQuery % numIterationsSimplification).str();
        };

        srand(1);
        conditions->loadFile(filenameFloorWithObstacle);
        path = finder.query(
            conditions,
            10, 10, thetaRight,
            777, 1265, thetaDown);
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
            600, 1000, thetaDown);
        if (isPPM && path != nullptr) {
            finder.savePath(
                std::dynamic_pointer_cast<ConditionsPPM>(conditions),
                path,
                makeFilename(2));
        }
    }
}

int main(int argc, char *argv[]) {
    ompl::msg::setLogLevel(ompl::msg::LOG_INFO);

    // assert(argc == 2);
    // int seed = std::atoi(argv[1]);
    int seed = 107;

    std::cout << "Seed: " << seed << std::endl;
    ompl::RNG::setSeed(seed);

    // testConditions(false, true);
    // testConditions(true, true);

    // testConditions(true, false, 0);
    // testConditions(true, false, 100);
    testConditions(true, false, 500);
    // testConditions(true, false, 1000);
    // testConditions(true, false);
}
