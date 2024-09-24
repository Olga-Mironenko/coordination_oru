/* Based on the demo from OMPL. Its author: Ioan Sucan */

#include <iostream>

#include <boost/graph/graphml.hpp>
#include <boost/filesystem.hpp>
#include <boost/graph/astar_search.hpp>

#include <ompl/base/PlannerDataStorage.h>
#include <ompl/base/PlannerDataGraph.h>
#include <ompl/base/samplers/DeterministicStateSampler.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>

#include "ConditionsPPM.h"
#include "PathFinder.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

// The same as in `coordination_oru`:
constexpr double thetaDown = -M_PI_2;
constexpr double thetaUp = M_PI_2;
constexpr double thetaRight = 0;
constexpr double thetaLeft = M_PI;

void testConditions(bool isPPM, bool isSinglePointFootprint) {
    const boost::filesystem::path dirResources(TEST_RESOURCES_DIR);
    const std::string filenameFloor = (dirResources / "ppm/floor.ppm").string();
    const std::string filenameFloorWithObstacle = (dirResources / "ppm/floor_with_obstacle.ppm").string();

    for (int iRun = 1000; iRun <= 1000; iRun += 500) {
        srand(1);

        std::cout << "### " << (isPPM ? "PPM" : "Occupancy") << " RUN " << iRun << std::endl;
        PathFinder finder;
        std::shared_ptr<ompl::geometric::PathGeometric> path;

        std::shared_ptr<Conditions> conditions;
        std::shared_ptr<Footprint> footprint;

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
            conditions = std::make_shared<ConditionsPPM>(iRun, "floor", filenameFloor, footprint);
        } else {
            // TODO
        }
        finder.constructIfNeeded(conditions);

        conditions->loadFile(filenameFloorWithObstacle);
        path = finder.query(conditions, 10, 10, thetaRight, 777, 1265, thetaDown);
        if (isPPM && path != nullptr) {
            finder.savePath(
                std::dynamic_pointer_cast<ConditionsPPM>(conditions),
                path,
                "tmp/result_demo1.ppm");
        }

        conditions->loadFile(filenameFloor);
        path = finder.query(conditions, 20, 20, thetaRight, 600, 1000, thetaDown);
        if (isPPM && path != nullptr) {
            finder.savePath(
                std::dynamic_pointer_cast<ConditionsPPM>(conditions),
                path,
                "tmp/result_demo2.ppm");
        }
    }
}

int main(int /*argc*/, char ** /*argv*/) {
    ompl::msg::setLogLevel(ompl::msg::LOG_INFO);

    // testConditions(true, true);
    testConditions(true, false);
}