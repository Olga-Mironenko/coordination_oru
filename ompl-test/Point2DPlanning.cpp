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

int main(int /*argc*/, char ** /*argv*/) {
    ompl::msg::setLogLevel(ompl::msg::LOG_INFO);

    const boost::filesystem::path dirResources(TEST_RESOURCES_DIR);
    const std::string filenameFloor = (dirResources / "ppm/floor.ppm").string();
    const std::string filenameFloorWithObstacle = (dirResources / "ppm/floor_with_obstacle.ppm").string();

    for (int iRun = 1000; iRun <= 1000; iRun += 500) {
        srand(1);

        std::cout << "### RUN " << iRun << std::endl;
        PathFinder finder;

        std::shared_ptr<ConditionsPPM> conditionsPPM = std::make_shared<ConditionsPPM>(iRun, "floor", filenameFloor);
        finder.constructIfNeeded(conditionsPPM);

        std::shared_ptr<ompl::geometric::PathGeometric> path;

        conditionsPPM->loadFile(filenameFloorWithObstacle);
        path = finder.query(conditionsPPM, 10, 10, thetaRight, 777, 1265, thetaDown);
        if (path != nullptr) {
            finder.savePath(conditionsPPM, path, "result_demo.ppm");
        }

        conditionsPPM->loadFile(filenameFloor);
        path = finder.query(conditionsPPM, 20, 20, thetaRight, 600, 1000, thetaDown);
        if (path != nullptr) {
            finder.savePath(conditionsPPM, path, "result_demo2.ppm");
        }
    }
}
