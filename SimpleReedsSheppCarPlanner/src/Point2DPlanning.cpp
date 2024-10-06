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
constexpr double thetaDown = -M_PI_2;   // Facing down direction
constexpr double thetaUp = M_PI_2;      // Facing up direction
constexpr double thetaRight = 0;        // Facing right direction
constexpr double thetaLeft = M_PI;      // Facing left direction

/**
 * Function to test different conditions for path-finding
 * - The main function that performs different path-finding tests by configuring the environment, footprint, and planner.
 * - Initializes resources like the map and footprint for the robot.
 * - Constructs the roadmap.
 * - Performs two queries, one on a map with an obstacle and one on a map without obstacles.
 *   - The map without obstacles is used AFTER the other map to check that no information about obstacles is
 *     unintentionally transferred between queries.
 *   - Each query:
 *     - loads the roadmap from a file (without any start and goal points);
 *     - adds the current start and goal points (and related connections through `addMilestone`)
 *       to the roadmap (further queries are not affected by these points).
 */
void testPathFinder(bool isPPM, bool isSinglePointFootprint, int numIterationsSimplification) {
    const boost::filesystem::path dirResources(TEST_RESOURCES_DIR);  // Path to test resources directory
    // Note: In the git repo, the images are stored in the PNG format (for storage efficiency).
    // Run `convert floor.png floor.ppm`, etc. (initially).
    const std::string filenameFloor = (dirResources / "ppm/floor.ppm").string();  // Filename for the map without obstacles
    const std::string filenameFloorWithObstacle = (dirResources / "ppm/floor_with_obstacle.ppm").string();  // Filename for the map with an obstacle

    constexpr int numIterationsConstruction = 2000;  // Number of iterations for roadmap construction
    constexpr double turningRadius = 10;  // Turning radius of the Reeds-Shepp vehicle model

    // Initialization:
    srand(1);  // Set the seed for random number generator to ensure reproducibility

    std::cout << "### " << (isPPM ? "PPM" : "Occupancy")
              << " " << (isSinglePointFootprint ? "single" : "non-single")
              << " con" << numIterationsConstruction
              << " simp" << numIterationsSimplification
              << std::endl;
    PathFinder finder;  // Create an instance of the PathFinder class to handle path-finding

    // Initialize the footprint for the robot (either a single point or a polygon)
    std::shared_ptr<Footprint> footprint;
    if (isSinglePointFootprint) {
        footprint = std::make_shared<Footprint>();  // Simple point footprint (single-point representation)
    } else {
        // Create a polygonal footprint based on specified dimensions.
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

    // Initialize conditions based on the map type (either PPM or Occupancy map)
    std::shared_ptr<Conditions> conditions;
    if (isPPM) {
        // Use PPM (Portable Pixel Map) image representation of the map
        conditions = std::make_shared<ConditionsPPM>(
            "floor", numIterationsConstruction, numIterationsSimplification, turningRadius, footprint,
            filenameFloor);
    } else {
        // Use occupancy (bit-per-pixel, like `OccupancyMap` in Java) representation of the map
        conditions = std::make_shared<ConditionsOccupancy>(
            "floor", numIterationsConstruction, numIterationsSimplification, turningRadius, footprint,
            filenameFloor);
    }

    // Construct the roadmap if the planner data does not already exist
    finder.constructIfNeeded(conditions);

    std::shared_ptr<ompl::geometric::PathGeometric> path;

    auto makeFilename = [&](int iQuery) -> std::string {
        return (
            boost::format("tmp/result_%s_query%d_simp%s.ppm")
            % conditions->computeIdConstruction() % iQuery % numIterationsSimplification).str();
    };

    // Query 1: Path-finding on the map with an obstacle
    srand(1);  // Set seed for reproducibility
    conditions->loadFile(filenameFloorWithObstacle);  // Load the map with an obstacle
    path = finder.query(
        conditions,
        10, 10, thetaRight,  // Start position (x, y, theta)
        777, 1265, thetaDown);  // Goal position (x, y, theta)
    if (isPPM && path != nullptr) {
        // Save the resulting path to a PPM file for visualization
        finder.savePath(
            std::dynamic_pointer_cast<ConditionsPPM>(conditions),
            path,
            makeFilename(1));
    }

    // Query 2: Path-finding on the map without obstacles
    srand(1);  // Reset seed for consistency
    conditions->loadFile(filenameFloor);  // Load the map without obstacles
    path = finder.query(
        conditions,
        20, 20, thetaRight,  // Start position (x, y, theta)
        600, 1000, thetaDown);  // Goal position (x, y, theta)
    if (isPPM && path != nullptr) {
        // Save the resulting path to a PPM file for visualization
        finder.savePath(
            std::dynamic_pointer_cast<ConditionsPPM>(conditions),
            path,
            makeFilename(2));
    }
}

int main(int argc, char *argv[]) {
    ompl::msg::setLogLevel(ompl::msg::LOG_INFO);  // Set the log level for OMPL to display informational messages

    // assert(argc == 2);
    // int seed = std::atoi(argv[1]);
    int seed = 107;  // Define a seed for random number generation

    std::cout << "Seed: " << seed << std::endl;
    ompl::RNG::setSeed(seed);  // Set the seed for OMPL's random number generator (used during path simplification)

    // Test different configurations of conditions (PPM/occupancy, footprint type, simplification iterations)
    // testPathFinder(false, true);
    // testPathFinder(true, true);

    // testPathFinder(true, false, 0);
    // testPathFinder(true, false, 100);
    testPathFinder(true, false, 0);  // Run test with PPM map, complex footprint, and no simplification
    // testPathFinder(true, false, 1000);
    // testPathFinder(true, false);
}