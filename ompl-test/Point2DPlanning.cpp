/* Based on the demo from OMPL. Its author: Ioan Sucan */

#include <iostream>

#include <boost/graph/graphml.hpp>
#include <boost/filesystem.hpp>
#include <boost/graph/astar_search.hpp>
#include <utility>

#include <ompl/base/PlannerData.h>
#include <ompl/base/PlannerDataStorage.h>
#include <ompl/base/PlannerDataGraph.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/samplers/DeterministicStateSampler.h>
#include <ompl/base/samplers/deterministic/HaltonSequence.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/terminationconditions/IterationTerminationCondition.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/util/PPM.h>

#include "PRMcustom.h"

namespace ob = ompl::base;
namespace base = ompl::base;
namespace og = ompl::geometric;

// The same as in `coordination_oru`:
constexpr double thetaDown = -M_PI_2;
constexpr double thetaUp = M_PI_2;
constexpr double thetaRight = 0;
constexpr double thetaLeft = M_PI;

class Plane2DEnvironment {
protected:
    std::string pathPD_;

    ompl::PPM map_; // the current map
    size_t width_;
    size_t height_;

    og::SimpleSetupPtr ss_;
    std::shared_ptr<og::PRMcustom> planner_;

public:
    explicit Plane2DEnvironment(std::string pathPD_)
        : pathPD_(std::move(pathPD_)) {
    }

protected:
    void createSimpleSetup() {
        ob::StateSpacePtr space(std::make_shared<ob::ReedsSheppStateSpace>(10));
        width_ = map_.getWidth();
        height_ = map_.getHeight();
        ob::RealVectorBounds bounds(2);
        bounds.low[0] = 0;
        bounds.low[1] = 0;
        bounds.high[0] = width_;
        bounds.high[1] = height_;
        space->as<ob::SE2StateSpace>()->setBounds(bounds);
        ss_ = std::make_shared<og::SimpleSetup>(space);

        // set the deterministic sampler
        space->setStateSamplerAllocator(
            std::bind(
                &Plane2DEnvironment::allocateSampler,
                this, std::placeholders::_1
            ));

        // set state validity checking for this space
        ss_->setStateValidityChecker([this](const ob::State *state) { return isStateValid(state); });

        const double minWallWidth = 1.0;
        const double resolution = minWallWidth / space->getMaximumExtent();
        ss_->getSpaceInformation()->setStateValidityCheckingResolution(resolution);
    }

protected:
    static std::string stateToString(const ob::State *stateRaw) {
        const auto state = stateRaw->as<ob::SE2StateSpace::StateType>();
        std::stringstream ss;
        ss << "(" << state->getX() << ", " << state->getY() << ", " << state->getYaw() << ")";
        return ss.str();
    }

protected:
    void dumpGraphML(const std::string &filename) const {
        ob::PlannerData pd(ss_->getSpaceInformation());
        ss_->getPlannerData(pd);

        std::ofstream file(filename.c_str());
        pd.printGraphML(file);
        file.close();
    }

protected:
    void dumpPlannerData() const {
        ob::PlannerData pd(ss_->getSpaceInformation());
        ss_->getPlannerData(pd);

        // Computing the weights of all edges based on the state space distance
        // This is not done by default for efficiency
        // const ob::PathLengthOptimizationObjective opt(ss_->getSpaceInformation());
        // pd.computeEdgeWeights(opt);
        // with that in findPath: 2.5 s for the first query
        // without that in findPath: 9.5 s for the first query
        // with that in construct: 9.5 s for the first query

        ob::PlannerDataStorage pdStorage;
        pdStorage.store(pd, pathPD_.c_str());
    }

protected:
    void loadPlannerData() {
        createSimpleSetup();

        ob::PlannerDataStorage pdStorage;
        ob::PlannerData pd(ss_->getSpaceInformation());

        clock_t start = clock();
        pdStorage.load(pathPD_.c_str(), pd);
        clock_t end = clock();
        OMPL_INFORM("(pdStorage.load took %.6f s)", static_cast<double>(end - start) / CLOCKS_PER_SEC);

        planner_ = std::make_shared<og::PRMcustom>(pd, true);
        ss_->setPlanner(planner_);
    }

public:
    void construct(const std::string &filenameMap, const int numIterations) {
        OMPL_INFORM("*** CONSTRUCTION:");

        map_.loadFile(filenameMap.c_str());
        createSimpleSetup();
        planner_ = std::make_shared<og::PRMcustom>(ss_->getSpaceInformation(), true);
        ss_->setPlanner(planner_);
        ss_->setup();

        const clock_t start = clock();
        planner_->constructRoadmap(numIterations);
        const clock_t end = clock();
        OMPL_INFORM("Construction took %.6f s", static_cast<double>(end - start) / CLOCKS_PER_SEC);

        dumpPlannerData();
    }

protected:
    void setStartGoal(
        unsigned int xStart, unsigned int yStart, double tStart,
        unsigned int xGoal, unsigned int yGoal, double tGoal) {
        ob::ScopedState<> start(ss_->getStateSpace());
        assert(xStart < width_);
        assert(yStart < height_);
        start[0] = xStart;
        start[1] = yStart;
        start[2] = tStart;

        ob::ScopedState<> goal(ss_->getStateSpace());
        assert(xGoal < width_);
        assert(yGoal < height_);
        goal[0] = xGoal;
        goal[1] = yGoal;
        goal[2] = tGoal;

        planner_->setProblemDefinition(ss_->getProblemDefinition());
        planner_->getProblemDefinition()->clearSolutionPaths();
        planner_->getProblemDefinition()->clearSolutionNonExistenceProof();
        planner_->getProblemDefinition()->setStartAndGoalStates(start, goal);
    }

protected:
    // Based on `readPlannerData()` from `ompl/demos/PlannerData.cpp`.
    std::shared_ptr<ompl::geometric::PathGeometric> findPath(ob::PlannerData &pd) const {
        struct EdgePredicate {
            base::SpaceInformationPtr si_;
            const ob::PlannerData *pd_;

            explicit EdgePredicate() = default;
            explicit EdgePredicate(base::SpaceInformationPtr si, const ob::PlannerData *pd) : si_(si), pd_(pd) {}

            bool operator()(const ob::PlannerData::Graph::Edge& e) const {
                const ob::PlannerData::Graph::Vertex fromVertex = e.m_source;
                const ob::PlannerData::Graph::Vertex toVertex = e.m_target;

                const auto fromState = pd_->getVertex(fromVertex).getState();
                const auto toState = pd_->getVertex(toVertex).getState();

                const bool isOk = si_->checkMotion(fromState, toState);

                if (!isOk) {
                    // std::cout << "filtering out edge: " << fromVertex << " " << stateToString(fromState) << " to " <<
                    //         toVertex << " " << stateToString(toState) << std::endl;
                } else {
                    // std::cout << "keeping edge:       " << fromVertex << " " << stateToString(fromState) << " to " <<
                    //         toVertex << " " << stateToString(toState) << std::endl;
                }
                return isOk;
            }
        };
        const auto si = ss_->getSpaceInformation();
        const EdgePredicate edgePredicate(si, &pd);

        // Re-extract a shortest path from the loaded planner data
        assert(pd.numStartVertices() == 1 && pd.numGoalVertices() == 1);

        // Create an optimization objective for optimizing path length in A*
        ob::PathLengthOptimizationObjective opt(si);

        // Computing the weights of all edges based on the state space distance
        // This is not done by default for efficiency
        pd.computeEdgeWeights(opt);
        // with that in findPath: 2.5 s for the first query
        // without that in findPath: 9.5 s for the first query
        // with that in construct: 9.5 s for the first query

        // Getting a handle to the raw Boost.Graph data
        ob::PlannerData::Graph::Type &graphOrig = pd.toBoostGraph();

        // boost::write_graphviz(std::cout, graph_orig);
        const boost::filtered_graph<ob::PlannerData::Graph::Type, EdgePredicate> graphFiltered(graphOrig, edgePredicate);

        // Now we can apply any Boost.Graph algorithm.  How about A*!

        // create a predecessor map to store A* results in
        const boost::vector_property_map<ob::PlannerData::Graph::Vertex> prev(pd.numVertices());

        // Retrieve a property map with the PlannerDataVertex object pointers for quick lookup
        boost::property_map<ob::PlannerData::Graph::Type, vertex_type_t>::type vertices = get(vertex_type_t(), graphOrig);

        // Run A* search over our planner data

        const auto startIndex = pd.getStartIndex(0);
        const auto goalIndex = pd.getGoalIndex(0);

        ob::GoalState goalState(si);
        goalState.setState(pd.getVertex(goalIndex).getState());

        OMPL_INFORM("Running A* from %d %s to %d %s",
            startIndex,
            stateToString(pd.getVertex(startIndex).getState()).c_str(),
            goalIndex,
            stateToString(pd.getVertex(goalIndex).getState()).c_str());

        const boost::astar_visitor<boost::null_visitor> dummy_visitor;

        boost::astar_search(graphFiltered,
                            boost::vertex(startIndex, graphOrig),
                            [&goalState, &opt, &vertices](const ob::PlannerData::Graph::Vertex v) {
                                return ob::Cost(opt.costToGo(vertices[v]->getState(), &goalState));
                            },
                            boost::predecessor_map(prev).
                            distance_compare([&opt](const ob::Cost c1, const ob::Cost c2) {
                                return opt.isCostBetterThan(c1, c2);
                            }).
                            distance_combine([&opt](const ob::Cost c1, const ob::Cost c2) {
                                return opt.combineCosts(c1, c2);
                            }).
                            distance_inf(opt.infiniteCost()).
                            distance_zero(opt.identityCost()).
                            visitor(dummy_visitor));

        // Extracting the path
        auto path = std::make_shared<og::PathGeometric>(si);
        for (ob::PlannerData::Graph::Vertex pos = boost::vertex(goalIndex, graphOrig);
             prev[pos] != pos;
             pos = prev[pos]) {
            path->append(vertices[pos]->getState());
        }
        if (path->getStateCount() == 0 && startIndex != goalIndex) {
            return nullptr;
        }
        path->append(vertices[startIndex]->getState());
        path->reverse();

        // print the path to screen
        //path.print(std::cout);
        std::cout << "Found stored solution with " << path->getStateCount() << " states and length " << path->length()
                << std::endl;
        return path;
    }

public:
    std::shared_ptr<ompl::geometric::PathGeometric> query(
        const std::string &filenameMap,
        unsigned int xStart, unsigned int yStart, double tStart,
        unsigned int xGoal, unsigned int yGoal, double tGoal) {
        OMPL_INFORM("*** QUERYING:");

        map_.loadFile(filenameMap.c_str());

        clock_t start = clock();
        loadPlannerData();
        setStartGoal(xStart, yStart, tStart, xGoal, yGoal, tGoal);
        const ob::PlannerStatus plannerStatusInit = planner_->initializeForSolve(ob::IterationTerminationCondition(0));
        clock_t end = clock();
        OMPL_INFORM("Query: initialization took %.6f s", static_cast<double>(end - start) / CLOCKS_PER_SEC);

        if (plannerStatusInit != ob::PlannerStatus::UNKNOWN) {
            return nullptr;
        }

        start = clock();
        ob::PlannerData pd(ss_->getSpaceInformation());
        planner_->getPlannerData(pd);
        // dumpGraphML("pd.graphml");
        std::shared_ptr<ompl::geometric::PathGeometric> path = findPath(pd);
        end = clock();
        OMPL_INFORM("Query: path finding took %.6f s", static_cast<double>(end - start) / CLOCKS_PER_SEC);

        return path;
    }

protected:
    void setColor(int x, int y, unsigned char r, unsigned char g, unsigned char b) {
        assert(0 <= x && x < width_);
        assert(0 <= y && y < height_);

        ompl::PPM::Color &c = map_.getPixel(y, x);
        c.red = r;
        c.green = g;
        c.blue = b;
    }

public:
    void savePath(
        const std::string &filenameMap,
        const std::shared_ptr<ompl::geometric::PathGeometric> &path,
        const std::string &filenameResult
        ) {
        map_.loadFile(filenameMap.c_str());

        path->interpolate();
        for (std::size_t i = 0; i < path->getStateCount(); ++i) {
            const auto state = path->getState(i)->as<ob::ReedsSheppStateSpace::StateType>();
            const int x = static_cast<int>(state->getX());
            const int y = static_cast<int>(state->getY());

            setColor(x, y, 255, 0, 0);
        }

        ob::PlannerData pd(ss_->getSpaceInformation());
        planner_->getPlannerData(pd);
        // std::cout << "starts=" << pd.numStartVertices() << ", goals=" << pd.numGoalVertices() << std::endl;
        // ob::PlannerData::Graph::Type& graph = pd.toBoostGraph();

        for (int i = 0; i < pd.numVertices(); i++) {
            const auto v = pd.getVertex(i);

            std::vector<unsigned int> neighbor_indices;
            pd.getEdges(i, neighbor_indices);
            if (neighbor_indices.size() == 0) {
                continue;
            }

            const auto state = pd.getVertex(i).getState()->as<ob::ReedsSheppStateSpace::StateType>();
            const int x = static_cast<int>(state->getX());
            const int y = static_cast<int>(state->getY());
            const double t = state->getYaw();

            assert(0 <= x && x < width_);
            assert(0 <= y && y < height_);
            assert(-M_PI <= t && t <= M_PI); // ?

            const int d = 2;
            const int xMin = std::max(0, x - d);
            const int xMax = std::min(static_cast<int>(width_) - 1, x + d);
            const int yMin = std::max(0, y - d);
            const int yMax = std::min(static_cast<int>(height_) - 1, y + d);

            for (int xp = xMin; xp <= xMax; ++xp) {
                for (int yp = yMin; yp <= yMax; ++yp) {
                    setColor(xp, yp, 0, 255, 0);
                }
            }

            for (
                double xp = x, yp = y;
                xMin <= static_cast<int>(round(xp)) && static_cast<int>(round(xp)) <= xMax &&
                yMin <= static_cast<int>(round(yp)) && static_cast<int>(round(yp)) <= yMax;
                xp += cos(t), yp -= sin(t)
            ) {
                setColor(static_cast<int>(round(xp)), static_cast<int>(round(yp)), 0, 0, 255);
            }

            setColor(x, y, 0, 127, 0);
        }

        map_.saveFile(filenameResult.c_str());
    }

protected:
    bool isStateValid(const ob::State *statePtr) const {
        const auto state = statePtr->as<ob::ReedsSheppStateSpace::StateType>();
        const int x = static_cast<int>(state->getX());
        const int y = static_cast<int>(state->getY());
        // OMPL_DEBUG("isStateValue(%d, %d)", x, y);

        if (!(0 <= x && x < width_ && 0 <= y && y < height_)) {
            return false;
        }

        const ompl::PPM::Color &c = map_.getPixel(y, x);
        const bool isValid = c.red > 127 && c.green > 127 && c.blue > 127;

        // OMPL_INFORM("isStateValid(%d,%d) -> %d", x, y, isValid ? 1 : 0);

        return isValid;
    }

    ob::StateSamplerPtr allocateSampler(const ob::StateSpace *space) const {
        // specify which deterministic sequence to use, here: HaltonSequence
        return std::make_shared<ob::SE2DeterministicStateSampler>(
            space, std::make_shared<ob::HaltonSequence>(3));
    }
};

int main(int /*argc*/, char ** /*argv*/) {
    ompl::msg::setLogLevel(ompl::msg::LOG_INFO);

    const boost::filesystem::path dirResources(TEST_RESOURCES_DIR);
    const std::string filenameFloor = (dirResources / "ppm/floor.ppm").string();
    const std::string filenameFloorWithObstacle = (dirResources / "ppm/floor_with_obstacle.ppm").string();

    for (int iRun = 1000; iRun <= 1000; iRun += 500) {
        srand(1);

        std::cout << "### RUN " << iRun << std::endl;
        Plane2DEnvironment env("pd.bin");

        // env.construct(filenameFloor, iRun);
        std::shared_ptr<ompl::geometric::PathGeometric> path;

        path = env.query(filenameFloorWithObstacle, 10, 10, thetaRight, 777, 1265, thetaDown);
        // path = env.query(filenameFloorWithObstacle, 720, 900, thetaRight, 720, 1050, thetaDown);
        if (path != nullptr) {
            env.savePath(filenameFloorWithObstacle, path, "result_demo.ppm");
        }

        path = env.query(filenameFloor, 20, 20, thetaRight, 600, 1000, thetaDown);
        if (path != nullptr) {
            env.savePath(filenameFloor, path, "result_demo2.ppm");
        }
    }
}
