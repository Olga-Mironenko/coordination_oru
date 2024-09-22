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

// Used for A* search.  Computes the heuristic distance from vertex v1 to the goal
ob::Cost distanceHeuristic(ob::PlannerData::Graph::Vertex v1,
                           const ob::GoalState *goal,
                           const ob::OptimizationObjective *obj,
                           const boost::property_map<ob::PlannerData::Graph::Type,
                               vertex_type_t>::type &plannerDataVertices) {
    return ob::Cost(obj->costToGo(plannerDataVertices[v1]->getState(), goal));
}

class Plane2DEnvironment {
protected:
    std::string ppm_file_;
    std::string pd_file_;

    ompl::PPM ppm_;
    size_t width_;
    size_t height_;

    og::SimpleSetupPtr ss_;
    std::shared_ptr<og::PRMcustom> planner_;

public:
    explicit Plane2DEnvironment(std::string ppm_file, std::string pd_file_)
        : ppm_file_(std::move(ppm_file))
        , pd_file_(std::move(pd_file_)) {
    }

protected:
    void createSimpleSetup() {
        ob::StateSpacePtr space(std::make_shared<ob::ReedsSheppStateSpace>(10));
        width_ = ppm_.getWidth();
        height_ = ppm_.getHeight();
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

public:
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

        ob::PlannerDataStorage pdStorage;
        pdStorage.store(pd, pd_file_.c_str());
    }

protected:
    void loadPlannerData() {
        createSimpleSetup();

        ob::PlannerDataStorage pdStorage;
        ob::PlannerData pd(ss_->getSpaceInformation());

        clock_t start = clock();
        pdStorage.load(pd_file_.c_str(), pd);
        clock_t end = clock();
        OMPL_INFORM("(pdStorage.load took %.6f s)", static_cast<double>(end - start) / CLOCKS_PER_SEC);

        planner_ = std::make_shared<og::PRMcustom>(pd, true);
        ss_->setPlanner(planner_);
    }

public:
    void construct(const int numIterations) {
        OMPL_INFORM("*** CONSTRUCTION:");

        ppm_.loadFile(ppm_file_.c_str());
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
    std::shared_ptr<ompl::geometric::PathGeometric> pdToPath(ob::PlannerData &pd) const {
        auto si = ss_->getSpaceInformation();

        // Re-extract a shortest path from the loaded planner data
        assert(pd.numStartVertices() == 1 && pd.numGoalVertices() == 1);

        // Create an optimization objective for optimizing path length in A*
        ob::PathLengthOptimizationObjective opt(si);

        // Computing the weights of all edges based on the state space distance
        // This is not done by default for efficiency
        pd.computeEdgeWeights(opt);

        // Getting a handle to the raw Boost.Graph data
        ob::PlannerData::Graph::Type &graph = pd.toBoostGraph();

        // Now we can apply any Boost.Graph algorithm.  How about A*!

        // create a predecessor map to store A* results in
        boost::vector_property_map<ob::PlannerData::Graph::Vertex> prev(pd.numVertices());

        // Retrieve a property map with the PlannerDataVertex object pointers for quick lookup
        boost::property_map<ob::PlannerData::Graph::Type, vertex_type_t>::type vertices = get(vertex_type_t(), graph);

        // Run A* search over our planner data
        ob::GoalState goal(si);
        goal.setState(pd.getGoalVertex(0).getState());
        ob::PlannerData::Graph::Vertex start = boost::vertex(pd.getStartIndex(0), graph);
        boost::astar_visitor<boost::null_visitor> dummy_visitor;
        boost::astar_search(graph, start,
                            [&goal, &opt, &vertices](ob::PlannerData::Graph::Vertex v1) {
                                return distanceHeuristic(v1, &goal, &opt, vertices);
                            },
                            boost::predecessor_map(prev).
                            distance_compare([&opt](ob::Cost c1, ob::Cost c2) { return opt.isCostBetterThan(c1, c2); }).
                            distance_combine([&opt](ob::Cost c1, ob::Cost c2) { return opt.combineCosts(c1, c2); }).
                            distance_inf(opt.infiniteCost()).
                            distance_zero(opt.identityCost()).
                            visitor(dummy_visitor));

        // Extracting the path
        auto path = std::make_shared<og::PathGeometric>(si);
        for (ob::PlannerData::Graph::Vertex pos = boost::vertex(pd.getGoalIndex(0), graph);
             prev[pos] != pos;
             pos = prev[pos]) {
            path->append(vertices[pos]->getState());
        }
        path->append(vertices[start]->getState());
        path->reverse();

        // print the path to screen
        //path.print(std::cout);
        std::cout << "Found stored solution with " << path->getStateCount() << " states and length " << path->length()
                << std::endl;
        return path;
    }

public:
    std::shared_ptr<ompl::geometric::PathGeometric> query(
        unsigned int xStart, unsigned int yStart, double tStart,
        unsigned int xGoal, unsigned int yGoal, double tGoal) {
        OMPL_INFORM("*** QUERYING:");

        ppm_.loadFile(ppm_file_.c_str());

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
        std::shared_ptr<ompl::geometric::PathGeometric> path = pdToPath(pd);
        end = clock();
        OMPL_INFORM("Query: path finding took %.6f s", static_cast<double>(end - start) / CLOCKS_PER_SEC);

        return path;
    }

protected:
    void setColor(int x, int y, unsigned char r, unsigned char g, unsigned char b) {
        assert(0 <= x && x < width_);
        assert(0 <= y && y < height_);

        ompl::PPM::Color &c = ppm_.getPixel(y, x);
        c.red = r;
        c.green = g;
        c.blue = b;
    }

public:
    void savePath(std::shared_ptr<ompl::geometric::PathGeometric> path, const char *filename) {
        ppm_.loadFile(ppm_file_.c_str());

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

            const auto state = planner_->vertexToState(i)->as<ob::ReedsSheppStateSpace::StateType>();
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

        ppm_.saveFile(filename);
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

        const ompl::PPM::Color &c = ppm_.getPixel(y, x);
        const bool isValid = c.red > 127 && c.green > 127 && c.blue > 127;

        // OMPL_INFORM("isStateValid(%d,%d) -> %d", x, y, isValid ? 1 : 0);

        return isValid;
    }

    ob::StateSamplerPtr allocateSampler(const ompl::base::StateSpace *space) const {
        // specify which deterministic sequence to use, here: HaltonSequence
        return std::make_shared<ompl::base::SE2DeterministicStateSampler>(
            space, std::make_shared<ompl::base::HaltonSequence>(3));
    }
};

int main(int /*argc*/, char ** /*argv*/) {
    ompl::msg::setLogLevel(ompl::msg::LOG_INFO);

    const boost::filesystem::path pathResources(TEST_RESOURCES_DIR);

    for (int iRun = 500; iRun <= 2500; iRun += 500) {
        srand(1);

        std::cout << "### RUN " << iRun << std::endl;
        Plane2DEnvironment env((pathResources / "ppm/floor.ppm").string(), "pd.bin");

        env.construct(iRun);
        std::shared_ptr<ompl::geometric::PathGeometric> path;

        path = env.query(10, 10, thetaRight, 777, 1265, thetaDown);
        if (path != nullptr) {
            env.savePath(path, "result_demo.ppm");
        }

        path = env.query(20, 20, thetaRight, 600, 1000, thetaDown);
        if (path != nullptr) {
            env.savePath(path, "result_demo2.ppm");
        }
    }
}
