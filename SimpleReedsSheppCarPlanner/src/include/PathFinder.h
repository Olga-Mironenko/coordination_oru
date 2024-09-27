#ifndef PATHFINDER_H
#define PATHFINDER_H

#include <iostream>

#include <boost/graph/graphml.hpp>
#include <boost/filesystem.hpp>
#include <boost/graph/astar_search.hpp>

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

#include "PRMcustom.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

class PathFinder {
protected:
    og::SimpleSetupPtr ss_;
    std::shared_ptr<og::PRMcustom> planner_;

public:
    explicit PathFinder() = default;

protected:
    void createSimpleSetup(std::shared_ptr<Conditions> conditions) {
        ob::StateSpacePtr space(std::make_shared<ob::ReedsSheppStateSpace>(conditions->getTurningRadius()));
        ob::RealVectorBounds bounds(2);
        bounds.low[0] = 0;
        bounds.low[1] = 0;
        bounds.high[0] = conditions->getWidth();
        bounds.high[1] = conditions->getHeight();
        space->as<ob::SE2StateSpace>()->setBounds(bounds);
        ss_ = std::make_shared<og::SimpleSetup>(space);

        // set the deterministic sampler
        space->setStateSamplerAllocator(
            std::bind(
                &PathFinder::allocateSampler,
                this, std::placeholders::_1
            ));

        // set state validity checking for this space
        ss_->setStateValidityChecker([this, conditions](const ob::State *state) {
            return isStateValid(conditions, state);
        });

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
    void dumpPlannerData(std::shared_ptr<Conditions> conditions) const {
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
        pdStorage.store(pd, conditions->computeFilenamePD().c_str());
    }

protected:
    void loadPlannerData(std::shared_ptr<Conditions> conditions) {
        createSimpleSetup(conditions);

        ob::PlannerDataStorage pdStorage;
        ob::PlannerData pd(ss_->getSpaceInformation());

        clock_t start = clock();
        pdStorage.load(conditions->computeFilenamePD().c_str(), pd);
        clock_t end = clock();
        OMPL_INFORM("(pdStorage.load took %.6f s)", static_cast<double>(end - start) / CLOCKS_PER_SEC);

        planner_ = std::make_shared<og::PRMcustom>(pd, true);
        ss_->setPlanner(planner_);
    }

protected:
    void construct(std::shared_ptr<Conditions> conditions) {
        OMPL_INFORM("*** CONSTRUCTION:");

        createSimpleSetup(conditions);
        planner_ = std::make_shared<og::PRMcustom>(ss_->getSpaceInformation(), true);
        ss_->setPlanner(planner_);
        ss_->setup();

        const clock_t start = clock();
        planner_->constructRoadmap(conditions->getNumIterations());
        const clock_t end = clock();
        OMPL_INFORM("Construction took %.6f s", static_cast<double>(end - start) / CLOCKS_PER_SEC);

        dumpPlannerData(conditions);
    }

public:
    void constructIfNeeded(std::shared_ptr<Conditions> conditions) {
        // Make a filename based on `conditions`.
        // If the file doesn't exist, call `construct`.
        std::string filenamePD = conditions->computeFilenamePD();
        OMPL_INFORM("Checking PD file %s", filenamePD.c_str());
        if (! boost::filesystem::exists(filenamePD)) {
            construct(conditions);
        }
    }

protected:
    void setStartGoal(
        std::shared_ptr<Conditions> conditions,
        unsigned int xStart, unsigned int yStart, double tStart,
        unsigned int xGoal, unsigned int yGoal, double tGoal) {
        ob::ScopedState<> start(ss_->getStateSpace());
        assert(xStart < conditions->getWidth());
        assert(yStart < conditions->getHeight());
        start[0] = xStart;
        start[1] = yStart;
        start[2] = tStart;

        ob::ScopedState<> goal(ss_->getStateSpace());
        assert(xGoal < conditions->getWidth());
        assert(yGoal < conditions->getHeight());
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
            ob::SpaceInformationPtr si_;
            const ob::PlannerData *pd_;

            explicit EdgePredicate() = default;
            explicit EdgePredicate(ob::SpaceInformationPtr si, const ob::PlannerData *pd) : si_(si), pd_(pd) {}

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
            OMPL_ERROR("Path not found");
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
        std::shared_ptr<Conditions> conditions,
        unsigned int xStart, unsigned int yStart, double tStart,
        unsigned int xGoal, unsigned int yGoal, double tGoal) {
        OMPL_INFORM("*** QUERYING:");

        constructIfNeeded(conditions);

        clock_t start = clock();
        loadPlannerData(conditions);
        setStartGoal(conditions, xStart, yStart, tStart, xGoal, yGoal, tGoal);
        const ob::PlannerStatus plannerStatusInit = planner_->initializeForSolve(ob::IterationTerminationCondition(0));
        clock_t end = clock();
        OMPL_INFORM("Query: initialization took %.6f s", static_cast<double>(end - start) / CLOCKS_PER_SEC);

        if (plannerStatusInit != ob::PlannerStatus::UNKNOWN) {
            OMPL_ERROR("Query: initialization failed, planner returned %s", plannerStatusInit.asString());
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
    void setColor(std::shared_ptr<ConditionsPPM> conditions, int x, int y, unsigned char r, unsigned char g, unsigned char b) const {
        assert(0 <= x && x < conditions->getWidth());
        assert(0 <= y && y < conditions->getHeight());

        const ompl::PPM::Color color = {r, g, b};
        conditions->setPixel(y, x, color);
    }

public:
    void savePath(
        std::shared_ptr<ConditionsPPM> conditions,
        const std::shared_ptr<ompl::geometric::PathGeometric> &path,
        const std::string &filenameResult
        ) {
        path->interpolate();
        for (std::size_t i = 0; i < path->getStateCount(); ++i) {
            const auto state = path->getState(i)->as<ob::ReedsSheppStateSpace::StateType>();
            const int x = static_cast<int>(state->getX());
            const int y = static_cast<int>(state->getY());

            setColor(conditions, x, y, 255, 0, 0);
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

            assert(0 <= x && x < conditions->getWidth());
            assert(0 <= y && y < conditions->getHeight());
            assert(-M_PI <= t && t <= M_PI); // ?

            const int d = 2;
            const int xMin = std::max(0, x - d);
            const int xMax = std::min(static_cast<int>(conditions->getWidth()) - 1, x + d);
            const int yMin = std::max(0, y - d);
            const int yMax = std::min(static_cast<int>(conditions->getHeight()) - 1, y + d);

            for (int xp = xMin; xp <= xMax; ++xp) {
                for (int yp = yMin; yp <= yMax; ++yp) {
                    setColor(conditions, xp, yp, 0, 255, 0);
                }
            }

            for (
                double xp = x, yp = y;
                xMin <= static_cast<int>(round(xp)) && static_cast<int>(round(xp)) <= xMax &&
                yMin <= static_cast<int>(round(yp)) && static_cast<int>(round(yp)) <= yMax;
                xp += cos(t), yp -= sin(t)
            ) {
                setColor(conditions, static_cast<int>(round(xp)), static_cast<int>(round(yp)), 0, 0, 255);
            }

            setColor(conditions, x, y, 0, 127, 0);
        }

        conditions->saveFile(filenameResult);
    }

protected:
    bool isStateValid(std::shared_ptr<Conditions> conditions, const ob::State *statePtr) const {
        const auto state = statePtr->as<ob::ReedsSheppStateSpace::StateType>();
        const int x = static_cast<int>(state->getX());
        const int y = static_cast<int>(state->getY());
        const double t = state->getYaw();

        return ! conditions->isFootprintOccupied(x, y, t);
    }

    ob::StateSamplerPtr allocateSampler(const ob::StateSpace *space) const {
        // specify which deterministic sequence to use, here: HaltonSequence
        return std::make_shared<ob::SE2DeterministicStateSampler>(
            space, std::make_shared<ob::HaltonSequence>(3));
    }
};

#endif //PATHFINDER_H
