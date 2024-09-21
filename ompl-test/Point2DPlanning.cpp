/* Based on the demo from OMPL. Its author: Ioan Sucan */

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

std::vector<std::tuple<double, double, double> > plannerDataToPoints(const ob::PlannerData& pd)
{
    std::ostringstream streamGraphml;
    pd.printGraphML(streamGraphml);

    // std::cout << streamGraphml.str() << std::endl;

    std::string s = streamGraphml.str();
    const std::string prefix = "<data key=\"key0\">";
    const std::string postfix = "</data>";

    std::vector<std::tuple<double, double, double> > points;

    size_t last = 0;
    size_t next = 0;
    while ((next = s.find('\n', last)) != std::string::npos) {
        std::string line = s.substr(last, next - last);
        const size_t indexPrefix = line.find(prefix);
        if (indexPrefix != std::string::npos) {
            const size_t indexPostfix = line.find(postfix);
            assert(indexPostfix != std::string::npos);
            std::string pair = line.substr(indexPrefix + prefix.size(), indexPostfix - indexPrefix - prefix.size());

            const size_t indexComma1 = pair.find(',');
            const size_t indexComma2 = pair.find(',', indexComma1 + 1);
            assert(indexComma1 != std::string::npos);
            double x = strtod(pair.substr(0, indexComma1).c_str(), nullptr);
            double y = strtod(pair.substr(indexComma1 + 1, indexComma2).c_str(), nullptr);
            double t = strtod(pair.substr(indexComma2 + 1).c_str(), nullptr);

            // std::cout << x << ", " << y << ", " << t << std::endl;

            points.push_back(std::make_tuple(x, y, t));
        }
        last = next + 1;
    }

    return points;
}

// Used for A* search.  Computes the heuristic distance from vertex v1 to the goal
ob::Cost distanceHeuristic(ob::PlannerData::Graph::Vertex v1,
                           const ob::GoalState* goal,
                           const ob::OptimizationObjective* obj,
                           const boost::property_map<ob::PlannerData::Graph::Type,
                           vertex_type_t>::type& plannerDataVertices)
{
    return ob::Cost(obj->costToGo(plannerDataVertices[v1]->getState(), goal));
}

class Plane2DEnvironment
{
public:
    explicit Plane2DEnvironment(const std::string &ppm_file)
    {
        ppm_file_ = ppm_file;
        try
        {
            ppm_.loadFile(ppm_file_.c_str());
        }
        catch (ompl::Exception &ex)
        {
            OMPL_ERROR("Unable to load %s.\n%s", ppm_file.c_str(), ex.what());
            return;
        }

        is_constructed_ = false;

        // TODO: an IntegerStateSpace
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

        const auto spaceInformation = ss_->getSpaceInformation();

        const double minWallWidth = 1.0;
        const double resolution = minWallWidth / space->getMaximumExtent();
        spaceInformation->setStateValidityCheckingResolution(resolution);

        // set state validity checking for this space
        ss_->setStateValidityChecker([this](const ob::State *state) { return isStateValid(state); });

        // space->setup();
        // space->printSettings(std::cout);

        planner_ = std::make_shared<og::PRMcustom>(spaceInformation, true);
        ss_->setPlanner(planner_);

        // set the deterministic sampler
        space->setStateSamplerAllocator(
            std::bind(
                &Plane2DEnvironment::allocateSampler,
                this, std::placeholders::_1
            ));
    }

    void dumpGraphML(const std::string &filename) const {
        ob::PlannerData pd(ss_->getSpaceInformation());
        ss_->getPlannerData(pd);

        std::ofstream file(filename.c_str());
        pd.printGraphML(file);
        file.close();
    }

    void dumpPlannerData(const std::string &filename) const {
        ob::PlannerData pd(ss_->getSpaceInformation());
        ss_->getPlannerData(pd);

        ob::PlannerDataStorage pdStorage;
        pdStorage.store(pd, filename.c_str());
    }

    void loadPlannerData(const std::string &filename) {
        ob::PlannerDataStorage pdStorage;
        ob::PlannerData pd(ss_->getSpaceInformation());
        pdStorage.load(filename.c_str(), pd);

        planner_ = std::make_shared<og::PRMcustom>(pd, true);
    }

    void construct(const int numIterations)
    {
        assert(ss_);

        planner_->clear();

        OMPL_INFORM("*** CONSTRUCTION:");
        const clock_t start = clock();
        planner_->constructRoadmap(numIterations);
        const clock_t end = clock();
        OMPL_INFORM("Construction took %.6f s", static_cast<double>(end - start)/ CLOCKS_PER_SEC);

        dumpPlannerData("pd.bin");

        is_constructed_ = true;
    }

    void setStartGoal(
        unsigned int xStart, unsigned int yStart, double tStart,
        unsigned int xGoal, unsigned int yGoal, double tGoal)
    {
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

        ss_->getProblemDefinition()->clearSolutionPaths();
        ss_->getProblemDefinition()->clearSolutionNonExistenceProof();
        ss_->getProblemDefinition()->setStartAndGoalStates(start, goal);
        planner_->setProblemDefinition(ss_->getProblemDefinition());
    }

    // Based on `readPlannerData()` from `ompl/demos/PlannerData.cpp`.
    void pdToPath(ob::PlannerData &data, og::PathGeometric &path)
    {
        auto si = ss_->getSpaceInformation();

        // Re-extract a shortest path from the loaded planner data
        assert(data.numStartVertices() > 0 && data.numGoalVertices() > 0);

        // Create an optimization objective for optimizing path length in A*
        ob::PathLengthOptimizationObjective opt(si);

        // Computing the weights of all edges based on the state space distance
        // This is not done by default for efficiency
        data.computeEdgeWeights(opt);

        // Getting a handle to the raw Boost.Graph data
        ob::PlannerData::Graph::Type& graph = data.toBoostGraph();

        // Now we can apply any Boost.Graph algorithm.  How about A*!

        // create a predecessor map to store A* results in
        boost::vector_property_map<ob::PlannerData::Graph::Vertex> prev(data.numVertices());

        // Retrieve a property map with the PlannerDataVertex object pointers for quick lookup
        boost::property_map<ob::PlannerData::Graph::Type, vertex_type_t>::type vertices = get(vertex_type_t(), graph);

        // Run A* search over our planner data
        ob::GoalState goal(si);
        goal.setState(data.getGoalVertex(0).getState());
        ob::PlannerData::Graph::Vertex start = boost::vertex(data.getStartIndex(0), graph);
        boost::astar_visitor<boost::null_visitor> dummy_visitor;
        boost::astar_search(graph, start,
            [&goal, &opt, &vertices](ob::PlannerData::Graph::Vertex v1) { return distanceHeuristic(v1, &goal, &opt, vertices); },
            boost::predecessor_map(prev).
            distance_compare([&opt](ob::Cost c1, ob::Cost c2) { return opt.isCostBetterThan(c1, c2); }).
            distance_combine([&opt](ob::Cost c1, ob::Cost c2) { return opt.combineCosts(c1, c2); }).
            distance_inf(opt.infiniteCost()).
            distance_zero(opt.identityCost()).
            visitor(dummy_visitor));

        // Extracting the path
        path.clear();
        for (ob::PlannerData::Graph::Vertex pos = boost::vertex(data.getGoalIndex(0), graph);
             prev[pos] != pos;
             pos = prev[pos])
        {
            path.append(vertices[pos]->getState());
        }
        path.append(vertices[start]->getState());
        path.reverse();

        // print the path to screen
        //path.print(std::cout);
        std::cout << "Found stored solution with " << path.getStateCount() << " states and length " << path.length() << std::endl;
    }

    /*
     * - load "pd.bin"
     * - do initialization of `solve()`
     * - do the search of `readPlannerData()`
     */
    bool query(
        unsigned int xStart, unsigned int yStart, double tStart,
        unsigned int xGoal, unsigned int yGoal, double tGoal,
        og::PathGeometric &path)
    {
        OMPL_INFORM("*** QUERYING:");

        loadPlannerData("pd.bin");

        setStartGoal(xStart, yStart, tStart, xGoal, yGoal, tGoal);

        const ob::PlannerStatus plannerStatusInit = planner_->initializeForSolve(ob::IterationTerminationCondition(0));
        if (plannerStatusInit != ob::PlannerStatus::UNKNOWN) {
            return false;
        }

        ob::PlannerData pd(ss_->getSpaceInformation());
        planner_->getPlannerData(pd);
        pdToPath(pd, path);
        return true;
    }

    /*
    bool plan(
        unsigned int xStart, unsigned int yStart, double tStart,
        unsigned int xGoal, unsigned int yGoal, double tGoal)
    {
        assert(ss_);
        assert(is_constructed_);

        OMPL_INFORM("*** PLANNING:");

        setStartGoal(xStart, yStart, tStart, xGoal, yGoal, tGoal);
        const size_t numStarts = 1;
        const size_t numGoals = 1;

        OMPL_INFORM("%d vertices", planner_->getRoadmap().m_vertices.size());
        planner_->clearQuery();


        OMPL_INFORM("Solution finding:");
        planner_->solve(ob::IterationTerminationCondition(1), true);
        OMPL_INFORM("%d vertices in the roadmap", planner_->getRoadmap().m_vertices.size());

        auto roadmap = planner_->getRoadmap();
        int numPointsInRoadmap = 0;
        for (int i = 0; i < roadmap.m_vertices.size(); ++i) {
            auto vertex = roadmap.m_vertices[i];
            size_t numEdges = vertex.m_out_edges.size();
            bool isPoint = i < numStarts + numGoals || numEdges > 0;
            if (isPoint) {
                numPointsInRoadmap++;
            }

            auto state = vertex.m_property.m_value;
            double x = state->as<ob::RealVectorStateSpace::StateType>()->values[0];
            double y = state->as<ob::RealVectorStateSpace::StateType>()->values[1];
            OMPL_DEBUG("roadmap vertex %d: x=%.1f, y=%.1f, numEdges=%d, isPoint=%d", i, x, y, numEdges, isPoint ? 1 : 0);
        }

        dumpGraphML("pd.graphml");

        ob::PlannerData pd(ss_->getSpaceInformation());
        ss_->getPlannerData(pd);

        points_ = plannerDataToPoints(pd);
        OMPL_INFORM("%d points in the planner data", points_.size());
        for (int i = 0; i < points_.size(); ++i) {
            auto point = points_[i];
            // OMPL_INFORM("planner data point %d: x=%.1f, y=%.1f, t=%.1f",
            //     i, std::get<0>(point), std::get<1>(point), std::get<2>(point));
        }
        assert(points_.size() <= roadmap.m_vertices.size());
        // assert(points_.size() == numPointsInRoadmap);

        if (ss_->haveSolutionPath()) {
            OMPL_INFORM("Found a solution");
            return true;
        }
        OMPL_WARN("Not found a solution");
        return false;
    }
    */

    void setColor(int x, int y, unsigned char r, unsigned char g, unsigned char b) {
        assert(0 <= x && x < width_);
        assert(0 <= y && y < height_);

        ompl::PPM::Color &c = ppm_.getPixel(y, x);
        c.red = r;
        c.green = g;
        c.blue = b;
    }

    void savePath(og::PathGeometric &path, const char *filename)
    {
        path.interpolate(); // TODO: why is it needed?

        ppm_.loadFile(ppm_file_.c_str());

        for (std::size_t i = 0; i < path.getStateCount(); ++i)
        {
            const auto state = path.getState(i)->as<ob::ReedsSheppStateSpace::StateType>();
            const int x = static_cast<int>(state->getX());
            const int y = static_cast<int>(state->getY());

            assert(0 <= x && x < width_);
            assert(0 <= y && y < height_);

            setColor(x, y, 255, 0, 0);
        }

        for (const auto point : points_) {
            const int x = static_cast<int>(std::get<0>(point));
            const int y = static_cast<int>(std::get<1>(point));
            const double t = std::get<2>(point);

            assert(0 <= x && x < width_);
            assert(0 <= y && y < height_);
            // assert(-M_PI_2 <= t && t < M_PI_2);

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

    og::PathGeometric makeEmptyPath()
    {
        return og::PathGeometric(ss_->getSpaceInformation());
    }

private:
    bool isStateValid(const ob::State *statePtr) const
    {
        const auto state = statePtr->as<ob::ReedsSheppStateSpace::StateType>();
        const int x = static_cast<int>(state->getX());
        const int y = static_cast<int>(state->getY());
        // OMPL_DEBUG("isStateValue(%d, %d)", x, y);

        if (! (0 <= x && x < width_ && 0 <= y && y < height_)) {
            return false;
        }

        const ompl::PPM::Color &c = ppm_.getPixel(y, x);
        const bool isValid = c.red > 127 && c.green > 127 && c.blue > 127;

        // OMPL_INFORM("isStateValid(%d,%d) -> %d", x, y, isValid ? 1 : 0);

        return isValid;
    }

    ob::StateSamplerPtr allocateSampler(const ompl::base::StateSpace *space)
    {
        // specify which deterministic sequence to use, here: HaltonSequence
        return std::make_shared<ompl::base::SE2DeterministicStateSampler>(
            space, std::make_shared<ompl::base::HaltonSequence>(3));
    }

    og::SimpleSetupPtr ss_;
    size_t width_;
    size_t height_;
    ompl::PPM ppm_;
    std::string ppm_file_;
    std::shared_ptr<og::PRMcustom> planner_;
    std::vector<std::tuple<double, double, double> > points_;
    bool is_constructed_;
};

int main(int /*argc*/, char ** /*argv*/)
{
    ompl::msg::setLogLevel(ompl::msg::LOG_INFO);

    const boost::filesystem::path pathResources(TEST_RESOURCES_DIR);

    for (int iRun = 1; iRun <= 1; iRun++) {
        srand(1);

        std::cout << "### RUN " << iRun << std::endl;
        Plane2DEnvironment env((pathResources / "ppm/floor.ppm").string());

        env.construct(1000);
        og::PathGeometric path = env.makeEmptyPath();

        if (env.query(10, 10, thetaRight, 777, 1265, thetaDown, path))
        {
            env.savePath(path, "result_demo.ppm");
        }

        if (env.query(20, 20, thetaRight, 600, 1000, thetaDown, path))
        {
            env.savePath(path, "result_demo2.ppm");
        }
    }
}
