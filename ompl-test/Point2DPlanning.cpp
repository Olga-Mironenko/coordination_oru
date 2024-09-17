/* Based on the demo from OMPL. Its author: Ioan Sucan */

#include <iostream>

#include <boost/graph/graphml.hpp>
#include <boost/filesystem.hpp>

#include <ompl/base/PlannerData.h>
#include <ompl/base/PlannerDataStorage.h>
#include <ompl/base/PlannerDataGraph.h>
#include <ompl/base/samplers/DeterministicStateSampler.h>
#include <ompl/base/samplers/deterministic/HaltonSequence.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/util/PPM.h>
#include <ompl/config.h>

namespace ob = ompl::base;
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

            std::cout << x << ", " << y << ", " << t << std::endl;

            points.push_back(std::make_tuple(x, y, t));
        }
        last = next + 1;
    }

    return points;
}

class Plane2DEnvironment
{
public:
    explicit Plane2DEnvironment(const char *ppm_file)
    {
        try
        {
            ppm_.loadFile(ppm_file);
        }
        catch (ompl::Exception &ex)
        {
            OMPL_ERROR("Unable to load %s.\n%s", ppm_file, ex.what());
            return;
        }

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

        space->setup();
        space->printSettings(std::cout);

        ob::PlannerDataStorage pdStorage;
        ob::PlannerData pd(spaceInformation);
        // const bool isLoaded = pdStorage.load("pd.bin", pd);
        const bool isLoaded = false;
        if (isLoaded) {
            OMPL_INFORM("PD is loaded");
            planner_ = std::make_shared<og::PRMstar>(pd);
            planner_->clearQuery();
        } else {
            OMPL_WARN("PD is not loaded");
            planner_ = std::make_shared<og::PRMstar>(spaceInformation);
        }
        ss_->setPlanner(planner_);

        // set the deterministic sampler
        space->setStateSamplerAllocator(
            std::bind(
                &Plane2DEnvironment::allocateSampler,
                this, std::placeholders::_1
            ));
    }

    bool plan(
        double time,
        unsigned int xStart, unsigned int yStart, double tStart,
        unsigned int xGoal, unsigned int yGoal, double tGoal)
    {
        if (!ss_)
            return false;

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

        ss_->setStartAndGoalStates(start, goal);
        const size_t numStarts = 1;
        const size_t numGoals = 1;

        // generate a few solutions; all will be added to the goal;
        OMPL_INFORM("%d vertices", planner_->getRoadmap().m_vertices.size());
        for (int i = 0; i < 1; ++i)
        {
            // if (ss_->getPlanner())
            //     ss_->getPlanner()->clearQuery();
            ss_->solve(time);
            OMPL_INFORM("%d vertices in the roadmap", planner_->getRoadmap().m_vertices.size());
        }

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
            // OMPL_DEBUG("roadmap vertex %d: x=%.1f, y=%.1f, numEdges=%d, isPoint=%d", i, x, y, numEdges, isPoint ? 1 : 0);
        }

        ob::PlannerData pd(ss_->getSpaceInformation());
        ss_->getPlannerData(pd);

        std::ofstream file("pd.graphml");
        pd.printGraphML(file);
        file.close();

        ob::PlannerDataStorage pdStorage;
        pdStorage.store(pd, "pd.bin");

        points_ = plannerDataToPoints(pd);
        OMPL_INFORM("%d points in the planner data", points_.size());
        for (int i = 0; i < points_.size(); ++i) {
            auto point = points_[i];
            OMPL_INFORM("planner data point %d: x=%.1f, y=%.1f, t=%.1f",
                i, std::get<0>(point), std::get<1>(point), std::get<2>(point));
        }
        assert(points_.size() <= roadmap.m_vertices.size());
        assert(points_.size() == numPointsInRoadmap);

        const std::size_t ns = ss_->getProblemDefinition()->getSolutionCount();
        OMPL_INFORM("Found %d solutions (haveSolutionPath=%d)", ns, ss_->haveSolutionPath() ? 1 : 0);
        // TODO: `haveSolutionPath` seems to be always `true`

        if (ss_->haveSolutionPath())
        {
            og::PathGeometric &p = ss_->getSolutionPath();
            return true;
        }

        return false;
    }

    void setColor(int x, int y, unsigned char r, unsigned char g, unsigned char b) {
        assert(0 <= x && x < width_);
        assert(0 <= y && y < height_);

        ompl::PPM::Color &c = ppm_.getPixel(y, x);
        c.red = r;
        c.green = g;
        c.blue = b;
    }

    void recordSolution()
    {
        if (!ss_ || !ss_->haveSolutionPath())
            return;
        og::PathGeometric &path = ss_->getSolutionPath();

        path.interpolate(); // TODO: why is it needed?

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
    }

    void save(const char *filename)
    {
        if (!ss_)
            return;
        ppm_.saveFile(filename);
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
    std::shared_ptr<og::PRMstar> planner_;
    std::vector<std::tuple<double, double, double> > points_;
};

int main(int /*argc*/, char ** /*argv*/)
{
    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

    const boost::filesystem::path path(TEST_RESOURCES_DIR);
    Plane2DEnvironment env((path / "ppm/floor.ppm").string().c_str());

    if (env.plan(10, 10, 10, thetaRight, 777, 1265, thetaDown))
    {
        env.recordSolution();
        env.save("result_demo.ppm");
    }

    return 0;
}
