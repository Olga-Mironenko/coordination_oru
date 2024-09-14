/* Based on the demo from OMPL. Its author: Ioan Sucan */

#include <iostream>

#include <boost/graph/graphml.hpp>
#include <boost/filesystem.hpp>

#include <ompl/base/PlannerDataGraph.h>
#include <ompl/base/samplers/DeterministicStateSampler.h>
#include <ompl/base/samplers/deterministic/HaltonSequence.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/util/PPM.h>
#include <ompl/config.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

std::vector<std::pair<double, double> > plannerDataToPoints(const ob::PlannerData& pd)
{
    std::ostringstream streamGraphml;
    pd.printGraphML(streamGraphml);
    // std::cout << streamGraphml.str() << std::endl;

    std::string s = streamGraphml.str();
    const std::string prefix = "<data key=\"key0\">";
    const std::string postfix = "</data>";

    std::vector<std::pair<double, double> > points;

    size_t last = 0;
    size_t next = 0;
    while ((next = s.find('\n', last)) != std::string::npos) {
        std::string line = s.substr(last, next - last);
        const size_t indexPrefix = line.find(prefix);
        if (indexPrefix != std::string::npos) {
            const size_t indexPostfix = line.find(postfix);
            assert(indexPostfix != std::string::npos);
            std::string pair = line.substr(indexPrefix + prefix.size(), indexPostfix - indexPrefix - prefix.size());

            const size_t indexComma = pair.find(',');
            assert(indexComma != std::string::npos);
            double x = strtod(pair.substr(0, indexComma).c_str(), nullptr);
            double y = strtod(pair.substr(indexComma + 1).c_str(), nullptr);

            // std::cout << x << ", " << y << std::endl;

            points.push_back(std::make_pair(x, y));
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
        auto space(std::make_shared<ob::RealVectorStateSpace>());
        width_ = ppm_.getWidth();
        height_ = ppm_.getHeight();
        space->addDimension(0.0, width_);
        space->addDimension(0.0, height_);
        ss_ = std::make_shared<og::SimpleSetup>(space);

        // set state validity checking for this space
        ss_->setStateValidityChecker([this](const ob::State *state) { return isStateValid(state); });
        space->setup();
        ss_->getSpaceInformation()->setStateValidityCheckingResolution(1.0 / space->getMaximumExtent());

        // set the deterministic sampler
        // 2D space, no need to specify bases specifically
        // PRMstar can use the deterministic sampling
        planner_ = std::make_shared<og::PRMstar>(ss_->getSpaceInformation());
        ss_->setPlanner(planner_);
        space->setStateSamplerAllocator(
            std::bind(
                &Plane2DEnvironment::allocateHaltonStateSamplerRealVector,
                this, std::placeholders::_1, 2, std::vector<unsigned int>{2, 3}
            ));
    }

    bool plan(unsigned int xStart, unsigned int yStart, unsigned int xGoal, unsigned int yGoal)
    {
        if (!ss_)
            return false;

        assert(xStart < width_);
        assert(yStart < height_);

        ob::ScopedState<> start(ss_->getStateSpace());
        start[0] = xStart;
        start[1] = yStart;

        ob::ScopedState<> goal(ss_->getStateSpace());
        goal[0] = xGoal;
        goal[1] = yGoal;

        ss_->setStartAndGoalStates(start, goal);
        const size_t numStarts = 1;
        const size_t numGoals = 1;

        // generate a few solutions; all will be added to the goal;
        OMPL_INFORM("%d vertices", planner_->getRoadmap().m_vertices.size());
        for (int i = 0; i < 1; ++i)
        {
            // if (ss_->getPlanner())
            //     ss_->getPlanner()->clearQuery();
            ss_->solve(0.1);
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
            OMPL_INFORM("roadmap vertex %d: x=%.1f, y=%.1f, numEdges=%d, isPoint=%d", i, x, y, numEdges, isPoint ? 1 : 0);
        }

        ob::PlannerData pd(ss_->getSpaceInformation());
        ss_->getPlannerData(pd);
        points_ = plannerDataToPoints(pd);
        OMPL_INFORM("%d points in the planner data", points_.size());
        for (int i = 0; i < points_.size(); ++i) {
            auto point = points_[i];
            OMPL_INFORM("planner data point %d: x=%.1f, y=%.1f", i, point.first, point.second);
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

    void recordSolution()
    {
        if (!ss_ || !ss_->haveSolutionPath())
            return;
        og::PathGeometric &path = ss_->getSolutionPath();

        path.interpolate(); // TODO: why is it needed?

        for (std::size_t i = 0; i < path.getStateCount(); ++i)
        {
            const auto state = path.getState(i)->as<ob::RealVectorStateSpace::StateType>();

            const int x = static_cast<int>(state->values[0]);
            assert(0 <= x && x < width_);

            const int y = static_cast<int>(state->values[1]);
            assert(0 <= y && y < height_);

            ompl::PPM::Color &c = ppm_.getPixel(y, x);
            c.red = 255;
            c.green = 0;
            c.blue = 0;
        }

        for (const auto point : points_) {
            const int x = static_cast<int>(point.first);
            assert(0 <= x && x < width_);

            const int y = static_cast<int>(point.second);
            assert(0 <= y && y < height_);

            for (int dx = -2; dx <= 2; ++dx) {
                for (int dy = -2; dy <= 2; ++dy) {
                    const int xp = x + dx;
                    const int yp = y + dy;
                    if (0 <= xp && xp < width_ && 0 <= yp && yp < height_) {
                        ompl::PPM::Color &c = ppm_.getPixel(yp, xp);
                        c.red = 0;
                        c.green = 255;
                        c.blue = 0;
                    }
                }
            }
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
        const auto state = statePtr->as<ob::RealVectorStateSpace::StateType>();

        const int x = static_cast<int>(state->values[0]);
        assert(0 <= x && x <= width_);
        if (x == width_) {
            return false;
        }

        const int y = static_cast<int>(state->values[1]);
        assert(0 <= y && y <= height_);
        if (y == height_) {
            return false;
        }

        const ompl::PPM::Color &c = ppm_.getPixel(y, x);
        return c.red > 127 && c.green > 127 && c.blue > 127;
    }

    ob::StateSamplerPtr allocateHaltonStateSamplerRealVector(const ompl::base::StateSpace *space, unsigned int dim,
                                                             std::vector<unsigned int> bases = {})
    {
        // specify which deterministic sequence to use, here: HaltonSequence
        // optionally we can specify the bases used for generation (otherwise first dim prime numbers are used)
        if (!bases.empty())
            return std::make_shared<ompl::base::RealVectorDeterministicStateSampler>(
                space, std::make_shared<ompl::base::HaltonSequence>(bases.size(), bases));

        return std::make_shared<ompl::base::RealVectorDeterministicStateSampler>(
            space, std::make_shared<ompl::base::HaltonSequence>(dim));
    }

    og::SimpleSetupPtr ss_;
    size_t width_;
    size_t height_;
    ompl::PPM ppm_;
    std::shared_ptr<og::PRMstar> planner_;
    std::vector<std::pair<double, double> > points_;
};

int main(int /*argc*/, char ** /*argv*/)
{
    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

    const boost::filesystem::path path(TEST_RESOURCES_DIR);
    Plane2DEnvironment env((path / "ppm/floor.ppm").string().c_str());

    if (env.plan(0, 0, 777, 1265))
    {
        env.recordSolution();
        env.save("result_demo.ppm");
    }

    return 0;
}
