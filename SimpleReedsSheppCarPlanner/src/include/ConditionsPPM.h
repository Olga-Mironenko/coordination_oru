 #ifndef CONDITIONSPPM_H
#define CONDITIONSPPM_H

#include <boost/graph/graphml.hpp>
#include <utility>

#include <ompl/util/PPM.h>

#include "Conditions.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

class ConditionsPPM : public Conditions {
public:
    size_t numIterations_;
    std::string mapId_;
    ompl::PPM ppm_;

    ConditionsPPM(size_t numIterations, std::string mapId, const std::string &filenamePPMWithoutObstacles)
        : numIterations_(numIterations), mapId_(std::move(mapId)) {
        ConditionsPPM::loadFile(filenamePPMWithoutObstacles);
    }

    size_t getNumIterations() const {
        return numIterations_;
    }

    std::string computeFilenamePD() const override {
        std::stringstream ss;
        ss << "tmp/" << mapId_ << "-" << numIterations_ << ".pd";
        return ss.str();
    }

    void loadFile(const std::string &filename) override {
        ppm_.loadFile(filename.c_str());
    }

    void saveFile(const std::string &filenameResult) override {
        ppm_.saveFile(filenameResult.c_str());
    }

    size_t getWidth() const override {
        return ppm_.getWidth();
    }

    size_t getHeight() const override {
        return ppm_.getHeight();
    }

    Color getPixel(size_t y, size_t x) const override {
        const ompl::PPM::Color &c = ppm_.getPixel(y, x);
        return {c.red, c.green, c.blue};
    }

    bool isOccupied(size_t y, size_t x) const override {
        const Conditions::Color c = getPixel(y, x);
        return ! (c.red > 127 && c.green > 127 && c.blue > 127);
    }

    void setPixel(size_t y, size_t x, const Color &color) override {
        ompl::PPM::Color &c = ppm_.getPixel(y, x);
        c.red = color.red;
        c.green = color.green;
        c.blue = color.blue;
    }
};

#endif //CONDITIONSPPM_H
