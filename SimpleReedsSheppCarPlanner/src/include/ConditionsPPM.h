#ifndef CONDITIONSPPM_H
#define CONDITIONSPPM_H

#include <ompl/util/PPM.h>

#include "Conditions.h"
#include "Footprint.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

class ConditionsPPM : public Conditions {
protected:
    ompl::PPM ppm_;

public:
    ConditionsPPM(const std::string &mapId,
                  const size_t numIterationsConstruction,
                  const size_t numIterationsSimplification,
                  const double turningRadius,
                  const std::shared_ptr<Footprint> &footprint,
                  const std::string &filenamePPMWithoutObstacles)
        : Conditions(mapId, numIterationsConstruction, numIterationsSimplification, turningRadius, footprint) {
        ConditionsPPM::loadFile(filenamePPMWithoutObstacles);
    }

    void loadFile(const std::string &filename) override {
        const size_t width = ppm_.getWidth();
        const size_t height = ppm_.getHeight();

        ppm_.loadFile(filename.c_str());

        if (width != 0) {
            assert(width == ppm_.getWidth());
            assert(height == ppm_.getHeight());
        }
    }

    void saveFile(const std::string &filenameResult) {
        ppm_.saveFile(filenameResult.c_str());
    }

    size_t getWidth() const override {
        return ppm_.getWidth();
    }

    size_t getHeight() const override {
        return ppm_.getHeight();
    }

    ompl::PPM::Color getPixel(int y, int x) const override {
        return ppm_.getPixel(y, x);
    }

    bool isPixelOccupied(int y, int x) const override {
        if (! isPixelInBounds(y, x)) {
            return true;
        }
        const ompl::PPM::Color c = getPixel(y, x);
        return isColorOccupied(c);
    }

    void setPixel(int y, int x, const ompl::PPM::Color &color) {
        ompl::PPM::Color &c = ppm_.getPixel(y, x);
        c.red = color.red;
        c.green = color.green;
        c.blue = color.blue;
    }
};

#endif //CONDITIONSPPM_H
