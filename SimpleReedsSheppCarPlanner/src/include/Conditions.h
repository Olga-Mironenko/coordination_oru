#ifndef CONDITIONS_H
#define CONDITIONS_H

#include <memory>

#include <ompl/util/PPM.h>

#include "Footprint.h"

class Conditions {
protected:
    std::string mapId_;
    size_t numIterations_;
    double turningRadius_;
    std::shared_ptr<Footprint> footprint_;

public:
    Conditions(const std::string &mapId, const size_t numIterations, const double turningRadius,
               const std::shared_ptr<Footprint> &footprint)
        : mapId_(mapId),
          numIterations_(numIterations),
          turningRadius_(turningRadius),
          footprint_(footprint) {
    }

    virtual ~Conditions() = default;

    std::string computeId() const {
        std::stringstream ss;
        ss << mapId_ << "_" << numIterations_ << "_" << turningRadius_ << "_" << footprint_->computeId();
        return ss.str();
    }

    std::string computeFilenamePD() const {
        std::stringstream ss;
        ss << "tmp/" << computeId() << ".pd";
        return ss.str();
    }

    size_t getNumIterations() const {
        return numIterations_;
    }

    std::shared_ptr<Footprint> getFootprint() const {
        return footprint_;
    }

    double getTurningRadius() const {
        return turningRadius_;
    }

    virtual void loadFile(const std::string &filename) = 0;

    virtual size_t getWidth() const = 0;
    virtual size_t getHeight() const = 0;

    virtual ompl::PPM::Color getPixel(int y, int x) const = 0;
    virtual bool isPixelOccupied(int y, int x) const = 0;

    bool isPixelInBounds(int y, int x) const {
        return x >= 0 && y >= 0 && x < getWidth() && y < getHeight();
    }

    bool isFootprintOccupied(double x, double y, double t) const {
        return ! getFootprint()->isValid(this, x, y, t);
    }

protected:
    static bool isColorOccupied(const ompl::PPM::Color &c) {
        return !(c.red > 127 && c.green > 127 && c.blue > 127);
    }
};

#endif //CONDITIONS_H
