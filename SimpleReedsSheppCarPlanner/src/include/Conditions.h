#ifndef CONDITIONS_H
#define CONDITIONS_H

#include <memory>

#include <ompl/util/PPM.h>

#include "Footprint.h"

class Conditions {
protected:
    // Construction:
    std::string mapId_;
    size_t numIterationsConstruction_;
    double turningRadius_;
    std::shared_ptr<Footprint> footprint_;
    // Querying:
    size_t numIterationsSimplification_;
    size_t numAttemptsSimplification_ = 3;

public:
    Conditions(const std::string &mapId,
               const size_t numIterationsConstruction,
               const size_t numIterationsSimplification,
               const double turningRadius,
               const std::shared_ptr<Footprint> &footprint)
        : mapId_(mapId),
          numIterationsConstruction_(numIterationsConstruction),
          numIterationsSimplification_(numIterationsSimplification),
          turningRadius_(turningRadius),
          footprint_(footprint) {
    }

    virtual ~Conditions() = default;

    std::string computeIdConstruction() const {
        std::stringstream ss;
        ss << mapId_ << "_" << numIterationsConstruction_ << "_" << turningRadius_ << "_" << footprint_->computeId();
        return ss.str();
    }

    std::string computeFilenamePD() const {
        std::stringstream ss;
        ss << "tmp/" << computeIdConstruction() << ".pd";
        return ss.str();
    }

    size_t getNumIterationsConstruction() const {
        return numIterationsConstruction_;
    }

    size_t getNumIterationsSimplification() const {
        return numIterationsSimplification_;
    }

    size_t getNumAttemptsSimplification() const {
        return numAttemptsSimplification_;
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
