#ifndef CONDITIONSOCCUPANCY_H
#define CONDITIONSOCCUPANCY_H

#include <cassert>
#include <cstdint>
#include <memory>
#include <string>

#include <ompl/util/PPM.h>

#include "Conditions.h"
#include "Footprint.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

class ConditionsOccupancy : public Conditions {
    const uint8_t* occupancyMap_;
    int mapWidth_;
    int mapHeight_;

public:
    ConditionsOccupancy(const std::string &mapId, const size_t numIterations, const double turningRadius,
                        const std::shared_ptr<Footprint> &footprint,
                        const uint8_t* occupancyMap, const int mapWidth, const int mapHeight)
        : Conditions(mapId, numIterations, turningRadius, footprint),
          occupancyMap_(occupancyMap), mapWidth_(mapWidth), mapHeight_(mapHeight) {}

    ConditionsOccupancy(const std::string &mapId, const size_t numIterations, const double turningRadius,
                        const std::shared_ptr<Footprint> &footprint,
                        const std::string &filenamePPMWithoutObstacles)
        : Conditions(mapId, numIterations, turningRadius, footprint) {
        ConditionsOccupancy::loadFile(filenamePPMWithoutObstacles);
    }

    void loadFile(const std::string &filename) override {
        ompl::PPM ppm;
        ppm.loadFile(filename.c_str());

        if (occupancyMap_ != nullptr) {
            assert(mapWidth_ == ppm.getWidth());
            assert(mapHeight_ == ppm.getHeight());
        } else {
            mapWidth_ = ppm.getWidth();
            mapHeight_ = ppm.getHeight();
        }

        const int totalPixels = mapWidth_ * mapHeight_;
        const int totalBytes = (totalPixels + 7) / 8;

        uint8_t* map = new uint8_t[totalBytes](); // the old memory should be free()d

        for (int y = 0; y < mapHeight_; ++y) {
            for (int x = 0; x < mapWidth_; ++x) {
                const int pixelNumber = y * mapWidth_ + x;

                const ompl::PPM::Color& c = ppm.getPixel(y, x);
                const uint8_t bit = isColorOccupied(c);

                map[pixelNumber / 8] |= bit << (pixelNumber % 8);
            }
        }

        occupancyMap_ = map;
    }

    size_t getWidth() const override {
        return mapWidth_;
    }

    size_t getHeight() const override {
        return mapHeight_;
    }

    ompl::PPM::Color getPixel(int y, int x) const override {
        return isPixelOccupied(y, x) ? ompl::PPM::Color{0, 0, 0} : ompl::PPM::Color{255, 255, 255};
    }

    bool isPixelOccupied(int y, int x) const override {
        if (! isPixelInBounds(y, x)) {
            return true;
        }
        const int pixelNumber = y * mapWidth_ + x;
        const uint8_t byte = occupancyMap_[pixelNumber / 8];
        const bool bit = (byte & (1 << (pixelNumber % 8))) != 0;
        return bit;
    }
};

#endif //CONDITIONSOCCUPANCY_H
