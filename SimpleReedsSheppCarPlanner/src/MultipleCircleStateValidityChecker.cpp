#include "MultipleCircleStateValidityChecker.h"


bool MultipleCircleStateValidityChecker::isOccupied(const int x, const int y) const {
    bool isOccupied = x < 0 || x >= mapWidth || y < 0 || y >= mapHeight;
    if (!isOccupied) {
        // See `OccupancyMap.java`, `asByteArray`.
        const int pixelNumber = y * mapWidth + x;
        const uint8_t byte = occupancyMap[ pixelNumber / 8 ];
        isOccupied = (byte & (1 << (pixelNumber % 8))) != 0;
    }
    if (isDebug) std::cout << "x=" << x << ", y=" << y << ": isOccupied=" << isOccupied << std::endl;

    return isOccupied;
}

bool MultipleCircleStateValidityChecker::isValid(const ob::State *state) const {
    if (isDebug) {
        for (int y = 0; y < 2; y++) {
            for (int x = 0; x < 120; x++) {
                isOccupied(x, y);
            }
        }
    }

    const ob::SE2StateSpace::StateType *s = state->as<ob::SE2StateSpace::StateType>();

    if (noMap) {
        //std::cout << "No map, so no collision! " << std::endl;
        return true;
    }

    // Position in meters in map frame.
    float xMetersCenter = static_cast<float>(s->getX()) - mapOriginX;
    float yMetersCenter = static_cast<float>(s->getY()) - mapOriginY;
    float theta = static_cast<float>(s->getYaw());
    if (isDebug) std::cout << "Checking for collision in " << xMetersCenter << "," << yMetersCenter << "," << theta << std::endl;

    for (int i = 0; i < numCoords; i++) {
        float xMeters = xCoords[i] * cos(theta) - yCoords[i] * sin(theta) + xMetersCenter;
        float yMeters = xCoords[i] * sin(theta) + yCoords[i] * cos(theta) + yMetersCenter;

        //Position center in pixel (in map frame)
        int xPixels = (int) (xMeters / mapResolution);
        int yPixels = mapHeight - (int) (yMeters / mapResolution);
        int radiusPixels = ceil(radius / mapResolution);

        for (int dx = -radiusPixels; dx <= radiusPixels; dx += radiusPixels == 0 ? 1 : 2 * radiusPixels) {
            for (int dy = -radiusPixels; dy <= radiusPixels; dy += radiusPixels == 0 ? 1 : 2 * radiusPixels) {
                /* Each "circle":

                   1                    3

                     (xPixels, yPixels)

                   2                    4

                 */

                const int x = xPixels + dx;
                const int y = yPixels + dy;

                if (isOccupied(x, y)) {
                    return false;
                }
            }
        }
    }

    //std::cout << "No collision found in " << refPoseX << "," << refPoseY << "," << refPoseTheta << std::endl;

    return true;
}
