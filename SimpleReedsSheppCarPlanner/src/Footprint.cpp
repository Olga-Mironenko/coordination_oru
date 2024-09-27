#include <cmath>
#include <iostream>

#include "Conditions.h"
#include "Footprint.h"

bool Footprint::isValid(const Conditions *conditions, double x, double y, double t) const {
    const float xMetersCenter = x - mapOriginX;
    const float yMetersCenter = y - mapOriginY;
    const float theta = t;
    if (isDebug) std::cout << "Checking for collision in " << xMetersCenter << "," << yMetersCenter << "," << theta << std::endl;

    for (int i = 0; i < numCoords; i++) {
        const float xMeters = xCoords[i] * cos(theta) - yCoords[i] * sin(theta) + xMetersCenter;
        const float yMeters = xCoords[i] * sin(theta) + yCoords[i] * cos(theta) + yMetersCenter;

        //Position center in pixel (in map frame)
        const int xPixels = static_cast<int>(xMeters / mapResolution);
        int yPixels = static_cast<int>(yMeters / mapResolution);
        if (isYInverted) {
            yPixels = conditions->getHeight() - yPixels;
        }
        const int radiusPixels = ceil(radius / mapResolution);

        for (int dx = -radiusPixels; dx <= radiusPixels; dx += radiusPixels == 0 ? 1 : 2 * radiusPixels) {
            for (int dy = -radiusPixels; dy <= radiusPixels; dy += radiusPixels == 0 ? 1 : 2 * radiusPixels) {
                /* Each "circle":

                   1                    3

                     (xPixels, yPixels)

                   2                    4

                 */

                const int xCircle = xPixels + dx;
                const int yCircle = yPixels + dy;

                if (conditions->isPixelOccupied(yCircle, xCircle)) {
                    return false;
                }
            }
        }
    }

    if (isDebug) std::cout << "No collision found in " << xMetersCenter << "," << yMetersCenter << "," << theta << std::endl;

    return true;
}
