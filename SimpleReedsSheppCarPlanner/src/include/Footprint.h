#ifndef FOOTPRINT_H
#define FOOTPRINT_H

#include <sstream>

class Conditions;

class Footprint {
protected:
    bool isPoint;
    bool isYInverted;

    double mapResolution;
    double mapOriginX;
    double mapOriginY;
    double radius;

    const double* xCoords;
    const double* yCoords;
    int numCoords;

    bool isDebug = false;

public:
    Footprint() : mapResolution(1), mapOriginX(0), mapOriginY(0), radius(0) {
        isPoint = true;
        isYInverted = false;

        numCoords = 1;
        xCoords = new double[numCoords]{0.0};
        yCoords = new double[numCoords]{0.0};
    }

    ~Footprint() {
        if (isPoint) {
            delete[] xCoords;
            delete[] yCoords;
        }
    }

    std::string computeId() const {
        std::stringstream ss;
        for (int i = 0; i < numCoords; i++) {
            ss << xCoords[i] << "," << yCoords[i] << "_";
        }
        ss << (isYInverted ? "inv" : "noninv");
        return ss.str();
    }

    Footprint(
        double _mapResolution, double _mapOriginX, double _mapOriginY, double _radius,
        const double* _xCoords, const double* _yCoords, int _numCoords, bool _isYInverted
        ) {
        isPoint = false;
        isYInverted = _isYInverted;

        mapResolution = _mapResolution;
        mapOriginX = _mapOriginX;
        mapOriginY = _mapOriginY;
        radius = _radius;

        xCoords = _xCoords;
        yCoords = _yCoords;
        numCoords = _numCoords;
    }

    bool isValid(const Conditions *conditions, double x, double y, double t) const;
};

#endif //FOOTPRINT_H
