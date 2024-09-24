#ifndef CONDITIONS_H
#define CONDITIONS_H

namespace ob = ompl::base;
namespace og = ompl::geometric;

class Conditions {
public:
    virtual size_t getNumIterations() const = 0;

    virtual std::string computeFilenamePD() const = 0;

    virtual void loadFile(const std::string &filename) = 0;
    virtual void saveFile(const std::string &filenameResult) = 0;

    virtual size_t getWidth() const = 0;
    virtual size_t getHeight() const = 0;

    bool isPixelInBounds(int y, int x) const {
        return x >= 0 && y >= 0 && x < getWidth() && y < getHeight();
    }

    struct Color {
        unsigned char red;
        unsigned char green;
        unsigned char blue;
    };

    virtual Color getPixel(int y, int x) const = 0;
    virtual bool isPixelOccupied(int y, int x) const = 0;
    virtual bool isFootprintOccupied(double x, double y, double t) const = 0;

    virtual void setPixel(int y, int x, const Color &color) = 0;

    virtual ~Conditions() = default;
};

#endif //CONDITIONS_H
