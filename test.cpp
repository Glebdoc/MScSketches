#include <iostream>
#include <cmath>

class Point {
private:
    double x;
    double y;
    double z;

public:
    Point(double xVal, double yVal, double zVal) : x(xVal), y(yVal), z(zVal) {}

    void translate(double dx, double dy, double dz) {
        x += dx;
        y += dy;
        z += dz;
    }

    double getX() const { return x; }
    double getY() const { return y; }
    double getZ() const { return z; }
};

class Vortex {
private:
    double Gamma;
    Point p1;
    Point p2;

public:
    Vortex(const Point& point1, const Point& point2, double GammaVal)
        : p1(point1), p2(point2), Gamma(GammaVal) {}

    double getGamma() const { return Gamma; }


    double crossX(Point externalPoint) {
        return (externalPoint.getY() - p1.getY()) * (externalPoint.getZ() - p2.getZ()) - (externalPoint.getZ() - p1.getZ()) * (externalPoint.getY() - p2.getY());
    }

    double crossY(Point externalPoint) {
        return (externalPoint.getZ() - p1.getZ()) * (externalPoint.getX() - p2.getX()) - (externalPoint.getX() - p1.getX()) * (externalPoint.getZ() - p2.getZ());
    }

    double crossZ(Point externalPoint) {
        return (externalPoint.getX() - p1.getX()) * (externalPoint.getY() - p2.getY()) - (externalPoint.getY() - p1.getY()) * (externalPoint.getX() - p2.getX());
    }

    double crossMag(Point externalPoint) {
        return sqrt(pow(crossX(externalPoint), 2) + pow(crossY(externalPoint), 2) + pow(crossZ(externalPoint), 2));
    }


    Point getP1() const { return p1; }
    Point getP2() const { return p2; }
};

int main() {
    Point p1(1, 0, 0);
    Point p2(0, 0, 0);
    Point p3(1, 1, 0);
    Vortex vortex(p1, p2, 1);

    std::cout << "Vortex Cross X: " << vortex.crossX(p3) << std::endl;
    std::cout << "Vortex Cross Y: " << vortex.crossY(p3) << std::endl;
    std::cout << "Vortex Cross Z: " << vortex.crossZ(p3) << std::endl;
    std::cout << "Vortex Cross Mag: " << vortex.crossMag(p3) << std::endl;

    std::cout << "Vortex Gamma: " << vortex.getGamma() << std::endl;
    std::cout << "Vortex P1: (" << vortex.getP1().getX() << ", " << vortex.getP1().getY() << ", " << vortex.getP1().getZ() << ")" << std::endl;
    std::cout << "Vortex P2: (" << vortex.getP2().getX() << ", " << vortex.getP2().getY() << ", " << vortex.getP2().getZ() << ")" << std::endl;

    return 0;
}