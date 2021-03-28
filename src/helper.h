#ifndef HELPER_H
#define HELPER_H

#include <math.h>
#include <iostream>
#include <vector>

class Helper
{
public:
    Helper();
    
    inline  double pi() { return M_PI; }
    inline double deg2rad(double x) { return x * pi() / 180; }
    inline double rad2deg(double x) { return x * 180 / pi(); }
    std::string verifyData(std::string s);
    double getDistance(double x1, double y1, double x2, double y2);
    int getClosestWaypoint(double x, double y, const std::vector<double> &maps_x, const std::vector<double> &maps_y);
    int getNextWaypoint(double x, double y, double theta, const std::vector<double> &maps_x, const std::vector<double> &maps_y);
    std::vector<double> getFrenet(double x, double y, double theta, const std::vector<double> &maps_x, const std::vector<double> &maps_y);
    std::vector<double> getXY(double s, double d, const std::vector<double> &maps_s, const std::vector<double> &maps_x, const std::vector<double> &maps_y);
};

#endif // HELPER_H
