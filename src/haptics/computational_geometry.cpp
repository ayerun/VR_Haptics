#include "computational_geometry.hpp"
#include <float.h>
#include <cmath>
#include <iostream>

namespace geometry {
    double normalize_angle(double rad) {
        while(rad > PI)
        {
            rad = rad-2*PI;
        }
        while(rad < -PI)
        {
            rad = rad+2*PI;
        }
        return rad;
    }
}

ExponentialFilter::ExponentialFilter(int n){
    alpha = 1;
    for (int i=0; i<n; i++) forecast.push_back(0);
    initialized = false;
}

ExponentialFilter::ExponentialFilter(int n, double a) {
    alpha = a;
    for (int i=0; i<n; i++) forecast.push_back(0);
    initialized = false;
}
void ExponentialFilter::filterData(std::vector<double>& x) {
    if (x.size() != forecast.size()) return;

    if (!initialized) {
        for (int i=0; i<x.size(); i++) forecast[i] = x[i];
        initialized = true;
        return;
    }
    
    for (int i=0; i<x.size(); i++) forecast[i] = alpha*x[i] + (1-alpha)*forecast[i];
    return;
}

std::vector<double> ExponentialFilter::getForcast() {
    return forecast;
}