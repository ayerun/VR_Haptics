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

ExponentialFilter::ExponentialFilter(){
    alpha = 1;
    forecast = 0;
    initialized = false;
}
ExponentialFilter::ExponentialFilter(double a) {
    alpha = a;
    forecast = 0;
    initialized = false;
}
double ExponentialFilter::filterData(double x) {
    if (!initialized) {
        forecast = x;
        initialized = true;
        return forecast;
    }
    
    forecast = alpha*x + (1-alpha)*forecast;
    return forecast;
}

LowPassFilter::LowPassFilter() {
    last_time = 0;
    last_height = 0;
    cutoff = DBL_MAX;
    last_filtered_val = 0; 
}

LowPassFilter::LowPassFilter(double cf) {
    last_time = 0;
    last_height = 0;
    cutoff = cf;
    last_filtered_val = 0; 
}

double LowPassFilter::filterData(double height, double time) {
    double speed = abs((height-last_height)/(time-last_time));
    last_time = time;
    last_height = height;
    if(speed > cutoff) {
        count++;
        std::cout << count << std::endl;
        return last_filtered_val;
    }
    else {
        last_filtered_val = height;
        return height;
    }
}