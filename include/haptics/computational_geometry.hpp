#ifndef COMPUTATION_GEOMETRY_COMMUNICATION_GUARD
#define COMPUTATION_GEOMETRY_COMMUNICATION_GUARD

#include <vector>

namespace geometry {

    constexpr double PI=3.14159265358979323846;

    /// \brief convert degrees to radians
    /// \param deg - angle in degrees
    /// \returns radians
    /// NOTE: implement this in the header file
    /// constexpr means that the function can be computed at compile time
    /// if given a compile-time constant as input
    constexpr double deg2rad(double deg)
    {
        return (deg*PI)/180;
    }

    /// \brief convert radians to degrees
    /// \param rad - angle in radians
    /// \returns the angle in degrees
    constexpr double rad2deg(double rad)
    {
        return (rad*180)/PI;
    }

    constexpr double rev2deg(double rev)
    {
        return rev*360;
    }

    constexpr double rev2rad(double rev)
    {
        return rev*2*PI;
    }

    /// \brief converts a angle to [-PI PI]
    /// \param twist - twist to integrate
    /// \return transformation to new frame after twist
    double normalize_angle(double rad);
}

class ExponentialFilter {
    public:
        ExponentialFilter();
        explicit ExponentialFilter(double a);

        double filterData(double x);

    private:
        double alpha;
        double forecast;
        bool initialized;
};

class ExponentialFilter2 {
    public:
        ExponentialFilter2(int n);
        explicit ExponentialFilter2(int n, double a);

        void filterData(std::vector<double>& x);
        std::vector<double> getForcast();

    private:
        double alpha;
        std::vector<double> forecast;
        bool initialized;
};

class LowPassFilter {
    public:
        LowPassFilter();
        explicit LowPassFilter(double cf);

        double filterData(double height, double time);

    private:
        double last_time;
        double last_height;
        double cutoff;
        double last_filtered_val;
        double count = 0;
};

#endif