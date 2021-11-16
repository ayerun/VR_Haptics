#ifndef COMPUTATION_GEOMETRY_COMMUNICATION_GUARD
#define COMPUTATION_GEOMETRY_COMMUNICATION_GUARD

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

#endif