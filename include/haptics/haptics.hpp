#ifndef COMPUTATION_GEOMETRY_COMMUNICATION_GUARD
#define COMPUTATION_GEOMETRY_COMMUNICATION_GUARD

#include <vector>
#include <Eigen/Geometry>

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

double map(double val, double i_low, double i_high, double o_low, double o_high);

class Drum {
    public:
        Drum();
        Drum(int m_id, Eigen::Vector3f m_center, double m_length, double m_width, double m_k, std::pair<int,int>m_sustain_limits, std::pair<int,int> m_level_limits);
    
        bool withinDrumBoundaries(const Eigen::Vector3f &drumstick_position);
        double calculateTorque(const float &drumstick_z_position);

        bool checkContact(const float &drumstick_z_position);
        double calculateDistance(const Eigen::Vector3f &drumstick_position);
        void sendToPureData(const Eigen::Vector3f &drumstick_position, const double &drumstick_velocity);

        double update(const Eigen::Vector3f &drumstick_position, const double &drumstick_velocity);


    private:
        Eigen::Vector3f center;
        double width;
        double length;
        double k;
        std::pair<int,int> sustain_limits;
        std::pair<int,int> level_limits;
        int id;
};

class ExponentialFilter {
    public:
        ExponentialFilter(int n);
        explicit ExponentialFilter(int n, double a);

        void filterData(std::vector<double>& x);
        std::vector<double> getForcast();
        Eigen::Vector3f getForcastFloat();

    private:
        double alpha;
        std::vector<double> forecast;
        bool initialized;
};

#endif