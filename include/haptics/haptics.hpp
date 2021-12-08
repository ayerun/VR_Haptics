#ifndef COMPUTATION_GEOMETRY_COMMUNICATION_GUARD
#define COMPUTATION_GEOMETRY_COMMUNICATION_GUARD

/// \file
/// \brief Library with various helper functions and objects used to create a haptic drumkit

#include <vector>
#include <Eigen/Geometry>

namespace geometry {

    /// \brief PI.  Not in C++ standard until C++20.
    constexpr double PI=3.14159265358979323846;

    /// \brief convert degrees to radians
    /// \param deg - angle in degrees
    /// \returns radians
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

    /// \brief convert revolutions to degrees
    /// \param rev - number of revolutions
    /// \returns angle in degrees
    constexpr double rev2deg(double rev)
    {
        return rev*360;
    }

    /// \brief convert revolutions to radians
    /// \param rev - number of revolutions
    /// \returns angle in radians
    constexpr double rev2rad(double rev)
    {
        return rev*2*PI;
    }

    /// \brief converts a angle to [-PI PI]
    /// \param twist - twist to integrate
    /// \return transformation to new frame after twist
    double normalize_angle(double rad);
}

/// \brief linearlly maps a value from an input range to an output range
/// \param val - value to be mapped
/// \param input_range - input range of value
/// \param output_range - output range of vales
/// \returns mapped value
double map(double val, std::pair<double,double> input_range, std::pair<double,double> output_range);

/// \brief A haptic drum
class Drum {
    
    public:

        /// \brief creates 0.4 x 0.4 drum at origin
        Drum();

        /// \brief creates specified drum
        /// \param m_id - drum id
        /// \param m_center - drum center coordinates [m]
        /// \param m_length - drum length [m]
        /// \param m_width - drum width [m]
        /// \param m_k - drum spring constant [N/m]
        /// \param m_sustain_limits - drum sustain range [%]
        /// \param m_level_limits - drum amplifier
        Drum(int m_id, Eigen::Vector3f m_center, double m_length, double m_width, double m_k, std::pair<double,double>m_sustain_limits, std::pair<double,double> m_level_limits);
    
        /// \brief determines if drumstick is within drum volume
        /// \param drumstick_position - position of drumstick
        /// \returns true if drumstick is within drum
        bool withinDrumBoundaries(const Eigen::Vector3f &drumstick_position);

        /// \brief calculates the torque to be exerted on user
        /// \param drumstick_z_position - z coordinate of drumstick
        /// \returns torque [Nm]
        double calculateTorque(const float &drumstick_z_position);

        /// \brief checks if drumstick just made initial contact
        /// \param drumstick_z_position - z coordinate of drumstick
        /// \returns true if drumstick just impacted drum
        bool checkContact(const float &drumstick_z_position);

        /// \brief calculates the distance in the xy plane between the drumstick and the center of the drum
        /// \param drumstick_position - position of drumstick
        /// \returns distance
        double calculateDistance(const Eigen::Vector3f &drumstick_position);

        /// \brief computes sustain and level commands then sends commands to PD
        /// \param drumstick_position - position of drumstick
        /// \param drumstick_velocity - drumstick velocity in the z direction
        void sendToPureData(const Eigen::Vector3f &drumstick_position, const double &drumstick_velocity);

        /// \brief performs all necessary computations and communicates with PD
        /// \returns torque [Nm]
        double update(const Eigen::Vector3f &drumstick_position, const double &drumstick_velocity);


    private:
        Eigen::Vector3f center;                     //drum center [m]
        double width;                               //drum width [m]
        double length;                              //drum length [m]
        double k;                                   //spring constant [N/m]
        std::pair<double,double> sustain_limits;    //sustain range [%]
        std::pair<double,double> level_limits;      //level range
        int id;                                     //drum id
};


/// \brief Lowpass filter
class ExponentialFilter {

    public:

        /// \brief creates a filter
        /// \param n - filter size
        ExponentialFilter(int n);

        /// \brief creates a filter
        /// \param n - filter size
        /// \param a - filter constant
        explicit ExponentialFilter(int n, double a);

        /// \brief filters data and updates forecast
        /// \param x - new data
        void filterData(std::vector<double>& x);

        /// \brief getter function
        /// \returns forecast
        std::vector<double> getForcast();

        /// \brief getter function
        /// \returns forecast as Eigen::Vector3f
        Eigen::Vector3f getForcastFloat();

    private:
        double alpha;                   //filter constant
        std::vector<double> forecast;   //filtered values
        bool initialized;               //true if initialized
};

#endif