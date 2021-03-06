#include <haptics.hpp>
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

double map(double val, std::pair<double,double> input_range, std::pair<double,double> output_range) {
    double m = (output_range.second-output_range.first)/(input_range.second-input_range.first);
    double b = output_range.first-m*input_range.first;
    double mapped_val = m*val+b;
    return mapped_val;
}

Drum::Drum() {
    center << 0.0, 0.0, 0.0;
    width = 0.4;
    length = 0.4;
    sustain_limits.first = 0;
    sustain_limits.second = 500;
    level_limits.first = 0;
    level_limits.second = 3;
    id = 0;
}

Drum::Drum(int m_id, Eigen::Vector3f m_center, double m_length, double m_width, double m_k, std::pair<double,double> m_sustain_limits, std::pair<double,double> m_level_limits) {
    center = m_center;
    length = m_length;
    width = m_width;
    k = m_k;
    level_limits = m_level_limits;
    sustain_limits = m_sustain_limits;
    id = m_id;
}

bool Drum::withinDrumBoundaries(const Eigen::Vector3f& drumstick_position) {
    if (drumstick_position[0] > center[0]+width/2 || drumstick_position[0] < center[0]-width/2 || drumstick_position[1] > center[1]+length/2 || drumstick_position[1] < center[1]-length/2) return false;
    else return true;
}

double Drum::calculateDistance(const Eigen::Vector3f &drumstick_position) {
    double dist = sqrt( pow((drumstick_position[1]-center[1]),2) + pow((drumstick_position[0]-center[0]),2) );
    return dist;
}

double Drum::calculateTorque(const float &drumstick_z_position) {
    double torque_lim = DBL_MAX;
    double displacement = drumstick_z_position-center[2];

    if (displacement < 0) {

        //square displacement to make K units N/m
        displacement = pow(displacement,2);

        //calculate toque
        double torque = abs(k*displacement);

        //clamp torque
        torque = std::min(torque,torque_lim);

        return torque;
    }
    else return 0;
}

bool Drum::checkContact(const float &drumstick_z_position) {
    static bool drum_contact = false;   //is pointer touching drum currently?
    static bool last_reading = false;   //was pointer touching drum during last reading?

    double displacement = drumstick_z_position-center[2];

    //determine if pointer is touching drum
    if (displacement >= 0) drum_contact = false;
    else drum_contact = true;

    //return true if the contact just occured
    if (drum_contact == true && last_reading == false) {
        last_reading = drum_contact;
        return true;
    }
    else {
        last_reading = drum_contact;
        return false;
    }
}

void Drum::sendToPureData(const Eigen::Vector3f &drumstick_position, const double &drumstick_velocity) {
    //calculate sustain command
    double distance_to_center = calculateDistance(drumstick_position);
    double sustain_input_limit = sqrt(pow(length,2)+pow(width,2))/2;
    std::pair<double,double> sustain_input_range{sustain_input_limit,0};
    double sustain_cmd = map(distance_to_center,sustain_input_range,sustain_limits);

    //calculate level command
    std::pair<double,double> level_input_range{0,8};
    double level_cmd = map(drumstick_velocity,level_input_range,level_limits);

    //send commands
    std::cout << id << " " << level_cmd << " " << sustain_cmd << ";" << std::endl;
}

double Drum::update(const Eigen::Vector3f &drumstick_position, const double &drumstick_velocity) {
    //enforce drum boundaries
    if (!withinDrumBoundaries(drumstick_position)) return 0;

    //Send value commands to PD if this is first sign of contact
    if (checkContact(drumstick_position[2])) sendToPureData(drumstick_position,drumstick_velocity);

    //calculate torque
    double torque = calculateTorque(drumstick_position[2]);
    return torque;
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

Eigen::Vector3f ExponentialFilter::getForcastFloat() {
    float x = forecast[0];
    float y = forecast[1];
    float z = forecast[2];
    Eigen::Vector3f forecast_vector;
    forecast_vector << x,y,z;
    return forecast_vector;
}