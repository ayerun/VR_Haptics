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

double map(double val, double i_low, double i_high, double o_low, double o_high) {
    double m = (o_high-o_low)/(i_high-i_low);
    double b = o_low-m*i_low;
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

Drum::Drum(int m_id, Eigen::Vector3f m_center, double m_length, double m_width, double m_k, std::pair<int,int> m_sustain_limits, std::pair<int,int> m_level_limits) {
    center = m_center;
    length = m_length;
    width = m_width;
    k = m_k;
    level_limits = m_level_limits;
    sustain_limits = m_sustain_limits;
    id = m_id;
}

bool Drum::withinDrumBoundaries(Eigen::Vector3f drumstick_position) {
    if (drumstick_position[0] > center[0]+width/2 || drumstick_position[0] < center[0]-width/2 || drumstick_position[1] > center[1]+length/2 || drumstick_position[1] < center[1]-length/2) return false;
    else return true;
}

double Drum::calculateDistance(Eigen::Vector3f drumstick_position) {
    double dist = sqrt( pow((drumstick_position[1]-center[1]),2) + pow((drumstick_position[0]-center[0]),2) );
    return dist;
}

double Drum::calculateTorque(float drumstick_z_position) {
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

bool Drum::checkContact(float drumstick_z_position) {
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

void Drum::sendToPureData(Eigen::Vector3f drumstick_position, double drumstick_velocity) {
    //calculate sustain command
    double distance_to_center = calculateDistance(drumstick_position);
    double sustain_input_limit = sqrt(pow(length,2)+pow(width,2))/2;
    double sustain_cmd = map(distance_to_center,0,sustain_input_limit,sustain_limits.second,sustain_limits.first);

    //calculate level command
    double level_cmd = map(drumstick_velocity,0,8,level_limits.first,level_limits.second);

    //send commands
    std::cout << id << " " << level_cmd << " " << sustain_cmd << ";" << std::endl;
}

double Drum::update(Eigen::Vector3f drumstick_position, double drumstick_velocity) {
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