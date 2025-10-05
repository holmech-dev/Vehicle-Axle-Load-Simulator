
/********************
Program    - Axle Load Model - Axle Load Fncs
Maintainer - C.Holmes
File       - Program Header
Version    - 0
    - Release Notes:
        - Version 0   - Class structure for Axle Load functions
********************/

#ifndef AXLE_LOAD_H
#define AXLE_LOAD_H

// Global Constants
const double g = 9.81; // gravity m/s^2

struct VehicleParams {
    double m;  // Mass
    double h;  // CoG height
    double L;  // Wheelbase
    double lf; // Front to CoG Length
    double lr; // Rear to CoG Length
};

struct AxleData {
    std::vector<double> theta;           // Slope Angles (rad)
    std::vector<double> accel;           // Accelerations(m/s^2)
    std::vector<std::vector<double>> WF; // Front Axle Load
    std::vector<std::vector<double>> WR; // Rear Axle Load
};

// Nonlinear load Model
AxleData CalculateAxleLoads (const VehicleParams&, double, double, int, double, double, int);

// Nominal Axle Loads at the operating point
std::pair<double, double> CalculateNominalAxleLoads(const VehicleParams&, double, double);

#endif // AXLE_LOAD_H
