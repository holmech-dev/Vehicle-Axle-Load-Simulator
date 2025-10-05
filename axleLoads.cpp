#include <iostream>
#include <vector>
#include "axleLoads.hpp"

/********************
Program    - Axle Load Model - Axle Load Fncs
Maintainer - C.Holmes
File       - Program Header
Version    - 0
    - Release Notes:
        - Version 0   - Axle Load functions
********************/

// Nonlinear load Model
AxleData CalculateAxleLoads (
    const VehicleParams& vp,
    double thetaMin, double thetaMax, int thetaSteps,
    double accelMin, double accelMax, int accelSteps
) {
    AxleData data;

    double W = vp.m * g; // Total Load
    data.theta.resize(thetaSteps); // ?
    data.accel.resize(accelSteps); // ?
    data.WF.resize(thetaSteps, std::vector<double>(accelSteps)); // ?
    data.WR.resize(thetaSteps, std::vector<double>(accelSteps)); // ?

    // // Create the slope and accel vectors
    for (int i = 0; i < thetaSteps; ++i)
        data.theta[i] = thetaMin + i * (thetaMax - thetaMin) / (thetaSteps - 1);

    for (int j = 0; j < accelSteps; ++j)
        data.accel[j] = accelMin + j * (accelMax - accelMin) / (accelSteps - 1);

    // Compute Loads
    for (int i = 0; i < thetaSteps; ++i) {
        double thetaI = data.theta[i];
        for (int j = 0; j < accelSteps; ++j) {
            double aI = data.accel[j];
            
            double WF = (vp.lr / vp.L) * W * std::cos(thetaI) - (vp.h / vp.L) * vp.m * (std::sin(thetaI) + aI / g);

            double WR = (vp.lf / vp.L) * W * std::cos(thetaI) + (vp.h / vp.L) * vp.m * (std::sin(thetaI) + aI / g);

            data.WF[i][j] = WF;
            data.WR[i][j] = WR;
        }
    }
    return data;
}


// Nominal Axle Loads at the operating point
std::pair<double, double> CalculateNominalAxleLoads(
    const VehicleParams& vp, 
    double thetaNom, 
    double accelNom
) {
    double W = vp.m * g;

    double WFOp = (vp.lr / vp.L) * W * std::cos(thetaNom) - (vp.h / vp.L) * vp.m * (std::sin(thetaNom) + accelNom / g);
    
    double WROp = (vp.lf / vp.L) * W * std::cos(thetaNom) + (vp.h / vp.L) * vp.m * (std::sin(thetaNom) + accelNom / g);

    return {WFOp, WROp};
}
