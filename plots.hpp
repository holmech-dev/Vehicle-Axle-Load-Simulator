
/********************
Program    - Axle Load Model - PLotting Fncs
Maintainer - C.Holmes
File       - Program Header
Version    - 0
    - Release Notes:
        - Version 0   - Class structure for PLotting functions
********************/

#ifndef PLOT_H
#define PLOT_H

// External deps
#include <vector>
#include "imgui.h"
#include "implot.h"

// Model types
#include "axleLoads.hpp"

// Simple container for UI-editable ranges
struct PlotRanges {
    double thetaMin;
    double thetaMax;
    double accelMin;
    double accelMax;
};

// Result of rendering range controls
struct ControlResult { bool apply; bool reset; };

// Renders controls above the plots:
// - 4 numeric inputs (theta min/max, accel min/max)
// - Apply and Reset buttons
// Returns ControlResult: apply=true when Apply clicked (caller should recompute),
// reset=true when Reset clicked (caller may restore additional UI defaults).
ControlResult RenderRangeControls(PlotRanges& ranges, const PlotRanges& defaults);

// Render the two side-by-side axle load plots inside an active ImGui window.
// Inputs:
// - vp: vehicle parameters
// - data: grids of theta, accel, WF, WR
// - thetaNom/accelNom: nominal operating point
// - WF0/WR0: operating point axle loads computed from CalculateNominalAxleLoads
void RenderAxleLoadPlots(const VehicleParams& vp,
                         const AxleData& data,
                         double thetaNom,
                         double accelNom,
                         double WF0,
                         double WR0);

#endif // PLOT_H
