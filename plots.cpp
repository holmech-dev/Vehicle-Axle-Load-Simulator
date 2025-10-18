#include "plots.hpp"
#include <string>
#include <vector>
#include <cstdio>
#include <algorithm>
#include <cmath>

/********************
Program    - Axle Load Model - PLotting Fncs
Maintainer - C.Holmes
File       - Program Header
Version    - 0
    - Release Notes:
        - Version 0   - Row 1 plots for axle loads
********************/

// ImGui/ImPlot headers are included via plots.hpp

// Render the axle load plots (front+rear vs slope, front+rear vs acceleration)
void RenderAxleLoadPlots(const VehicleParams& vp,
                         const AxleData& AxleLoadData,
                         double thetaNom,
                         double accelNom,
                         double WF0,
                         double WR0) {
    if (AxleLoadData.WF.empty() || AxleLoadData.theta.empty() || AxleLoadData.accel.empty())
        return;

    const int nThetaAvail = (int)AxleLoadData.theta.size();
    const int nThetaRows  = (int)AxleLoadData.WF.size();
    const int rows = nThetaAvail < nThetaRows ? nThetaAvail : nThetaRows;
    const int nAccel = (int)AxleLoadData.accel.size();

    // Compute equal widths for two side-by-side plots
    float full_row = ImGui::GetContentRegionAvail().x;
    float spacing = ImGui::GetStyle().ItemSpacing.x;
    float plot_w = (full_row - spacing) * 0.5f;

    // Row: Front & Rear vs Slope
    if (rows > 0 && nAccel > 0) {
        if (ImPlot::BeginPlot("Axle Loads vs Slope", ImVec2(plot_w, 0))) {
            // Primary X axis in radians, add a secondary top X axis in degrees for reference
            ImPlot::SetupAxes("Slope (rad)", "Front Load (N)", ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
            // Enable secondary X2 axis first, then set its limits/format
            ImPlot::SetupAxis(ImAxis_X2, "Slope (deg)");
            // Ensure X and X2 share the same limits so 0 aligns across both axes
            if (!AxleLoadData.theta.empty()) {
                double xmin = AxleLoadData.theta.front();
                double xmax = AxleLoadData.theta.back();
                if (xmax < xmin) std::swap(xmin, xmax);
                ImPlot::SetupAxisLimits(ImAxis_X1, xmin, xmax, ImPlotCond_Once);
                ImPlot::SetupAxisLimits(ImAxis_X2, xmin, xmax, ImPlotCond_Once);
            }
            ImPlot::SetupAxisFormat(ImAxis_X2,
                [](double v, char* buff, int size, void*) {
                    double deg = v * 180.0 / 3.14159265358979323846;
                    return std::snprintf(buff, size, "%.1f", deg);
                }
            );
            const int j_idx[2] = {0, nAccel - 1};
            auto fmt = [](double v){ char b[32]; std::snprintf(b, sizeof(b), "%.2f", v); return std::string(b); };
            const double a_min = AxleLoadData.accel.front();
            const double a_max = AxleLoadData.accel.back();
            const std::string lblFrontAmin = std::string("Front Load (min a=") + fmt(a_min) + ")";
            const std::string lblFrontAmax = std::string("Front Load (max a=") + fmt(a_max) + ")";
            for (int k = 0; k < 2; ++k) {
                const int j = j_idx[k];
                if (j < 0 || j >= nAccel) continue;
                bool row_ok = true;
                std::vector<double> y(rows, 0.0);
                for (int i = 0; i < rows; ++i) {
                    if ((int)AxleLoadData.WF[i].size() != nAccel) { row_ok = false; break; }
                    y[i] = AxleLoadData.WF[i][j];
                }
                if (row_ok) {
                    const char* label = (k == 0) ? lblFrontAmin.c_str() : lblFrontAmax.c_str();
                    ImPlot::PlotLine(label, AxleLoadData.theta.data(), y.data(), rows);
                }
            }

            // Tangent (linearized) traces around the operating point show how well the
            // small-angle approximation holds across the slope range.
            if (rows > 0) {
                const double theta_op = thetaNom;
                const double frontSlope =
                    -(vp.lr / vp.L) * vp.m * g * std::sin(theta_op) -
                    (vp.h  / vp.L) * vp.m * std::cos(theta_op);
                const double rearSlope =
                    -(vp.lf / vp.L) * vp.m * g * std::sin(theta_op) +
                    (vp.h  / vp.L) * vp.m * std::cos(theta_op);

                std::vector<double> frontLinear(rows, 0.0);
                std::vector<double> rearLinear(rows, 0.0);
                for (int i = 0; i < rows; ++i) {
                    const double theta_i = AxleLoadData.theta[i];
                    const double deltaTheta = theta_i - theta_op;
                    frontLinear[i] = WF0 + frontSlope * deltaTheta;
                    rearLinear[i]  = WR0 + rearSlope  * deltaTheta;
                }

                ImPlot::SetNextLineStyle(ImVec4(1.0f, 0.7f, 0.0f, 1.0f), 1.5f);
                ImPlot::PlotLine("Front Linearized (OP tangent)",
                                 AxleLoadData.theta.data(), frontLinear.data(), rows);
                ImPlot::SetNextLineStyle(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), 1.5f);
                ImPlot::PlotLine("Rear Linearized (OP tangent)",
                                 AxleLoadData.theta.data(), rearLinear.data(), rows);
                ImPlot::SetNextLineStyle(ImVec4(0.0f, 0.0f, 0.0f, 0.0f)); // restore defaults
            }

            // Overlay rear axle traces at same accel slices (min/max)
            const std::string lblRearAmin = std::string("Rear Load (min a=") + fmt(a_min) + ")";
            const std::string lblRearAmax = std::string("Rear Load (max a=") + fmt(a_max) + ")";
            for (int k = 0; k < 2; ++k) {
                const int j = j_idx[k];
                if (j < 0 || j >= nAccel) continue;
                bool row_ok = true;
                std::vector<double> yR(rows, 0.0);
                for (int i = 0; i < rows; ++i) {
                    if ((int)AxleLoadData.WR[i].size() != nAccel) { row_ok = false; break; }
                    yR[i] = AxleLoadData.WR[i][j];
                }
                if (row_ok) {
                    const char* label = (k == 0) ? lblRearAmin.c_str() : lblRearAmax.c_str();
                    ImPlot::PlotLine(label, AxleLoadData.theta.data(), yR.data(), rows);
                }
            }

            // Operating point markers from model: x = thetaNom, y = WF0/WR0
            const double xOP[1] = {thetaNom};
            const double yOP[1] = {WF0};
            const double yOPr[1] = {WR0};
            ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle, 6.0f, ImVec4(1,0,0,1), 2.0f, ImVec4(1,1,1,1));
            ImPlot::PlotScatter("Front OP", xOP, yOP, 1);
            ImPlot::SetNextMarkerStyle(ImPlotMarker_Asterisk, 6.0f, ImVec4(1,0,0,1), 2.0f, ImVec4(2,1,1,1));
            ImPlot::PlotScatter("Rear OP", xOP, yOPr, 1);

            ImPlot::EndPlot();
        }
    }

    // Place the acceleration plot on the same row as the slope plot
    if (nAccel > 0 && nThetaRows > 0) {
        ImGui::SameLine();
        if (ImPlot::BeginPlot("Axle Loads vs Accel", ImVec2(plot_w, 0))) {
            ImPlot::SetupAxes("Acceleration (m/s^2)", "Front Load (N)", ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
            const int row_idx[2] = {0, rows - 1};
            auto fmt2 = [](double v){ char b[32]; std::snprintf(b, sizeof(b), "%.3f", v); return std::string(b); };
            const double th_min = AxleLoadData.theta.front();
            const double th_max = AxleLoadData.theta.back();
            const std::string lblFrontTmin = std::string("Front Load (min theta =") + fmt2(th_min) + " rad)";
            const std::string lblFrontTmax = std::string("Front Load (max theta =") + fmt2(th_max) + " rad)";
            for (int k = 0; k < 2; ++k) {
                const int i = row_idx[k];
                if (i < 0 || i >= nThetaRows) continue;
                if ((int)AxleLoadData.WF[i].size() == nAccel) {
                    const char* label = (k == 0) ? lblFrontTmin.c_str() : lblFrontTmax.c_str();
                    ImPlot::PlotLine(label, AxleLoadData.accel.data(), AxleLoadData.WF[i].data(), nAccel);
                }
            }

            // Overlay rear axle traces at the same slope slices (min/max θ)
            const std::string lblRearTmin = std::string("Rear Load (min theta =") + fmt2(th_min) + " rad)";
            const std::string lblRearTmax = std::string("Rear Load (max theta =") + fmt2(th_max) + " rad)";
            for (int k = 0; k < 2; ++k) {
                const int i = row_idx[k];
                if (i < 0 || i >= nThetaRows) continue;
                if ((int)AxleLoadData.WR[i].size() == nAccel) {
                    const char* label = (k == 0) ? lblRearTmin.c_str() : lblRearTmax.c_str();
                    ImPlot::PlotLine(label, AxleLoadData.accel.data(), AxleLoadData.WR[i].data(), nAccel);
                }
            }

            // Operating point markers from model: x = accelNom, y = WF0/WR0
            const double xOP2[1] = {accelNom};
            const double yOP2[1] = {WF0};
            const double yOP2r[1] = {WR0};
            ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle, 6.0f, ImVec4(1,0,0,1), 2.0f, ImVec4(1,1,1,1));
            ImPlot::PlotScatter("Front OP", xOP2, yOP2, 1);
            ImPlot::SetNextMarkerStyle(ImPlotMarker_Asterisk, 6.0f, ImVec4(1,0,0,1), 2.0f, ImVec4(1,1,1,1));
            ImPlot::PlotScatter("Rear OP", xOP2, yOP2r, 1);

            ImPlot::EndPlot();
        }
    }
}
ControlResult RenderRangeControls(PlotRanges& ranges, const PlotRanges& defaults) {
    ControlResult res{false,false};

    ImGui::TextUnformatted("Input Ranges");
    ImGui::Separator();
    ImGui::PushID("ranges");

    // Theta controls (min/max)
    ImGui::SetNextItemWidth(140);
    ImGui::InputDouble("theta min (rad)", &ranges.thetaMin, 0.01, 0.1, "%.3f");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(140);
    ImGui::InputDouble("theta max (rad)", &ranges.thetaMax, 0.01, 0.1, "%.3f");

    // Accel controls (min/max)
    ImGui::SetNextItemWidth(140);
    ImGui::InputDouble("accel min (m/s^2)", &ranges.accelMin, 0.1, 1.0, "%.2f");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(140);
    ImGui::InputDouble("accel max (m/s^2)", &ranges.accelMax, 0.1, 1.0, "%.2f");

    // Buttons
    if (ImGui::Button("Apply")) {
        if (ranges.thetaMax < ranges.thetaMin) std::swap(ranges.thetaMin, ranges.thetaMax);
        if (ranges.accelMax < ranges.accelMin) std::swap(ranges.accelMin, ranges.accelMax);
        res.apply = true;
    }
    ImGui::SameLine();
    if (ImGui::Button("Reset")) {
        ranges = defaults;
        // Do not set apply=true automatically; user can hit Apply to recompute
        res.reset = true;
    }

    ImGui::PopID();
    ImGui::Separator();
    return res;
}
