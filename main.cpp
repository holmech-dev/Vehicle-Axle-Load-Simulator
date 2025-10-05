#include <iostream>
#include <vector>
#include <cmath>
#include <cstdio>
#include <string>

// ImGui + GLFW
#include "imgui.h"
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"
#include <GLFW/glfw3.h>
// ImPlot
#include "implot.h"
// Plot helpers
#include "plots.hpp"
// Model Includes
#include "axleLoads.hpp"

/********************
Program    - Axle Load Modelling for Brake Redistribution
Maintainer - C.Holmes
File       - Main Program
Version    - 0
    - Release Notes:
        - Version 0   - Program Test Structure
            -- build -> ✅
            -- run   -> ./WheelLoadDistributor
********************/


int main() {
    // Init GLFW
    if (!glfwInit()) return -1;

    // Configure GL context + GLSL version per platform
    const char* glsl_version = "#version 130";
    #if defined(IMGUI_IMPL_OPENGL_ES2)
        glsl_version = "#version 100";
    #elif defined(__APPLE__)
        glsl_version = "#version 150";
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
        glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    #else
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    #endif

    GLFWwindow* window = glfwCreateWindow(700, 500, "Brake Bias Simulator", nullptr, nullptr);
    if (!window) {
        std::cerr << "Failed to create GLFW window. Check that a compatible OpenGL context is available." << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // Init ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();  
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    ImGui::StyleColorsDark();
    ImGui::GetStyle().ScaleAllSizes(0.9f); ImGui::GetIO().FontGlobalScale = 0.95f;
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    std::cout << "Main Program Entrypoint..." << std::endl;

    //********************//
    // Model Function Calls   
    //********************//
    VehicleParams vp{1475.0, 0.55, 2.636, 1.0544, 1.5816};  // m, h, L, CoG Fr, CoG Rr
    // , 5 slopes, 100 accel points
    AxleData AxleLoadData = CalculateAxleLoads(vp, -0.3, 0.3, 5, -10.0, 10.0, 100); // vp, thetaMin -0.1, thetaMax 0.1, thetaSteps 5, accelMin -6, accelMax 6, accelSteps 100
    double thetaNom = 0.0;
    double accelNom = 0.0;
    auto [WF0, WR0]  = CalculateNominalAxleLoads(vp, thetaNom, accelNom);
    std::cout << "Nominal Front Load: " << WF0 << " N\n";
    std::cout << "Nominal Rear Load:  " << WR0 << " N\n";



    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::Begin("Brake Bias Feedforward Simulator");

        // Vehicle parameters (above ranges)
        ImGui::TextUnformatted("Vehicle Parameters");
        ImGui::Separator();
        static double ui_m   = vp.m;   // kg
        static double ui_h   = vp.h;   // m
        static double ui_L   = vp.L;   // m
        // Front/Rear mass distribution (%) — initialize from current geometry
        // Front mass % = lr / L; Rear mass % = lf / L
        static double ui_cogFrPct = (vp.lr / vp.L) * 100.0; // % mass on front axle
        static double ui_cogRrPct = (vp.lf / vp.L) * 100.0; // % mass on rear axle
        static const double def_ui_m = ui_m;
        static const double def_ui_h = ui_h;
        static const double def_ui_L = ui_L;
        static const double def_ui_cogFrPct = ui_cogFrPct;
        static const double def_ui_cogRrPct = ui_cogRrPct;
        ImGui::SetNextItemWidth(140); ImGui::InputDouble("mass (kg)", &ui_m, 10.0, 100.0, "%.1f");
        ImGui::SameLine();             ImGui::SetNextItemWidth(140); ImGui::InputDouble("CoG height h (m)", &ui_h, 0.01, 0.1, "%.3f");
        ImGui::SameLine();             ImGui::SetNextItemWidth(140); ImGui::InputDouble("wheelbase L (m)", &ui_L, 0.01, 0.1, "%.3f");
        ImGui::SetNextItemWidth(140); ImGui::InputDouble("Front mass (%)", &ui_cogFrPct, 0.5, 5.0, "%.2f");
        ImGui::SameLine();             ImGui::SetNextItemWidth(140); ImGui::InputDouble("Rear mass (%)",  &ui_cogRrPct, 0.5, 5.0, "%.2f");
        ImGui::Separator();

        // Range controls (above plots). Defaults mirror initial computation above.
        static PlotRanges ranges{-0.35, 0.35, -10.0, 10.0};
        static const PlotRanges defaults{-0.35, 0.35, -10.0, 10.0};
        const int thetaSteps = 5;
        const int accelSteps = 100;

        if (auto ctrl = RenderRangeControls(ranges, defaults); true) {
            if (ctrl.reset) {
                // Restore UI vehicle parameters to defaults
                ui_m = def_ui_m;
                ui_h = def_ui_h;
                ui_L = def_ui_L;
                ui_cogFrPct = def_ui_cogFrPct;
                ui_cogRrPct = def_ui_cogRrPct;
            }
            if (ctrl.apply) {
                // Apply updated vehicle parameters (convert CoG % to distances using L)
                vp.m = ui_m;
                vp.h = ui_h;
                vp.L = ui_L;
                // Convert mass percentages to distances using L
                double pF = ui_cogFrPct / 100.0;
                double pR = ui_cogRrPct / 100.0;
                double sum = pF + pR;
                if (sum > 0.0) { pF /= sum; pR /= sum; }
                // Front mass % = lr/L; Rear mass % = lf/L
                vp.lr = pF * vp.L;
                vp.lf = pR * vp.L;

                AxleLoadData = CalculateAxleLoads(
                    vp,
                    ranges.thetaMin, ranges.thetaMax, thetaSteps,
                    ranges.accelMin, ranges.accelMax, accelSteps
                );
                // OP remains at (thetaNom, accelNom); recompute loads in case model changed
                std::tie(WF0, WR0) = CalculateNominalAxleLoads(vp, thetaNom, accelNom);
            }
        }

        RenderAxleLoadPlots(vp, AxleLoadData, thetaNom, accelNom, WF0, WR0);
        ImGui::End();       

        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
    }
    // Cleanup
    ImPlot::DestroyContext();
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
