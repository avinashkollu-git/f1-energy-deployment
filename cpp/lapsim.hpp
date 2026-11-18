// C++17 port of model.simulate(): braking envelope + forward lap integration.
// Same equations and step order as src/model.py so results match to rounding.
#pragma once

extern "C" {

// Field order must match the ctypes Structure in src/cpp_lapsim.py.
struct CarParams {
    double mass, cda, cla, mu, crr;
    double p_ice, p_k_max, p_harvest_max;
    int taper;  // 1 = 2026-style MGU-K deploy taper at high speed
    double e_window, harvest_per_lap, eta;
};

// Inputs: n grid points spaced ds apart, cornering speed limit v_corner[n], MGU-K command cmd[n] (W),
// start battery energy e0 (J), start speed v0 (m/s).
// Outputs (caller-allocated, length n): v, e, p_k, dt, clipped. Returns lap time in seconds.
double simulate_lap(const CarParams* car, int n, double ds, const double* v_corner, const double* cmd,
                    double e0, double v0, double* v, double* e, double* p_k, double* dt,
                    unsigned char* clipped);
}
