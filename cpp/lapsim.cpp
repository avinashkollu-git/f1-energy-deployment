#include "lapsim.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace {
constexpr double G = 9.81;
constexpr double RHO = 1.20;

double k_deploy_limit(const CarParams& c, double v) {
    if (!c.taper) return c.p_k_max;
    return std::clamp(1850e3 - 5e3 * v * 3.6, 150e3, c.p_k_max);
}

std::vector<double> braking_envelope(const CarParams& c, int n, double ds, const double* v_corner) {
    std::vector<double> env(v_corner, v_corner + n);
    // Two passes backwards around the closed lap so the last corner sees the first one.
    for (int k = 2 * n - 1; k >= 0; --k) {
        const int i = k % n, j = (k + 1) % n;
        const double v = env[j];
        const double decel = (c.mu * (c.mass * G + 0.5 * RHO * c.cla * v * v) + 0.5 * RHO * c.cda * v * v +
                              c.crr * c.mass * G) / c.mass;
        env[i] = std::min(env[i], std::sqrt(v * v + 2.0 * ds * decel));
    }
    return env;
}
}  // namespace

double simulate_lap(const CarParams* car, int n, double ds, const double* v_corner, const double* cmd,
                    double e0, double v0, double* v, double* e, double* p_k, double* dt,
                    unsigned char* clipped) {
    const CarParams& c = *car;
    const std::vector<double> env = braking_envelope(c, n, ds, v_corner);
    v[0] = std::min(v0, env[0]);
    e[0] = e0;
    p_k[n - 1] = 0.0;
    dt[n - 1] = 0.0;
    clipped[n - 1] = 0;
    double harvested = 0.0, lap_time = 0.0;

    for (int i = 0; i < n - 1; ++i) {
        const double vi = std::max(v[i], 1.0);
        const double ei = e[i];
        const double dt_est = ds / vi;
        const double f_res = 0.5 * RHO * c.cda * vi * vi + c.crr * c.mass * G;
        const double p_out_max = ei * c.eta / dt_est;
        const double p_in_max =
            std::min(c.e_window - ei, std::max(c.harvest_per_lap - harvested, 0.0)) / (c.eta * dt_est);
        const double want = std::clamp(cmd[i], -c.p_harvest_max, k_deploy_limit(c, vi));
        double pk = std::clamp(want, -p_in_max, p_out_max);
        bool clip = want > 0 && pk < want - 1.0;

        const double p_prop = std::max(c.p_ice + pk, 0.0);
        const double f_grip = c.mu * (c.mass * G + 0.5 * RHO * c.cla * vi * vi);
        const double f_prop = std::min(p_prop / vi, f_grip);
        const double v_free = std::sqrt(std::max(vi * vi + 2.0 * ds * (f_prop - f_res) / c.mass, 1.0));
        const double v_next = std::min(v_free, env[i + 1]);
        const bool limited = v_next < v_free - 1e-9;
        const double v_avg = 0.5 * (vi + v_next);
        const double step_dt = ds / v_avg;

        if (limited) {
            const double f_need = c.mass * (v_next * v_next - vi * vi) / (2.0 * ds) + f_res;
            const double p_need = f_need * v_avg;
            if (f_need < 0) {
                pk = -std::min(std::min(-p_need, c.p_harvest_max), p_in_max);  // braking regen
            } else if (pk > 0) {
                pk = std::min(pk, std::max(p_need - c.p_ice, 0.0));  // part throttle cuts MGU-K first
            }
            clip = false;
        }

        double e_next = ei - (pk > 0 ? pk / c.eta : pk * c.eta) * step_dt;
        if (pk < 0) harvested += -pk * c.eta * step_dt;
        e_next = std::clamp(e_next, 0.0, c.e_window);

        v[i + 1] = v_next;
        e[i + 1] = e_next;
        p_k[i] = pk;
        dt[i] = step_dt;
        clipped[i] = clip;
        lap_time += step_dt;
    }
    return lap_time;
}
