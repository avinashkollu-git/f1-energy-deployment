// Unit tests for the C++ core. No framework: each check prints and counts failures.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

#include "lapsim.hpp"

static int failures = 0;
#define CHECK(cond, msg)                                    \
    do {                                                    \
        if (!(cond)) {                                      \
            std::printf("FAIL: %s\n", msg);                 \
            ++failures;                                     \
        } else {                                            \
            std::printf("ok:   %s\n", msg);                 \
        }                                                   \
    } while (0)

static CarParams car2026() {
    return CarParams{798, 1.2, 3.5, 1.8, 0.012, 400e3, 350e3, 350e3, 1, 4e6, 8.5e6, 0.95};
}

struct Run {
    std::vector<double> v, e, pk, dt;
    std::vector<unsigned char> clip;
    double t;
};

static Run run(const CarParams& c, const std::vector<double>& vc, double cmd, double e0, double v0, double ds) {
    const int n = static_cast<int>(vc.size());
    Run r{std::vector<double>(n), std::vector<double>(n), std::vector<double>(n), std::vector<double>(n),
          std::vector<unsigned char>(n), 0};
    std::vector<double> cmds(n, cmd);
    r.t = simulate_lap(&c, n, ds, vc.data(), cmds.data(), e0, v0, r.v.data(), r.e.data(), r.pk.data(),
                       r.dt.data(), r.clip.data());
    return r;
}

int main() {
    const double ds = 5.0;

    // 1. Coast-down on a straight with only drag: v(s) = v0 * exp(-rho*CdA/(2m) * s).
    //    The integrator is explicit (forces taken at the start of each step), so it has a
    //    first-order error: small at ds = 5 m and shrinking ~linearly as ds shrinks.
    {
        CarParams c = car2026();
        c.p_ice = 0;
        c.crr = 0;
        const double exact = 80.0 * std::exp(-1.2 * c.cda / (2 * c.mass) * 1000.0);
        double err[2];
        const double steps[2] = {5.0, 1.0};
        for (int k = 0; k < 2; ++k) {
            const int n = static_cast<int>(1000.0 / steps[k]) + 1;
            Run r = run(c, std::vector<double>(n, 200.0), 0.0, 2e6, 80.0, steps[k]);
            err[k] = std::fabs(r.v[n - 1] - exact);
            std::printf("      coast-down ds=%.0f m: v_end %.4f, analytic %.4f m/s\n", steps[k], r.v[n - 1], exact);
        }
        CHECK(err[0] / exact < 0.005, "coast-down within 0.5% of analytic solution at ds=5 m");
        CHECK(err[1] < err[0] / 4, "coast-down error shrinks with step size (first order)");
    }

    // 2. A short track with one slow corner: SoC stays inside the window and power limits hold.
    std::vector<double> vc(600, 200.0);
    for (int i = 280; i < 300; ++i) vc[i] = 25.0;
    {
        CarParams c = car2026();
        Run r = run(c, vc, c.p_k_max, 2e6, 60.0, ds);
        bool soc_ok = true, power_ok = true;
        for (size_t i = 0; i < vc.size(); ++i) {
            soc_ok &= r.e[i] >= 0 && r.e[i] <= c.e_window;
            const double lim = std::clamp(1850e3 - 5e3 * r.v[i] * 3.6, 150e3, c.p_k_max);
            power_ok &= r.pk[i] <= lim + 1e-6 && r.pk[i] >= -c.p_harvest_max - 1e-6;
        }
        CHECK(soc_ok, "state of charge stays within [0, window]");
        CHECK(power_ok, "MGU-K power within deploy taper and harvest limit");
        CHECK(r.v[290] <= 25.0 + 1e-9, "car slows to the corner speed limit");
    }

    // 3. Battery bookkeeping: energy change equals deploy/harvest flows with efficiency.
    {
        CarParams c = car2026();
        Run r = run(c, vc, 100e3, 2e6, 60.0, ds);
        double flow = 0;
        for (size_t i = 0; i + 1 < vc.size(); ++i) flow += (r.pk[i] > 0 ? r.pk[i] / c.eta : r.pk[i] * c.eta) * r.dt[i];
        CHECK(std::fabs((r.e.front() - r.e.back()) - flow) < 1.0, "battery energy is conserved (<1 J)");
    }

    // 4. More deploy power never makes the lap slower.
    {
        CarParams c = car2026();
        Run slow = run(c, vc, 0.0, 2e6, 60.0, ds);
        Run fast = run(c, vc, c.p_k_max, 2e6, 60.0, ds);
        CHECK(fast.t < slow.t, "deploying makes the lap faster");
    }

    // 5. Empty battery: no deploy possible, clipping flagged.
    {
        CarParams c = car2026();
        Run r = run(c, std::vector<double>(100, 200.0), c.p_k_max, 0.0, 60.0, ds);
        CHECK(r.pk[10] == 0.0 && r.clip[10], "empty battery gives zero deploy and a clipping flag");
    }

    std::printf("%s (%d failures)\n", failures ? "FAILED" : "PASSED", failures);
    return failures ? 1 : 0;
}
