"""Point-mass longitudinal lap model with a hybrid power unit and a battery.

The car is a single mass moving along the racing line. Cornering is handled by a
speed limit from the line's curvature; everything else (drag, rolling resistance,
engine and MGU-K power, braking, regen) is integrated along distance.

Units are SI throughout (m, s, kg, W, J) unless a name says otherwise.
"""
from dataclasses import dataclass, replace
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
G = 9.81
RHO = 1.20  # air density, kg/m^3
DS = 5.0  # integration step along the lap, m


@dataclass(frozen=True)
class Car:
    """All parameters are approximations from public reporting, not any team's data (see README)."""

    mass: float = 798.0  # kg, 2026 min 768 kg incl. driver + ~30 kg race fuel
    # mu and cda come from calibrate() on the real 2025 lap (a test checks they still match).
    # The 2026 car keeps the same chassis so the comparison isolates the power-unit change.
    cda: float = 1.20  # drag area Cd*A, m^2
    cla: float = 3.5  # downforce area Cl*A, m^2 (assumed low-downforce Monza level, not fitted)
    mu: float = 1.80  # tyre friction coefficient
    crr: float = 0.012  # rolling resistance coefficient
    p_ice: float = 400e3  # combustion engine power, W
    p_k_max: float = 350e3  # MGU-K deploy limit, W
    p_harvest_max: float = 350e3  # MGU-K harvest limit, W
    taper: bool = True  # 2026-style MGU-K deploy taper at high speed
    e_window: float = 4.0e6  # usable battery energy window, J (assumption, see README)
    harvest_per_lap: float = 8.5e6  # max energy recovered per lap, J
    eta: float = 0.95  # one-way electrical efficiency (motor + inverter + battery)


CAR_2026 = Car()
# 2025-style power unit, used only to validate the vehicle model against the real 2025 lap.
CAR_2025 = replace(CAR_2026, mass=828.0, p_ice=560e3, p_k_max=120e3, p_harvest_max=120e3,
                   taper=False, harvest_per_lap=4.0e6)


def k_deploy_limit(car: Car, v):
    """MGU-K deploy limit at speed v [m/s].

    2026 drafts reportedly cap it at 1850 - 5*v_kmh kW (so the full 350 kW only up to 300 km/h)
    with a 150 kW floor above 340 km/h. Approximation of public reporting only.
    """
    if not car.taper:
        return np.full_like(np.asarray(v, float), car.p_k_max)
    v_kmh = np.asarray(v, float) * 3.6
    return np.clip(1850e3 - 5e3 * v_kmh, 150e3, car.p_k_max)


@dataclass
class Track:
    s: np.ndarray  # distance along lap, m (uniform DS grid)
    v_corner: np.ndarray  # cornering speed limit from curvature, m/s
    v_real: np.ndarray  # real car's speed, m/s
    t_real: float  # real lap time, s
    curvature: np.ndarray  # 1/m


def load_track(car: Car, path: Path = ROOT / "data" / "monza_lap.csv") -> Track:
    t, s, x, y, v = np.loadtxt(path, delimiter=",").T
    grid = np.arange(0.0, s[-1], DS)
    xg, yg = np.interp(grid, s, x), np.interp(grid, s, y)
    # GPS-like positions are noisy and curvature is a 2nd derivative, so smooth hard first.
    xs, ys = _smooth_closed(xg, 7), _smooth_closed(yg, 7)
    heading = np.unwrap(np.arctan2(np.gradient(ys), np.gradient(xs)))
    kappa = np.abs(_smooth_closed(np.gradient(heading) / DS, 5))
    return Track(grid, corner_speed(car, kappa), np.interp(grid, s, v), float(t[-1]), kappa)


def _smooth_closed(a, n):
    """Moving average over n points, wrapping around because the lap is a closed loop."""
    pad = np.concatenate([a[-n:], a, a[:n]])
    return np.convolve(pad, np.ones(n) / n, mode="same")[n:-n]


def corner_speed(car: Car, kappa):
    """Max speed where lateral grip mu*(m*g + downforce) equals m*v^2*kappa."""
    aero = car.mu * 0.5 * RHO * car.cla / car.mass  # grip gained per v^2
    denom = np.maximum(kappa - aero, 1e-6)  # if aero grip wins, the corner is flat out
    return np.minimum(np.sqrt(car.mu * G / denom), 120.0)


def braking_envelope(car: Car, v_corner):
    """Highest speed at each point from which the car can still brake down for every later corner.

    Runs backwards around the lap twice so the last corner also sees the first one.
    """
    n = len(v_corner)
    env = v_corner.copy()
    for k in range(2 * n - 1, -1, -1):
        i, j = k % n, (k + 1) % n
        v = env[j]
        decel = (car.mu * (car.mass * G + 0.5 * RHO * car.cla * v * v)
                 + 0.5 * RHO * car.cda * v * v + car.crr * car.mass * G) / car.mass
        env[i] = min(env[i], np.sqrt(v * v + 2.0 * DS * decel))
    return env


def step(car: Car, v, e, harvested, cmd, v_next_max):
    """Advance one DS step. Works on scalars or numpy arrays (the DP calls it on many states at once).

    v: speed [m/s], e: battery energy [J], harvested: energy recovered so far this lap [J]
    cmd: requested MGU-K power [W], >0 deploy, <0 harvest from the engine
    v_next_max: braking/cornering envelope at the next point
    Returns v_next, e_next, harvested_next, p_k (MGU-K mechanical power actually used), dt, clipped.
    """
    v = np.maximum(v, 1.0)
    dt_est = DS / v  # used only to size the battery limits for this step
    f_res = 0.5 * RHO * car.cda * v * v + car.crr * car.mass * G
    # Battery limits for this step: can't take out more than is left, can't put in more than fits
    # or more than the per-lap harvest allowance.
    p_out_max = e * car.eta / dt_est
    p_in_max = np.minimum(car.e_window - e, np.maximum(car.harvest_per_lap - harvested, 0.0)) / (car.eta * dt_est)
    lim = k_deploy_limit(car, v)
    want = np.clip(cmd, -car.p_harvest_max, lim)
    p_k = np.clip(want, -p_in_max, p_out_max)
    clipped = (want > 0) & (p_k < want - 1.0)  # wanted to deploy but the battery was empty

    p_prop = np.maximum(car.p_ice + p_k, 0.0)
    f_grip = car.mu * (car.mass * G + 0.5 * RHO * car.cla * v * v)
    f_prop = np.minimum(p_prop / v, f_grip)
    v_free = np.sqrt(np.maximum(v * v + 2.0 * DS * (f_prop - f_res) / car.mass, 1.0))
    v_next = np.minimum(v_free, v_next_max)
    limited = v_next < v_free - 1e-9
    v_avg = 0.5 * (v + v_next)
    dt = DS / v_avg

    # When the envelope limits speed, the driver lifts or brakes. Work out the force actually needed.
    f_need = car.mass * (v_next * v_next - v * v) / (2.0 * DS) + f_res
    p_need = f_need * v_avg
    brake_regen = -np.minimum(np.minimum(-p_need, car.p_harvest_max), p_in_max)
    # Part throttle: the MGU-K deploy is cut first if the engine alone is enough.
    part_throttle = np.where(p_k > 0, np.minimum(p_k, np.maximum(p_need - car.p_ice, 0.0)), p_k)
    p_k = np.where(limited, np.where(f_need < 0, brake_regen, part_throttle), p_k)
    clipped = clipped & ~limited

    e_next = e - np.where(p_k > 0, p_k / car.eta, p_k * car.eta) * dt
    harvested = harvested + np.where(p_k < 0, -p_k * car.eta * dt, 0.0)
    # dt_est vs dt mismatch can leave a few joules out of bounds; clamp (checked in tests)
    e_next = np.clip(e_next, 0.0, car.e_window)
    return v_next, e_next, harvested, p_k, dt, clipped


def simulate(car: Car, track: Track, cmd, e0, v0=None):
    """Simulate one flying lap with MGU-K command cmd[i] (W) at each grid point.

    Start speed v0 defaults to the real car's speed at the line, the same for every strategy.
    Returns dict of traces (length n) and totals.
    """
    env = braking_envelope(car, track.v_corner)
    n = len(env)
    v = np.empty(n)
    e = np.empty(n)
    p_k = np.zeros(n)
    dt = np.zeros(n)
    clipped = np.zeros(n, bool)
    v[0] = min(track.v_real[0] if v0 is None else v0, env[0])
    e[0] = e0
    harvested = 0.0
    for i in range(n - 1):
        v[i + 1], e[i + 1], harvested, p_k[i], dt[i], clipped[i] = step(
            car, v[i], e[i], harvested, cmd[i], env[i + 1])
    # last step closes the loop back to the start line (DS from s[-1] to the lap length is ignored)
    return dict(v=v, e=e, p_k=p_k, dt=dt, clipped=clipped, lap_time=float(dt.sum()),
                deployed=float(p_k[p_k > 0] @ dt[p_k > 0]), harvested=float(-p_k[p_k < 0] @ dt[p_k < 0]),
                clipped_m=float(clipped.sum() * DS), env=env)


def calibrate(track_csv: Path = ROOT / "data" / "monza_lap.csv"):
    """Fit grip (mu) and drag (CdA) of the 2025-spec car to the real 2025 lap.

    Grid search, scored on lap time plus speed RMSE. Small grid on purpose: two numbers, cheap.
    """
    best = None
    for mu in np.arange(1.2, 2.21, 0.1):
        for cda in np.arange(0.80, 1.51, 0.05):
            car = replace(CAR_2025, mu=float(mu), cda=float(cda))
            tr = load_track(car, track_csv)
            r = simulate(car, tr, np.full(len(tr.s), car.p_k_max), e0=car.e_window)
            rmse = float(np.sqrt(np.mean((r["v"] - tr.v_real) ** 2)))
            score = abs(r["lap_time"] - tr.t_real) + rmse
            if best is None or score < best[0]:
                best = (score, float(mu), float(cda))
    return best[1], best[2]
