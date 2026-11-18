"""Three ways to decide where the MGU-K deploys and harvests on a lap.

Each strategy returns cmd[i], the requested MGU-K power at every grid point
(>0 deploy, <0 harvest using the engine). model.simulate() turns it into a lap.

Scenario for all three: a race lap that starts at half charge. A strategy is
"charge-sustaining" if it ends the lap with at least the charge it started with,
so it could repeat the same lap every lap of the race.
"""
import numpy as np

from model import Car, Track, braking_envelope, simulate, step


def flat_out(car: Car, track: Track, e0: float):
    """(a) Deploy maximum power everywhere until the battery is empty. Harvest only under braking."""
    return np.full(len(track.s), car.p_k_max)


def rule_based(car: Car, track: Track, e0: float, v_threshold: float):
    """(b) Deploy full power whenever the current speed is above v_threshold, otherwise none.
    Harvest only under braking.

    The rule depends on speed, so drive the lap step by step and decide each command from the
    speed the car actually has at that point (like a controller would), recording the commands.
    """
    env = braking_envelope(car, track.v_corner)
    cmd = np.zeros(len(env))
    v, e, h = min(track.v_real[0], env[0]), e0, 0.0
    for i in range(len(env) - 1):
        cmd[i] = car.p_k_max if v > v_threshold else 0.0
        v, e, h, *_ = step(car, v, e, h, cmd[i], env[i + 1])
    return cmd


def best_rule_based(car: Car, track: Track, e0: float):
    """Pick the threshold (2 km/h steps) with the fastest lap that is still charge-sustaining."""
    best = None
    for v_kmh in range(150, 351, 2):
        cmd = rule_based(car, track, e0, v_kmh / 3.6)
        r = simulate(car, track, cmd, e0)
        if r["e"][-1] >= e0 - 1.0 and (best is None or r["lap_time"] < best[0]):
            best = (r["lap_time"], v_kmh, cmd)
    if best is None:
        raise RuntimeError("no rule-based threshold is charge-sustaining")
    return best[2], best[1]


def dynamic_programming(car: Car, track: Track, e0: float, seg_points: int = 10, n_bins: int = 81,
                        actions=(-1.0, -0.5, 0.0, 0.5, 1.0)):
    """(c) Choose one MGU-K command per 50 m segment to minimise lap time, ending with SoC >= start.

    Stage = segment, state = battery energy (n_bins bins over the usable window),
    decision = command as a fraction of the MGU-K limit (negative = harvest).
    Stage cost = time to drive the segment, found by simulating it exactly.

    Approximation: speed is also a state, but we don't grid it. Each energy bin keeps only the
    fastest path that reached it, along with that path's exact speed and energy. This can miss
    a path that is slightly slower now but faster later; it is the usual trade for speed.
    """
    env = braking_envelope(car, track.v_corner)
    n = len(env)
    bin_size = car.e_window / (n_bins - 1)
    acts = np.array(actions) * car.p_k_max

    # Arrays indexed by energy bin. inf time = bin not reached yet.
    t = np.full(n_bins, np.inf)
    v = np.zeros(n_bins)
    e = np.zeros(n_bins)
    h = np.zeros(n_bins)  # energy harvested so far this lap (per-lap cap)
    start_bin = int(round(e0 / bin_size))
    t[start_bin], v[start_bin], e[start_bin] = 0.0, min(track.v_real[0], env[0]), e0
    history = []  # per segment: (prev_bin, action) for every bin, to walk back the best path

    for start in range(0, n - 1, seg_points):
        stop = min(start + seg_points, n - 1)
        live = np.flatnonzero(np.isfinite(t))
        # every live bin x every action, all simulated at once
        b0 = np.repeat(live, len(acts))
        a = np.tile(np.arange(len(acts)), len(live))
        vv, ee, hh, tt = v[b0], e[b0], h[b0], t[b0].copy()
        for i in range(start, stop):
            vv, ee, hh, _, dt, _ = step(car, vv, ee, hh, acts[a], env[i + 1])
            tt += dt
        new_bin = np.clip(np.rint(ee / bin_size).astype(int), 0, n_bins - 1)
        # keep the fastest candidate in each bin: sort by time, take first per bin
        order = np.lexsort((tt, new_bin))
        first = order[np.concatenate([[True], np.diff(new_bin[order]) != 0])]
        t = np.full(n_bins, np.inf)
        t[new_bin[first]], v[new_bin[first]] = tt[first], vv[first]
        e[new_bin[first]], h[new_bin[first]] = ee[first], hh[first]
        prev = np.full(n_bins, -1)
        act = np.full(n_bins, -1)
        prev[new_bin[first]], act[new_bin[first]] = b0[first], a[first]
        history.append((start, stop, prev, act))

    ok = np.isfinite(t) & (e >= e0 - 1.0)  # terminal constraint: end with at least the start charge
    if not ok.any():
        raise RuntimeError("DP found no charge-sustaining lap")
    b = int(np.argmin(np.where(ok, t, np.inf)))
    dp_time = float(t[b])
    cmd = np.zeros(n)
    for start, stop, prev, act in reversed(history):
        cmd[start:stop] = acts[act[b]]
        b = prev[b]
    return cmd, dp_time


def run_all(car: Car, track: Track, e0: float):
    """Run the three strategies and return name -> (simulation result, note, command)."""
    out = {}
    cmd = flat_out(car, track, e0)
    out["flat-out"] = (simulate(car, track, cmd, e0), "", cmd)
    cmd, v_th = best_rule_based(car, track, e0)
    out["rule-based"] = (simulate(car, track, cmd, e0), f"deploy above {v_th} km/h", cmd)
    cmd, dp_time = dynamic_programming(car, track, e0)
    out["dynamic programming"] = (simulate(car, track, cmd, e0), f"DP predicted {dp_time:.3f} s", cmd)
    return out
