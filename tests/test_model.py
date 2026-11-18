"""Physics, battery and strategy checks for the Python model, plus Python vs C++ agreement."""
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from cpp_lapsim import simulate_cpp  # noqa: E402
from model import CAR_2026, DS, Track, calibrate, k_deploy_limit, load_track, simulate  # noqa: E402
from strategies import run_all  # noqa: E402

E0 = 2.0e6


@pytest.fixture(scope="module")
def track():
    return load_track(CAR_2026)


@pytest.fixture(scope="module")
def results(track):
    return run_all(CAR_2026, track, E0)


def straight(n=201, v_limit=200.0):
    s = np.arange(n) * DS
    return Track(s, np.full(n, v_limit), np.full(n, 80.0), 0.0, np.zeros(n))


def battery_flow(car, r):
    """Energy that left the battery according to the MGU-K power trace."""
    pk, dt = r["p_k"], r["dt"]
    return float(np.sum(np.where(pk > 0, pk / car.eta, pk * car.eta) * dt))


def test_coast_down_matches_analytic():
    car = replace(CAR_2026, p_ice=0.0, crr=0.0)
    r = simulate(car, straight(), np.zeros(201), E0)
    exact = 80.0 * np.exp(-1.2 * car.cda / (2 * car.mass) * 1000.0)
    assert abs(r["v"][-1] - exact) / exact < 0.005  # explicit integrator, first-order error


def test_battery_energy_conserved(results):
    for name, (r, _, _) in results.items():
        assert abs((r["e"][0] - r["e"][-1]) - battery_flow(CAR_2026, r)) < 10.0, name  # J


def test_soc_within_window(results):
    for name, (r, _, _) in results.items():
        assert r["e"].min() >= 0.0 and r["e"].max() <= CAR_2026.e_window, name


def test_power_limits_respected(results):
    for name, (r, _, _) in results.items():
        assert np.all(r["p_k"] <= k_deploy_limit(CAR_2026, r["v"]) + 1e-6), name
        assert np.all(r["p_k"] >= -CAR_2026.p_harvest_max - 1e-6), name
        assert r["harvested"] * CAR_2026.eta <= CAR_2026.harvest_per_lap + 1.0, name


def test_speed_never_above_envelope(results):
    for name, (r, _, _) in results.items():
        assert np.all(r["v"][1:] <= r["env"][1:] + 1e-9), name


def test_dp_is_charge_sustaining_and_fastest(results):
    dp = results["dynamic programming"][0]
    rule = results["rule-based"][0]
    assert dp["e"][-1] >= E0 - 1.0
    assert rule["e"][-1] >= E0 - 1.0
    assert dp["lap_time"] < rule["lap_time"]


def test_dp_prediction_matches_resimulation(results):
    r, note, _ = results["dynamic programming"]
    assert f"{r['lap_time']:.3f}" in note  # DP's own time == full simulation of its plan


def test_flat_out_clips(results):
    assert results["flat-out"][0]["clipped_m"] > results["dynamic programming"][0]["clipped_m"]


def test_cpp_matches_python(track, results):
    for name, (_, _, cmd) in results.items():
        py, cpp = simulate(CAR_2026, track, cmd, E0), simulate_cpp(CAR_2026, track, cmd, E0)
        assert abs(py["lap_time"] - cpp["lap_time"]) < 1e-6, name
        assert np.abs(py["v"] - cpp["v"]).max() < 1e-6, name
        assert np.abs(py["e"] - cpp["e"]).max() < 1e-3, name
        assert np.array_equal(py["clipped"], cpp["clipped"]), name


def test_calibration_matches_defaults():
    mu, cda = calibrate()
    assert (mu, cda) == pytest.approx((CAR_2026.mu, CAR_2026.cda))
