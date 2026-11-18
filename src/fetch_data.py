"""Download one real qualifying lap and save it as a small CSV.

Lap: 2025 Italian GP (Monza) qualifying, fastest lap of car #1, 78.792 s.
Monza is chosen because it is the most energy-limited track on the calendar:
long full-throttle straights and few heavy braking zones to harvest from.

Source order:
  1. FastF1 (needs F1 live-timing access; since late 2026 unauthenticated access returns HTTP 401)
  2. OpenF1 public REST API (api.openf1.org), same timing feed, no login.
Raw downloads are cached in data/cache (gitignored).

Output: data/monza_lap.csv with columns t [s], s [m], x [m], y [m], v [m/s].
"""
import hashlib
import json
import urllib.request
from datetime import datetime, timedelta
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
CACHE = ROOT / "data" / "cache"
OUT = ROOT / "data" / "monza_lap.csv"
SESSION_KEY, DRIVER = 9908, 1  # OpenF1 keys for 2025 Monza Q, car #1


def try_fastf1():
    try:
        import fastf1

        fastf1.Cache.enable_cache(str(CACHE))
        session = fastf1.get_session(2025, "Monza", "Q")
        session.load(laps=True, telemetry=True, weather=False, messages=False)
        lap = session.laps.pick_drivers(DRIVER).pick_fastest()
        tel = lap.get_telemetry()
        t = tel["Time"].dt.total_seconds().to_numpy()
        return t, tel["X"].to_numpy() / 10.0, tel["Y"].to_numpy() / 10.0, tel["Speed"].to_numpy() / 3.6
    except Exception as exc:  # any failure (auth, network, schema) -> fall back
        print(f"FastF1 unavailable ({type(exc).__name__}: {str(exc)[:80]}), using OpenF1")
        return None


def openf1(endpoint: str, query: str):
    path = CACHE / f"openf1_{endpoint}_{hashlib.sha1(query.encode()).hexdigest()[:10]}.json"
    if not path.exists():
        url = f"https://api.openf1.org/v1/{endpoint}?{query}"
        with urllib.request.urlopen(url, timeout=60) as r:
            path.write_bytes(r.read())
    return json.loads(path.read_text())


def parse_time(s: str) -> datetime:
    return datetime.fromisoformat(s.replace("Z", "+00:00"))


def from_openf1():
    laps = [l for l in openf1("laps", f"session_key={SESSION_KEY}&driver_number={DRIVER}") if l["lap_duration"]]
    lap = min(laps, key=lambda l: l["lap_duration"])
    t0 = parse_time(lap["date_start"])
    t1 = t0 + timedelta(seconds=lap["lap_duration"])
    # 2 s margin either side so interpolation covers the whole lap
    win = (f"session_key={SESSION_KEY}&driver_number={DRIVER}"
           f"&date>={(t0 - timedelta(seconds=2)).isoformat()}&date<={(t1 + timedelta(seconds=2)).isoformat()}")
    win = win.replace("+00:00", "").replace(">", "%3E").replace("<", "%3C")
    car = openf1("car_data", win)
    loc = openf1("location", win)
    tc = np.array([(parse_time(c["date"]) - t0).total_seconds() for c in car])
    vc = np.array([c["speed"] for c in car]) / 3.6
    tl = np.array([(parse_time(p["date"]) - t0).total_seconds() for p in loc])
    xl = np.array([p["x"] for p in loc], float) / 10.0  # timing feed positions are in 1/10 m
    yl = np.array([p["y"] for p in loc], float) / 10.0
    t = np.append(np.arange(0.0, lap["lap_duration"], 0.05), lap["lap_duration"])  # include lap end
    return t, np.interp(t, tl, xl), np.interp(t, tl, yl), np.interp(t, tc, vc), lap["lap_duration"]


def main() -> None:
    CACHE.mkdir(parents=True, exist_ok=True)
    got = try_fastf1()
    if got is not None:
        t, x, y, v = got
        lap_time = t[-1] - t[0]
        src = "FastF1"
    else:
        t, x, y, v, lap_time = from_openf1()
        src = "OpenF1"
    s = np.concatenate([[0.0], np.cumsum(0.5 * (v[1:] + v[:-1]) * np.diff(t))])  # distance from speed
    header = f"2025 Monza Q fastest lap car #{DRIVER}, {lap_time:.3f} s, source {src}\nt,s,x,y,v"
    np.savetxt(OUT, np.column_stack([t - t[0], s, x, y, v]), delimiter=",", header=header, fmt="%.4f")
    path_len = np.sum(np.hypot(np.diff(x), np.diff(y)))
    print(f"wrote {OUT.name}: {len(t)} samples, lap {lap_time:.3f} s, "
          f"distance from speed {s[-1]:.0f} m, from x/y {path_len:.0f} m, vmax {v.max()*3.6:.0f} km/h")


if __name__ == "__main__":
    main()
