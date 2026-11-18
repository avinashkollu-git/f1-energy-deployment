# Bugs and problems hit while building this

Real issues found during development, what caused them, and the fix.

## 1. FastF1 could not load any session

**Symptom:** `session.load()` logged "Failed to load session info data", then
`DataNotLoadedError` when reading laps. Same for 2023, 2024 and 2025.

**Root cause:** FastF1 reads the official live-timing feed. `curl` on
`https://livetiming.formula1.com/static/2024/Index.json` returns **HTTP 401**: the feed now
needs a Formula 1 account (FastF1 logs "No F1 account credentials available"). Not a FastF1
bug and not a network problem.

**Fix:** `fetch_data.py` still tries FastF1 first, then falls back to the public OpenF1 REST
API, which serves the same timing data (car speed and x/y position) without a login.

## 2. Download cache never hit

**Symptom:** every `fetch_data.py` run downloaded the OpenF1 data again and left a new set of
cache files.

**Root cause:** the cache file name used Python's built-in `hash(query)`. String hashing is
randomised per process (`PYTHONHASHSEED`), so the same query gave a different file name on
every run.

**Fix:** use `hashlib.sha1(query)`, which is stable across runs.

## 3. Real lap time was 78.800 s instead of 78.792 s

**Root cause:** the lap was resampled on `np.arange(0, 78.792, 0.05)`, which stops at
78.750 s, and the loader added one step (0.05 s) to get the lap time. Off by 8 ms.

**Fix:** append the exact lap end time to the time grid; the loader reads `t[-1]`.

## 4. Drag area hit the edge of the calibration grid

**Symptom:** first calibration returned CdA = 1.00, the top of the search range, with a model
top speed of 365 km/h against 348 km/h real.

**Root cause:** my starting guess for drag was too low. A quick check: at 348 km/h
(96.7 m/s) with ~680 kW, power = ½ρ·CdA·v³ gives CdA ≈ 1.25 m².

**Fix:** widened the grid to 0.80 to 1.50. The fit settled at 1.20, inside the range, and a
test now checks the hard-coded defaults still equal the calibration result.

## 5. Coast-down test failed: 32.32 m/s vs exact 32.45 m/s

**Root cause:** not a code bug but a wrong test expectation. The integrator takes drag at the
start of each 5 m step (explicit Euler on v²), which has a first-order error; over a 1 km
coast it is 0.4%. My tolerance (0.05 m/s) assumed it was exact.

**Fix:** the test now checks the error is below 0.5% at 5 m steps, and that it shrinks with
smaller steps (0.13 m/s at 5 m, 0.026 m/s at 1 m, about 5x for a 5x smaller step, as
expected for first order). The limitation is listed in the README.

## 6. Rule-based strategy found by an iteration that has two answers

**Symptom:** the first rule-based version barely deployed at all, and thresholds between
290 and 320 km/h all gave the identical lap.

**Root cause:** the simulator takes a command per distance, but the rule "deploy above
X km/h" depends on speed, which depends on the command. I iterated "simulate, set command
from speeds, repeat" starting from no deploy. That loop has more than one fixed point
(deploying raises the speed, which justifies deploying), and starting from zero finds the
lowest one.

**Fix:** drive the lap step by step and decide each command from the speed the car actually
has at that point, like a real controller. No iteration needed, and the recorded command
array reproduces the same lap exactly in `simulate()`.

**What remained true after the fix:** the threshold rule is still a poor strategy for this
car (2.7 s off DP), because the car rarely exceeds ~290 km/h without help, so the rule flips
from "drains the battery" (286 km/h: ends at 1.4 MJ) to "almost never deploys"
(288 km/h: ends at 2.5 MJ).

## 7. Validation error is large in corners (open, documented)

Speed RMSE against the real lap is 19.6 km/h, worst point 67 km/h. Shifting the model trace
along the track does not help (best shift ±0 m), and tuning the curvature smoothing changed
RMSE only between 19.4 and 22.2 km/h. Root cause: position data at ~3.7 Hz gives one point
every ~20 m at speed, too coarse to get chicane curvature right, and the point-mass model has
no traction or gearshift limits out of slow corners. Not fixed; listed under limitations.
