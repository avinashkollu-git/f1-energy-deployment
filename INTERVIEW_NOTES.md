# Interview notes

Plain-English notes for explaining this project. Numbers are from `make run` (see README).

## 1. The hybrid power unit

- **ICE (internal combustion engine):** burns fuel, gives roughly constant power at full
  throttle. In this model: 400 kW.
- **MGU-K (motor generator unit, kinetic):** an electric machine on the drivetrain. As a
  motor it adds power (deploy). As a generator it slows the car and charges the battery
  (harvest). Under the 2026 rules it is reported to be 350 kW, close to the engine itself.
- **Energy store (ES, the battery):** small. It can only swing a few megajoules. In this model
  the usable window is 4 MJ (my assumption). 4 MJ at 350 kW lasts **11 seconds**. A Monza lap
  is 80 seconds.
- **Where energy comes from:** braking (free energy that would otherwise become brake heat) and
  "harvesting on throttle" (the engine drives the MGU-K as a generator, costing speed now for
  energy later). The rules cap recovery per lap (8.5 MJ reported).
- **Deploy taper:** reported 2026 drafts limit MGU-K power at high speed (full 350 kW only up to
  ~300 km/h, falling to 150 kW above 340 km/h). So deploying at top speed buys little.
- 2026 removes the MGU-H (the turbo-shaft generator), so all electrical energy must come through
  the MGU-K.

## 2. Why deployment strategy matters

The electrical share is big but the battery is small, so you cannot deploy everywhere. The
question becomes: **where does each joule save the most time?**

- Time saved by extra power is largest where the car is slow and accelerating (out of slow
  corners): the same power gives more force at low speed (F = P/v), and the extra speed is
  carried down the whole following straight.
- At high speed, drag grows with v² (drag power with v³), so extra power mostly turns into
  heat in the air, and the taper limits power anyway.
- Recovering energy where the car is already near top speed costs little time.

Result in this model on Monza: flat-out deploy empties the battery after 1.1 km and spends
3.9 km "clipping" (wanting power and having none), and ends the lap empty. Dynamic programming
harvests at the end of the straights and deploys out of corners: **0.05 s faster than
flat-out and still ends the lap with the energy it started with**. Clipping drops to 355 m.

## 3. Dynamic programming basics

DP solves problems that are a sequence of decisions where each decision changes a "state":

1. **Stages:** the lap split into 116 segments of 50 m.
2. **State:** battery energy (81 bins of 50 kJ).
3. **Decision:** MGU-K command in each segment (-100%, -50%, 0, +50%, +100%).
4. **Stage cost:** time to drive the segment, found by simulating it.
5. **Principle of optimality:** the best way to reach a state at stage k+1 is the best way to
   reach some state at stage k plus one decision. So keep only the best path into each state
   and move forward; at the end pick the best final state that meets the constraint, and walk
   back the stored decisions.
6. **Constraint:** final energy ≥ start energy (charge-sustaining).

Cost: 116 stages × 81 states × 5 actions ≈ 47,000 segment simulations, vectorised with numpy:
0.04 s. Trying every plan would be 5^116, impossible.

Honest caveat: speed is also a state (a car that arrives faster finishes the segment faster),
but I don't grid it. Each energy bin keeps the fastest path and its exact speed. That makes it
an approximation; a finer version would add a speed dimension or refine the result locally.

## 4. Model vs real

- Calibrated two numbers (grip 1.80, drag area 1.20 m²) on the real 2025 lap with a 2025-spec
  power unit.
- Lap time error +0.027 s, but **that is by construction**: fitted and checked on the same lap.
- Speed RMSE 19.6 km/h is the real measure. Straights and top speed (348 vs 348 km/h) match.
  Corners do not: curvature from ~3.7 Hz positions is too coarse for chicanes, and there are no
  tyre, traction or gearshift limits.
- What I would do next: validate on a different lap or track without refitting (out-of-sample),
  use higher-rate position data, add a traction limit at low speed.

## 5. Test and validation thinking

Three layers:

1. **Physics checks with known answers:** coast-down with only drag has an exact solution
   (exponential decay); the integrator matches within 0.4% and the error shrinks as the step
   shrinks (the expected first-order behaviour, so the error is understood, not hidden).
2. **Invariants that must always hold:** battery energy stays inside the window; MGU-K power
   never exceeds the deploy taper or harvest limit; per-lap harvest cap respected; battery
   energy change equals the integrated electrical flow (conservation); speed never exceeds the
   braking envelope.
3. **Cross-checks between independent paths:** C++ vs Python on all three strategies (identical
   to 1e-13 m/s); the DP's predicted lap time equals a full re-simulation of its chosen plan.

Plus: report the error honestly (RMSE, not just the fitted lap time), and record every bug and
its root cause (BUGS.md).

## 6. Likely interview questions

**Q1. Walk me through the project in one minute.**
A point-mass lap model on Monza using a real qualifying lap's data. It has drag, rolling
resistance, engine and MGU-K power limits, braking regen and a battery. I compare three ways
to use the battery: flat-out, a speed rule, and dynamic programming with a charge-sustaining
constraint. DP is 0.05 s faster than flat-out while ending with a full-ish battery instead of
empty. The integrator is also ported to C++ (about 280x faster, identical results), with tests
on physics, limits and conservation.

**Q2. Why a point-mass model and not a full vehicle model?**
The question is about energy over a lap, which is dominated by longitudinal dynamics: power,
drag, braking. A point mass captures that with few parameters I can justify. The cost is
corners, which I treat as a speed cap; that is where most of my speed error is.

**Q3. How do you compute the braking points?**
From each corner's speed limit I integrate backwards: the maximum speed at each point from
which the car can still brake (grip including downforce, plus drag) down to every later
limit. Then the forward pass accelerates with available power and takes the minimum of the two.

**Q4. Why does flat-out deploy lose?**
It spends energy at the start of the lap including at high speed where the taper and drag make
it inefficient, then runs out for 3.9 km. It also finishes empty, so it is not sustainable for
the next lap.

**Q5. Why is the rule-based strategy so bad here?**
"Deploy above X km/h" is one knob. This car only passes ~290 km/h without help on the main
straight, so the rule either drains the battery (X ≤ 286) or hardly deploys (X ≥ 288). It ends
the lap with unused energy and is 2.7 s slower. A better rule would deploy out of slow corners
and harvest at the end of straights, which is what DP discovered.

**Q6. Is your DP result the global optimum?**
No. Speed is a hidden second state that I don't grid, and commands are 5 levels per 50 m. It is
a good feasible plan, and it is checked by re-simulation. A finer grid or a speed dimension
would tighten it.

**Q7. How do you know the C++ port is correct?**
Same equations and step order as Python; a pytest compares lap time, the full speed trace, SoC
trace and clipping flags for all three strategies (agree to 1e-13 m/s). The C++ also has its own
unit tests: coast-down vs analytic, SoC bounds, power limits, conservation, empty-battery case.

**Q8. Your lap time matches within 0.03 s. Is the model accurate?**
Not from that number: I fitted grip and drag to the same lap. The honest metric is speed RMSE,
19.6 km/h, mostly from corners. I'd validate on a different lap without refitting.

**Q9. What would you add next?**
Out-of-sample validation; a tyre/traction model; battery temperature and resistance (SoC-
dependent power limits); fuel energy limits; active aero modes; and a multi-lap race
optimisation where SoC carries over between laps.

**Q10. How does this relate to test and development work?**
It's the same loop as on a test bench or at the track: define what correct behaviour looks
like, check invariants and known answers, compare against measured data, quantify the gap
honestly, and find the root cause of each discrepancy rather than tuning it away.
