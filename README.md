# quadrotor-autonomy-from-scratch

A small but complete autonomy stack for a simulated quadrotor, written from scratch in **C++17**: a 6-DOF flight simulator with sensor models, an extended Kalman filter for state estimation, a cascaded flight controller, a behavior-tree mission layer with failsafes, and a software-in-the-loop (SIL) test harness that flies seeded scenarios with wind and injected faults and reports pass rates. The aim is to build each layer by hand, test it against physics and against the layer below it, and measure the whole loop the way a flight-test team would.

> **Status:** work in progress. The design and roadmap are below. Results will be added here as each part lands.

## Why

An autonomous aircraft runs a loop: **sense** (estimate where it is from noisy sensors), **decide** (choose what to do next, including when something goes wrong) and **act** (turn that decision into motor commands). Each layer is a well-studied problem on its own. The hard part is the integration: the controller only sees the estimator's output, the estimator only sees sensors that the vehicle's own motion corrupts, and the mission logic has to react to failures that the layers below report late or not at all.

The questions this project answers with its own measurements:

- **How well does the loop track a route** in wind, with realistic sensor noise and biases, when the controller flies on the estimate instead of the true state?
- **Is the estimator honest about its own uncertainty?** An EKF that is overconfident fails silently; the normalised estimation error squared (NEES) test checks it.
- **Do the failsafes fire, and fire correctly?** Low battery, GPS loss and geofence breaches are injected on purpose, and every scenario has a pass/fail criterion.
- **How robust is it?** Monte Carlo runs over wind, noise and fault seeds give a pass rate instead of one good-looking plot.

## Design

| Layer | What it does |
|---|---|
| **Middleware** | Every layer below is a component that publishes and subscribes typed messages on an in-process message bus. A deterministic multi-rate scheduler ticks each component at its own rate in simulated time, so a run is reproducible from its seed. Every message is recorded, and a recorded flight can be replayed into one component (say the EKF or the mission logic) to debug it offline without re-flying |
| **Rigid-body dynamics** | 6-DOF Newton-Euler equations, quaternion attitude, body rates, diagonal inertia, linear drag, ground contact. Integrated with RK4 at 1 kHz, quaternion renormalised each step |
| **Motors and mixer** | X-configuration, thrust k_f·ω² and drag torque k_m·ω² per rotor, first-order motor lag, saturation. The allocation matrix maps collective thrust and body torques to rotor speeds |
| **Environment** | Constant wind plus gusts (Ornstein-Uhlenbeck), gravity, and a battery model that drains with electrical power |
| **Sensors** | IMU (gyro and accelerometer with white noise and random-walk bias), GPS at 10 Hz, barometer, magnetometer; each at its own rate, all from one seeded RNG so every run is reproducible |
| **State estimation** | Error-state EKF over position, velocity, attitude and IMU biases: IMU-driven prediction, GPS, barometer and magnetometer updates, chi-square innovation gating to reject GPS glitches |
| **Control** | Cascaded loops at separate rates: position → velocity (PID) → desired acceleration → thrust vector and attitude setpoint → attitude (quaternion error) → body rates (PID) → mixer. Integrator anti-windup and tilt limits |
| **Guidance** | Waypoint following with speed limits and acceptance radii; routes can come from [motion-planning-from-scratch](https://github.com/supersanchayrx/motion-planning-from-scratch) |
| **Behavior trees** | A hand-written engine: Sequence, Fallback, Parallel, Inverter, Retry and Timeout nodes, Condition and Action leaves returning SUCCESS / FAILURE / RUNNING, and a typed blackboard, ticked at 10 Hz |
| **Mission** | Arm → take off → fly route → land, with failsafes as higher-priority branches: low battery → return to launch, GPS lost → hold and land, geofence breach → return to launch |
| **Scenarios** | JSON files describing the route, wind, sensor faults (GPS dropout, IMU bias step, rotor efficiency loss), battery and the pass criteria |
| **SIL harness** | Runs a scenario headless, faster than real time; a Monte Carlo runner sweeps seeds and reports pass rate, tracking error and estimator error; every run writes a CSV log |
| **Scenario tests** | A pytest suite drives the simulator binary once per scenario and seed, parses the logs and asserts each scenario's pass criteria, so a failing seed becomes a named, reproducible test |
| **Tools** | Python log parsing and plots; a Docker image so the build and the scenario suite run the same on Linux CI as locally; a 3D replay viewer for logs (raylib) |

## Evaluation (planned)

- **Physics sanity.** Free fall, pure rotation and hover equilibrium match closed-form answers; energy is conserved with drag and thrust off.
- **Controller.** Step responses per loop (rise time, overshoot, settling), route-tracking RMSE in still air and in wind, flown first on true state and then on the EKF estimate.
- **Estimator.** Position, velocity and attitude RMSE against truth; NEES within its chi-square bounds over Monte Carlo runs; recovery time after a GPS dropout.
- **Mission and failsafes.** For each injected fault, whether the right branch fired and how long it took.
- **Robustness.** Pass rate over hundreds of seeded runs per scenario, with the failing seeds kept as regression tests.

Method, raw logs and plots will live in `RESULTS.md`.

## Testing

GoogleTest for units, the pytest scenario suite for integration, both in CI (inside the Docker image) on every push.

- Quaternion operations: unit norm preserved, composition and inverse round-trip, rotation of known vectors.
- RK4 integration error shrinks at fourth order as the step size halves.
- The mixer's allocation round-trips: thrust and torques → rotor speeds → thrust and torques.
- Hover: the trimmed rotor speeds hold altitude with zero drift on true state.
- The EKF with zero noise and a perfect model tracks the truth exactly; with noise, NEES stays inside its bounds.
- Behavior-tree semantics: each node type's SUCCESS / FAILURE / RUNNING behavior and halting of running children, checked against hand-written tick sequences.
- Every scenario has machine-checked pass criteria, and a run with the same seed produces a byte-identical log.
- Replaying a recorded flight into the EKF reproduces the estimates recorded live, exactly.
- The message bus delivers in a fixed order per tick, and a component never sees a message stamped later than the current simulated time.

## Roadmap

- [x] Scope and design
- [ ] Message bus, multi-rate scheduler and message recording
- [ ] Math helpers, RK4 and rigid-body dynamics, tested against closed-form cases
- [ ] Motor model, mixer and open-loop hover
- [ ] Cascaded controller on true state: step responses and route tracking in wind
- [ ] Sensor models with noise and bias
- [ ] Error-state EKF, NEES tests, GPS-glitch gating; controller flown on the estimate
- [ ] Behavior-tree engine, mission and failsafes
- [ ] Scenario files, fault injection, Monte Carlo runner, pytest scenario suite, Docker image and CI
- [ ] Log replay into single components, 3D replay viewer, results and write-up

**Later, if the core lands:** minimum-snap trajectories, routes from the motion planner around no-fly zones, several vehicles sharing one airspace, a comparison against PX4 SITL on the same route, and a ROS 2 bridge.

## Planned structure

```
include/quad/     # public headers
src/
  bus/            # typed message bus, multi-rate scheduler, recorder, replay
  math/           # quaternion helpers, RK4
  sim/            # rigid body, motors, environment, battery, sensors
  estimation/     # error-state EKF
  control/        # position, velocity, attitude and rate loops, mixer
  guidance/       # waypoint following
  bt/             # behavior-tree engine and blackboard
  mission/        # mission tree, failsafe conditions and actions
  scenario/       # scenario loader, fault injection, runner, Monte Carlo
apps/
  simulate.cpp    # fly one scenario, write a CSV log
  montecarlo.cpp  # sweep seeds, write a summary
  viewer/         # 3D log replay
scenarios/        # *.json
tests/            # GoogleTest suites
scenario_tests/   # pytest suite: fly each scenario, assert pass criteria on the logs
tools/            # log parsing and plots
Dockerfile        # Linux build + test image used by CI
results/          # RESULTS.md, figures
```

## Requirements

C++17 compiler (GCC, Clang or MSVC), CMake 3.20+. Eigen for linear algebra, GoogleTest, nlohmann/json for scenario files and raylib for the viewer, all fetched by CMake. No robotics, middleware, estimation or control library. Python with pytest and matplotlib for the scenario suite and plots. Docker for the CI image.

## References

- Mahony, Kumar and Corke, *Multirotor Aerial Vehicles: Modeling, Estimation, and Control of Quadrotor* (IEEE Robotics & Automation Magazine, 2012)
- Solà, *Quaternion Kinematics for the Error-State Kalman Filter* (2017): https://arxiv.org/abs/1711.02508
- Lee, Leok and McClamroch, *Geometric Tracking Control of a Quadrotor UAV on SE(3)* (CDC 2010)
- Brescianini, Hehn and D'Andrea, *Nonlinear Quadrocopter Attitude Control* (ETH Zurich, 2013)
- Mellinger and Kumar, *Minimum Snap Trajectory Generation and Control for Quadrotors* (ICRA 2011)
- Colledanchise and Ögren, *Behavior Trees in Robotics and AI: An Introduction* (2018): https://arxiv.org/abs/1709.00084
- Bar-Shalom, Li and Kirubarajan, *Estimation with Applications to Tracking and Navigation* (2001), NEES consistency tests
- PX4 controller diagrams: https://docs.px4.io/main/en/flight_stack/controller_diagrams.html
