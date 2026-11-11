# Evidence-gated correction of reversed steering polarity

Models and core implementation for the paper

> L. Fu and X. Du, "Evidence-gated correction of reversed steering polarity in AGV path tracking under steering saturation, rate limits, and lag," submitted to the *Journal of the Franklin Institute*.

A reversed steering polarity, for example after maintenance, turns the negative feedback of a path-tracking controller into positive feedback. BCN tracks the path with barrier-coordinate layers and a one-bit sign hypothesis. It reverses the hypothesis only when two conditions coincide: a monitored function leaves its envelope, and saturation-invariant evidence formed from the encoder steering angle contradicts the hypothesis.

## Contents

| Folder | File | What it is |
|---|---|---|
| `model/` | `tii_simulate.m` | Closed loop: kinematic single-track AGV in the Frenet frame, unknown polarity b(t), steering saturation, rate limit and first-order lag (Eq. 1). RK4 plant at 2 ms, controller at 20 ms with zero-order hold. |
| | `tii_plant_dyn.m` | Dynamic single-track plant (tyre slip, yaw inertia, friction limit) used for the plant-transfer check. |
| | `tii_scenario.m`, `tii_corridor.m`, `tii_kappa.m`, `tii_speed.m` | The eight benchmark scenarios S1–S8: initial states, corridor with intrusions, curvature, speed, noise, disturbances, actuator mismatch. |
| | `tii_default_params.m` | Plant, simulation and BCN parameters. The values are those reported in the paper, including the tuned monitor setting. |
| `controller/` | `ctrl_bcn.m` | The proposed controller: barrier-coordinate layers (Sections 4.1–4.2) and the evidence-gated polarity monitor (Section 4.3). |
| | `tii_controller.m` | Selects BCN, its ablations (BCN-noEv, -noF2, -noBC, BCN-Nuss), BCN-nominal and BC-oracle. |
| `analysis/` | `lock_angle.m`, `window_min.m`, `worst_recovery.m` | Lock angle and lock time (Remark 2, Theorem 1), the window rule W_min (Section 5.3), and the minimum-time recovery bound (Section 5.4). |
| `examples/` | `demo_wrong_polarity.m` | BCN recovering from a wrong initial polarity in S2, next to the fixed-sign controller. |

## Requirements

MATLAB R2021a or newer. No toolboxes are needed for the code in this repository.

## Quick start

```matlab
cd examples
demo_wrong_polarity
```

Expected output (S2, true polarity −1, seed 1017):

```
BCN:         success 1, max corridor ratio 0.905, reversals 1, first at 0.18 s
BCN-nominal: success 0 (fixed sign +1)
lock angle 27 deg, reached no sooner than 0.34 s from a centred wheel
```

One run:

```matlab
addpath model controller analysis
P  = tii_default_params();
sc = tii_scenario('S3', -1, 1017);            % scenario, true polarity, seed
R  = tii_simulate(sc, tii_controller('BCN'), P, true);   % true: keep the time log
R.success, R.gmax, R.nsw                      % mission success, max corridor ratio, reversals
```

The main study uses seeds 1017:1000:30017, both polarities and scenarios S1–S8. A single run reproduces the paper's values bit for bit (for example S2, b = −1, seeds 1017/2017/3017 give γ_max = 0.90473 / 0.96076 / 0.90058).

## Release plan

During review, this repository contains the plant and scenario models, the proposed controller and monitor, and the analysis functions that the theory section uses. On acceptance, the following will be added: the baselines, the common tuning protocol, the full experiment pipeline, and the script that regenerates every table, figure and number of the paper. Until then, they are available from the corresponding author on request.

## License

MIT, see `LICENSE`.

## Citation

If you use this code, please cite the paper above. The full reference will be added on publication.
