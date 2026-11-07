# Dashas Drop Sim

Reproducible workpiece drop simulation for the BiBaZu L-shaped chute. Dashas Drop Sim uses MuJoCo to estimate which orientations occur after release and whether a workpiece has settled at the observation point. This README records the reference settings, computing environment, and timing for the CIRP Conference on Manufacturing Systems (CIRP CMS) study materials.

> **Suggested Figure 1 · Chute and coordinate system**
>
> Add a MuJoCo screenshot showing the workpiece, PE conveyor belt, PTFE side wall, and observation line. Label the belt direction and the axes pointing away from the wall and belt. Suggested file `docs/images/chute-overview.png`.

[Detailed user guide](USER_GUIDE.md) · [Windows launcher](WindowsLaunchers/README.md)

The GUI provides a saved **Friction cone** selector. Fresh settings and `configs/gui-current.json` use elliptic cones. The paired reference batch documented below used pyramidal cones at the pinned revision. Use `"friction_cone": "elliptic"` in a current CLI configuration for elliptic-cone experiments.

The current defaults use **belt friction 0.10**, **wall friction 0.05**, and **pose grouping enabled**. Explicit values in historical configurations and saved GUI preferences remain authoritative. The historical study table below records its original settings. Grouped runs retain native `pose_key` values and add frequency-ranked `group_id` values starting at 0, with deterministic key ordering for ties. `groups.json` lists every member trial; grouping only processes recorded outcomes and does not change the physics.

## Simulation protocol

Each trial drops a single rigid workpiece in a uniformly sampled 3D orientation. The model derives mass, center of mass, and inertia from a closed mesh and a uniform material density. CoACD decomposes concave meshes into convex collision bodies. The paired reference batch covers **39 workpieces × 100 drops × 2 belt conditions**, with the same trial seed and release inputs used in both conditions for each workpiece.

| Setting | Paired reference batch |
| --- | --- |
| Moving conveyor | 100 mm/s nominal speed with ±3 mm/s variation and 0.2 s between speed targets |
| Stationary conveyor | 0 mm/s from release, observed for 10 s |
| Chute and observation length | 45° transverse angle, 0° longitudinal inclination, 500 mm configured length |
| Release | 100 mm nominal drop height and 0 mm lateral offset, both with zero configured spread |
| Material and contact | 1.15 g/cm³ uniform density, belt friction coefficient 0.40, wall friction coefficient 0.20 |
| Solver | 1 ms integration step, 5 ms contact time constant, 0.19 contact damping ratio |
| Angular contact friction | 5.0 mm torsional and 0.1 mm rolling friction parameters |
| Initial contact excitation | Stochastic impulses for the first 1.0 s, with 49.05 mm/s equivalent free-body velocity change and a 0.2 s mean event interval |
| Trial execution | Seed 42, four worker processes per workpiece, up to four workpieces concurrently |
| Analysis options | Pose grouping, braking at the observation line, and disturbance sweeps disabled |

The moving case evaluates settlement at the virtual exit using a half-second window for contact support, orientation, and transverse motion. The stationary case evaluates settlement after 10 s and checks motion along all three axes. The contact coefficients and impulse model are simulation assumptions. Their values should accompany any reported pose incidence.

> **Suggested Figure 2 · Recorded study settings**
>
> Capture the GUI with the workpiece selector, belt mode, release parameters, and contact settings visible. Use the values in the table for the selected workpiece. Suggested file `docs/images/study-settings.png`.

## Computing environment

| Component | Reported Beckhoff study platform |
| --- | --- |
| Industrial PC | Beckhoff C6043 |
| Processor | 13th Gen Intel Core i9-13900E, 1.80 GHz base frequency for the performance cores |
| Memory | 32 GB RAM |
| Operating system | Windows 11 IoT Enterprise LTSC 24H2, 64-bit, build 26100.7840 |

The reference simulator build is **Dashas Drop Sim 0.5.0**, commit `643b0e1`. The local reference environment used Python 3.12.10, MuJoCo 3.14.0, CoACD 1.0.14, NumPy 2.5.3, SciPy 1.18.1, and trimesh 5.1.0. PyQt6 6.11.0 provided the GUI. Each run manifest records the simulator commit, geometry hashes, and dependency versions. [Pinned simulator source](https://github.com/match-BiBaZu/Dashas_drop_sim/tree/643b0e1ac5020602a4846eba96baaacef255a332) · [Beckhoff C6043 specifications](https://www.beckhoff.com/en-us/products/ipc/pcs/c60xx-ultra-compact-industrial-pcs/c6043.html)

## Test duration

| Timing measure | Recorded local reference result |
| --- | --- |
| Moving trial, simulated time from release to outcome | **5.33 s median** across 3,900 drops, with a 5th–95th percentile range of **4.29–6.93 s** |
| Stationary trial, simulated observation time | **10 s** at zero belt speed |
| Moving series, elapsed compute time for 100 drops of one workpiece | **20.5 s median** across 39 series, including mesh preparation where required |
| Full paired batch, elapsed time | **25 min 55 s** for 7,800 drops, from the batch log opening to the final report write |

**Provisional benchmark.** The elapsed times above come from the local Windows rerun on an AMD Ryzen 7 PRO 7840U. The full-batch time reflects overlapping workpiece jobs and includes mesh preparation and report generation. Replace the elapsed-time rows with the measured Beckhoff timings when its run log is available. The simulated observation times describe the local reference settings.

## Reference results

| Belt condition | Completed drops | Settled | Settlement rate |
| --- | ---: | ---: | ---: |
| Moving | 3,900 | 3,465 | 88.8% |
| Stationary from release | 3,900 | 3,680 | 94.4% |

The rates describe the respective settlement checks and observation points. Pose incidence against physical tests is evaluated separately using the published roadmap orientations, workpiece symmetry, and the denominator of all recorded trials in each series.

> **Suggested Figure 3 · Physical and simulated pose incidence**
>
> Export a representative chart from the comparison plotter using **Local 100-drop validation** as the simulator source. Keep the pose thumbnails, condition, and sample sizes visible. A second panel can show a difficult case such as Kk1a and its unsettled fraction. Suggested file `docs/images/pose-incidence.png`.

## Reproduce the paired batch

In PowerShell, clone the repository and select the recorded revision.

```powershell
git clone https://github.com/match-BiBaZu/Dashas_drop_sim.git
Set-Location Dashas_drop_sim
git checkout 643b0e1ac5020602a4846eba96baaacef255a332
uv sync --locked --python 3.12 --all-extras
uv run --no-sync dashas-drop-sim gui
```

On Windows, double-click `DropSimulationGUI.cmd` to launch the GUI after installation.

Run the same paired configuration headlessly from the repository root.

```powershell
uv run --no-sync python experiments/validate_belt_modes.py --output results/belt-validation --scratch results/belt-native --jobs 4
```

Use `run_index.json` to locate each workpiece-condition run and its `config.json`, `manifest.json`, `trials.csv`, and `trials.jsonl`. The batch also writes `workpiece_summary.csv`, `paired_trials.csv`, and `report.md`. For a publication archive, retain these records with the exact simulator revision and the timing log from the benchmark machine.
