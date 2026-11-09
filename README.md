# BiBaZu_PoseFindingDropSim

**Reproducible rigid-body drop tests and stable-pose generation for the BiBaZu L-shaped conveyor chute.**

The simulator uses MuJoCo to simulate individual workpieces landing against a conveyor belt and side wall. It estimates pose incidence, distinguishes settled and unsettled outcomes, and assesses how observed poses respond to disturbances. A PyQt6 GUI supports parameter studies and visible trials; the command line supports repeatable batches and assessment of saved results.

The Beckhoff benchmark ran on 7–8 November 2026 and covered **240,000 drops** across 12 workpieces, with 10,000 trials per workpiece in each belt mode.

[User guide](USER_GUIDE.md) · [Windows launchers](WindowsLaunchers/README.md) · [Current GUI profile](configs/gui-current.json) · [Benchmark timing records](https://github.com/match-BiBaZu/BiBaZu_StableComponentenPoses/tree/4df4f1b27f928adca5e5d74377f8212c8f1ae0c9/timing/20261107_10000)

> **Suggested Figure 1 — Experimental arrangement and simulation scene**
>
> Place a photograph of the physical chute beside a MuJoCo screenshot. Identify the workpiece, polyethylene conveyor belt, PTFE wall, release height, and 500 mm observation distance. Label the belt direction and chute coordinate axes. Suggested image: `docs/images/chute-overview.png`.

## What the simulator does

- Loads 39 bundled workpieces or a user-supplied STL/STEP model, with geometry specified in millimetres.
- Derives mass, centre of mass, and inertia from the mesh and a uniform material density. CoACD supplies convex collision bodies for concave geometry.
- Samples a uniform 3D release orientation for each independently seeded trial. Release-height and lateral-position variation can also be configured.
- Runs either a moving conveyor, including configurable speed variation, or a conveyor stationary from release. The GUI preserves moving settings when switching conditions.
- Records every trial's outcome, seed, release inputs, position, orientation, and simulation time. Unsettled, stalled, lost, and invalid outcomes remain in the records when observed.
- Recognises observed orientations using workpiece symmetry and optionally merges them into assessed pose basins. Physical-test labels can be allocated separately, after pose generation.

The model treats each workpiece as rigid. Friction, contact damping, collision geometry, and stochastic contact impulses are modelling assumptions; agreement with a physical pose distribution must be assessed for the relevant material and feeding condition.

## Install and run

Use **Python 3.12** and [uv](https://docs.astral.sh/uv/). From PowerShell:

```powershell
git clone https://github.com/match-BiBaZu/BiBaZu_PoseFindingDropSim.git
Set-Location BiBaZu_PoseFindingDropSim
uv sync --locked --python 3.12 --extra gui --extra dev
uv run --no-sync dashas-drop-sim gui
```

After installation, Windows users can double-click `DropSimulationGUI.cmd`. The GUI includes workpiece selection, release and contact parameters, **Bandbetrieb** (belt mode), **Posen gruppieren** (group poses), frequency tables, and pose previews. Saved user preferences take precedence over the packaged GUI defaults.

Automatic geometry-symmetry detection uses the optional, pinned `bibazu-chute-pose` dependency. With access to its source repository, install it with `uv sync --locked --python 3.12 --extra gui --extra dev --extra symmetry`. Without it, automatic recognition uses identity symmetry and reports `symmetry_available: false`; this changes pose classification. STEP import additionally requires the `step` extra. The grouping engine itself is included in this repository.

```powershell
# Current GUI-equivalent profile: 100 drops, four workers, grouping and assessment
uv run --no-sync dashas-drop-sim run --config configs/gui-current.json

# Inspect one seeded release, or watch a complete series
uv run --no-sync dashas-drop-sim preview --config configs/gui-current.json --trial 0
uv run --no-sync dashas-drop-sim watch --config configs/gui-current.json

# Run the simulator tests
uv run --no-sync python -m pytest
```

JSON paths resolve relative to the configuration file. The current GUI profile uses elliptic cones, belt/wall friction **0.10/0.05**, damping **0.19**, and grouping with stability assessment enabled. Older explicit settings are preserved; omitted parameters in older CLI configurations can use different engine defaults. Use a complete configuration and its manifest when reproducing an experiment.

## Trial protocol

The simulation settings represent the prototype's chute arrangement, PE mesh conveyor, PTFE side wall, and nominal release and transport conditions. The table distinguishes this physical correspondence from the effective contact parameters and numerical choices. Agreement in pose incidence is evaluated separately against the physical observations.

| Setting | Recorded benchmark value | Relation to the physical prototype |
| --- | --- | --- |
| Moving belt | 100 mm/s nominal, ±3 mm/s variation; 0.2 s between speed targets | Nominal feeding speed with prescribed transport-speed fluctuations |
| Stationary belt | 0 mm/s from release, with speed variation disabled | Stationary-chute test condition |
| Chute geometry | 45° transverse angle, 0° longitudinal inclination; 500 mm observation distance | Configured prototype chute arrangement and observation location |
| Release | 100 mm drop height, 0 mm lateral offset; zero configured spread for both | Nominal release arrangement; height and lateral scatter are disabled |
| Material model | Uniform density of 1.15 g/cm³ | Rigid, homogeneous approximation of the Phrozen 8K resin workpieces |
| Belt sliding friction | Coefficient 0.10 | Effective PE mesh belt/workpiece contact; supported by the dry PE/plastic analogue in the [Habasit engineering guide, p. 94](https://rodavigo.net/catalogos/HABASIT/Bandas%20modulares%20HABASITLINK/HABASIT%2004%20Gu%C3%ADa%20Ingenier%C3%ADa%20HabasitLINK.pdf#page=94) |
| Wall sliding friction | Coefficient 0.05 | Effective PTFE wall/workpiece contact selected in the simulation sensitivity studies |
| Contact model | Elliptic friction cones; 5 ms contact time constant; damping ratio 0.19 | Numerical approximation of compliant, dissipative contact |
| Angular friction parameters | 5.0 mm torsional and 0.1 mm rolling friction | Effective resistance to contact rotation, expressed as lengths |
| Integration | 1 ms time step | Numerical resolution |
| Surface excitation | Stochastic `weight_impulse` disturbances for the first 1.0 s; 49.05 mm/s equivalent free-body velocity change and 0.2 s mean event interval | Statistical representation of contact irregularities; mesh topography is not resolved explicitly |
| Moving-belt outcome | Recorded while the belt runs, at the virtual exit for the configured 500 mm chute | End of transport for the reported moving pose incidence |
| Post-exit braking observation | 0.2 s braking ramp followed by 2.0 s with the belt stopped | Separate rest check after the moving outcome is recorded |
| Pose analysis during benchmark | `group_poses: false`, `compute_stability: false`; no member disturbance assessments or catalogue publishing | Analysis choices independent of the prototype hardware |

These are the values in the archived [run plan](https://github.com/match-BiBaZu/BiBaZu_StableComponentenPoses/blob/4df4f1b27f928adca5e5d74377f8212c8f1ae0c9/timing/20261107_10000/run_plan.json). The impulse parameters describe simulated contact excitation, rather than measured surface topography or a measured resin/PTFE friction coefficient.

### Basis for the PE conveyor-belt friction

The physical conveyor has a **polyethylene (PE) mesh belt**, and the workpieces are printed in **Phrozen 8K resin**. A nominal belt coefficient of **0.10** has a published material-pair analogue: Habasit's *HabasitLINK Plastic Modular Belts Engineering Guide*, Media 6031, December 2010, p. 94, gives **0.10 for a dry PE belt against a plastic (PET) product**. The uni-chains *Engineering Manual*, p. 97, independently lists **0.10 for PE against a plastic product**, with its tables described as dynamic friction under clean conditions. These concern belt-to-product friction; belt-to-support friction is a different interface. [Habasit guide, distributor-hosted copy](https://rodavigo.net/catalogos/HABASIT/Bandas%20modulares%20HABASITLINK/HABASIT%2004%20Gu%C3%ADa%20Ingenier%C3%ADa%20HabasitLINK.pdf#page=94) · [uni-chains guide, distributor-hosted copy](https://www.tyma.cz/pdf/uni-chains-modular-belts-engineering-manual.pdf#page=97)

This supports **0.10 as a plausible effective sliding-friction assumption**. It does not establish a measured coefficient for cured Phrozen resin on this particular mesh belt. The modular-belt tables provide an analogy; print finish, curing, belt weave, contamination, and contact load can change the actual value. The value was selected during simulation sensitivity studies, and this literature comparison is a plausibility check rather than independent experimental validation. It provides no direct justification for the separate wall coefficient of 0.05.

The simulator approximates the belt as a contact plane with separate stochastic impulses for surface excitation. Mesh open area does not justify reducing the Coulomb coefficient in proportion to the remaining contact area. For an independently measured value, slide a flat resin coupon over the actual belt at approximately 100 mm/s, prevent rolling, and divide the steady tangential force by the applied normal force. Measure breakaway friction separately; the present contact model uses one sliding-friction parameter for sticking and sliding.

### When is a trial settled?

**Moving belt.** At the virtual exit, the simulator records the outcome with the belt still running. The preceding half-second window checks repeated support at the belt and wall, orientation stability, and transverse motion. Longitudinal transport along the belt is permitted. The moving pose frequencies use this exit outcome. In the Beckhoff benchmark, the simulator then braked the belt and recorded a separate rest outcome. [Example moving-run summary](https://github.com/match-BiBaZu/BiBaZu_StableComponentenPoses/blob/4df4f1b27f928adca5e5d74377f8212c8f1ae0c9/Kk1a/simulations/20261107_10000/100mm_s/summary.json)

**Stationary from release:** the belt remains at zero throughout the drop and the 10 s observation. Exit braking is inactive. The final half-second window applies the position-range and filtered-speed limits to **all three axes**, so continued longitudinal sliding fails the rest check.

Both checks require position ranges no greater than 1 mm in the evaluated axes and filtered translational speed no greater than 1 mm/s. The orientation window permits at most 1° deviation from its final orientation and filtered angular speed of 0.02 rad/s. Contact support must occur in at least 25% of samples for each surface. A settled pose can therefore mean a steady transported orientation in the moving case and a resting orientation in the stationary case; report the condition with every incidence result.

> **Suggested Figure 2 — Settings and observation criteria**
>
> Add a GUI screenshot showing belt mode, speed variation, friction, damping, impulses, and grouping. For a benchmark screenshot, set 10,000 trials, 24 workers, and grouping/assessment off. A small accompanying diagram can distinguish the moving exit measurement from the 10 s stationary observation. Suggested image: `docs/images/benchmark-settings.png`.

## Beckhoff hardware and software

| Component | Benchmark environment |
| --- | --- |
| Industrial controller | Beckhoff C6043 |
| Processor | 13th Gen Intel Core i9-13900E, displayed base frequency 1.80 GHz |
| Memory | 32 GB RAM |
| Operating system | Windows 11 IoT Enterprise LTSC 24H2, 64-bit; supplied system screenshot: build 26100.7840 |
| Recorded CPU availability | 24 logical CPUs; 24 workers per run |
| Python | 3.12.13 |
| Simulator | `dashas-drop-sim` 0.5.0, commit `72b1ba08aa9b317804cd60e1e2ae6e636910f3fd` |
| Physics and collision processing | MuJoCo 3.14.0; CoACD 1.0.14 |
| Numerical and mesh libraries | NumPy 2.5.3; SciPy 1.18.1; trimesh 5.1.0 |
| Symmetry package | bibazu-chute-pose 0.1.0 |

The controller, exact processor designation, RAM, and Windows edition are supplied study-platform specifications. Python, package versions, logical CPU count, and simulator revision are recorded in the [machine timing log](https://github.com/match-BiBaZu/BiBaZu_StableComponentenPoses/blob/4df4f1b27f928adca5e5d74377f8212c8f1ae0c9/timing/20261107_10000/run_timings.json). The interactive application uses PyQt6; the benchmark log does not record a GUI-library version. The benchmark predates the grouping restoration in this branch, so its durations describe the recorded ungrouped build.

## Measured trial-series duration

The following wall-clock durations are **minutes per 10,000-drop series**, measured on the Beckhoff with 24 workers. Static and moving series are separate runs.

| Workpiece | Stationary, 10,000 drops (min) | Moving, 10,000 drops (min) |
| --- | ---: | ---: |
| Df1a | 29.29 | 21.90 |
| Dk1i | 41.33 | 30.77 |
| Dl1a | 23.18 | 17.27 |
| Kf1i | 96.82 | 63.79 |
| Kk1a | 21.42 | 16.87 |
| Kl1i | 25.25 | 19.24 |
| Qf1i | 51.64 | 34.53 |
| Qk1a | 22.00 | 17.32 |
| Ql1i | 61.24 | 41.87 |
| Rf1a | 23.23 | 17.92 |
| Rk1a | 28.73 | 22.04 |
| Rl1a | 22.19 | 16.87 |
| **Total, 120,000 drops per condition** | **446.33** | **320.38** |

Across all **240,000 drops**, the sum of measured simulation calls is **46,002.416 s (12 h 46 min 42 s)**. This corresponds to **5.22 drops per wall-clock second**, or **0.192 s per drop amortised across the 24 workers**. These are batch-throughput measures; a single trial's simulated observation time or individual execution latency is different.

Timing uses `time.perf_counter` around `run_experiment`. It includes mesh/symmetry preparation, worker startup, physics, and raw-result writing. It excludes validation, archive compression, Git commits, and pushes. Moving-series timings include the separate post-exit braking and rest check. Geometry-cache reuse can affect durations. These drop-only timings exclude the additional member assessments required by current grouped execution.

The complete workflow ran from **7 November 2026, 16:25:01 UTC** to **8 November 2026, 12:48:34 UTC**: **20 h 23 min 32 s** end to end. That interval includes preparation, validation, compression, publishing, and a logged interruption/recovery period. Use the simulation-call total when reporting compute duration; use the longer interval when reporting time to complete the whole workflow. Values are derived from the unrounded seconds in the [timing JSON](https://github.com/match-BiBaZu/BiBaZu_StableComponentenPoses/blob/4df4f1b27f928adca5e5d74377f8212c8f1ae0c9/timing/20261107_10000/run_timings.json); the [timing CSV](https://github.com/match-BiBaZu/BiBaZu_StableComponentenPoses/blob/4df4f1b27f928adca5e5d74377f8212c8f1ae0c9/timing/20261107_10000/run_timings.csv) provides per-run values.

The archived [batch audit](https://github.com/match-BiBaZu/BiBaZu_StableComponentenPoses/blob/4df4f1b27f928adca5e5d74377f8212c8f1ae0c9/timing/20261107_10000/batch_validation.json) reports 24 complete runs, 10,000 unique seeded trials per run, matching file checksums and CSV/JSONL records, finite normalised orientations, and complete timing records. The earlier local AMD timing placeholders have been superseded by these Beckhoff measurements.

## Pose recognition, grouping, and stability

Pose generation starts with measured simulator orientations and workpiece symmetry. Quaternions use **xyzw**, mapping the part into the right-handed chute frame: X along the belt, Y away from the wall, and Z away from the belt. The optional `observed_catalog_path` supplies compatible measured orientation anchors. Otherwise, the first settled trial establishes a source pose; all measured reference orientations are retained. Recognition uses a **5° tolerance** and **1° ambiguity margin**.

With `group_poses: true`, the included grouping procedure constructs seated-energy basins using a **0.5 mm equivalent-lift barrier**, **60° whole-group diameter guard**, **0.1° path sampling**, and directed absorption rules. The **1% frequency cutoff is applied after aggregation**. Source provenance and reproducibility fixtures are included in [the grouping restoration record](experiments/grouped_pose_study/restoration_provenance.json).

Complete grouping requires `compute_stability: true`; the GUI enables this visibly. Each source member receives **73 assessments**: one zero-energy observation and twelve directions at each positive lift level of **0.05, 0.1, 0.2, 0.4, 0.5, and 0.8 mm**. Trace-based checks measure whether it remains in its group. Representative selection uses retention area under the curve, retention at the grouping barrier, source incidence, and source ID as tie-breakers.

Display group IDs start at **0**, ordered by descending simulated incidence with deterministic membership ties. Original group identifiers, member orientations, stability ranks, and representative IDs remain available separately. Unsettled and ambiguous outcomes retain explicit statuses. Disturbance assessments are counted separately from the original drop-incidence denominator.

To reuse completed drops and perform their grouping assessments:

```powershell
uv run --no-sync dashas-drop-sim rebuild-groups --run-dir results/SAVED_RUN --output-dir results/NEW_ASSESSMENT
```

This executes additional member assessments without repeating the original releases. It requires the saved run's mesh; use `--mesh` if the file has moved. Checkpoints validate inputs and source hashes before resuming, and original records are preserved under `source/`. The updated bundled Df1a mesh differs from the mesh used by the recorded benchmark, so retain the archived geometry when reassessing those historical runs. Grouped representative exports are local; automatic catalogue publication is disabled for grouped execution.

## Results and reproducibility

| Output | Purpose |
| --- | --- |
| `config.json`, `manifest.json` | Full settings, effective belt mode, geometry hashes, recognition settings, source revision, and environment |
| `trials.csv`, `trials.jsonl` | All recorded drop trials, including non-settled outcomes |
| `frequencies.csv`, `summary.json` | Pose incidence and run-level counts |
| `groups.json`, `group_members.json` | Group membership, assessment completeness, representatives, and display-ID crosswalk |
| `grouped_trials.jsonl`, `group_frequencies.csv` | Additional group classifications and aggregated incidence |
| `group_stability.csv`, `group_stability_summary.csv` | Retention curves and representative ranking |
| `representatives/` | Assessed, eligible representative poses and associated geometry |

Grouped outputs are produced when their analysis is enabled. Percentages use **all completed drop trials**, including unsettled outcomes, rather than only successfully classified poses. Preserve the exact mesh, run configuration, manifest, raw trials, simulator commit, and timing log with any published result. A shared version number alone does not establish identical simulation settings or geometry.

To reproduce the measured benchmark, use [its simulator revision](https://github.com/match-BiBaZu/BiBaZu_PoseFindingDropSim/tree/72b1ba08aa9b317804cd60e1e2ae6e636910f3fd), archived per-workpiece configuration and mesh, and the recorded software versions. The current GUI profile instead runs 100 trials with four workers and full grouped assessment. The separately maintained local comparison plotter and local result directories are excluded from this simulator repository.

> **Suggested Figure 3 — Physical and simulated pose incidence**
>
> Insert an exported comparison chart with the condition, physical and simulated sample sizes, all member-pose thumbnails, and unsettled outcomes visible. State the physical observation/sampling rule and simulator revision in the caption. A second panel showing Kk1a can illustrate sensitivity to contact assumptions. Suggested image: `docs/images/pose-incidence.png`.
