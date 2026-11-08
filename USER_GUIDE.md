# Dashas Drop Sim user guide

Dashas Drop Sim runs reproducible drop tests for individual workpieces in the BiBaZu L-shaped chute. Each drop starts in a uniformly sampled 3D orientation. The simulator records the final pose and can run a separate set of angular-impulse tests to measure pose retention. The results report pose incidence and disturbance response separately.

## Installation and launch

Use Windows or another system that supports MuJoCo, Python 3.12, and [uv](https://docs.astral.sh/uv/). Installation of the optional symmetry dependency requires access to GitHub. Run these commands from the cloned repository.

```powershell
uv sync --locked --python 3.12 --extra gui --extra dev
uv run --no-sync dashas-drop-sim gui
```

These extras install PyQt6 and pytest. Add `--extra symmetry` for the pinned geometry-symmetry package (source-repository access required), and `--extra step` for OpenCascade STEP import. The grouping engine is included in the simulator. On Windows, double-click `DropSimulationGUI.cmd` for a direct launch. The optional installer adds Desktop and Start menu shortcuts under **BiBaZu > BiBaZu Drop Simulation**. The simulator can run alongside other BiBaZu applications and shares CPU and memory resources with them. [Windows launcher and shortcuts](WindowsLaunchers/README.md)

### Defaults and saved profiles

**Als Standard speichern** saves the drop-test settings, advanced simulation parameters, random contact disturbances, and selected paths. You can save settings before choosing a workpiece. The GUI loads them on startup from `%APPDATA%\BiBaZu\DashasDropSim\gui_defaults.json` on Windows. On other platforms it uses `$XDG_CONFIG_HOME/dashas-drop-sim/gui_defaults.json` or `~/.config/dashas-drop-sim/gui_defaults.json`.

A fresh GUI uses the versioned [GUI defaults](src/dashas_drop_sim/assets/gui_defaults.json), including 100 drops, four workers, a 500 mm observation length, contact damping ratio 0.19, one second of initial contact impulses, and a rest check after belt braking. Saved values take precedence. Older CLI configurations retain their engine defaults for omitted settings.

| Profile | Workpiece | Trials | Workers | Catalog export |
| --- | --- | ---: | ---: | --- |
| [gui-current.json](configs/gui-current.json) | Dk1i | 100 | 4 | Off |
| [dk1i-1000.json](configs/dk1i-1000.json) | Dk1i | 1,000 | 16 | Off; historical profile |
| [kf1i-10000.json](configs/kf1i-10000.json) | Kf1i | 10,000 | 16 | Historical publishing profile; adapt before use |

The profiles use bundled STL meshes and paths relative to the profile file. Use `gui-current.json` for the current grouped workflow. The Dk1i and Kf1i historical profiles preserve their original physical settings and do not describe the newest Beckhoff benchmark. To reuse one with current grouping, enable `compute_stability` and clear `catalog_repo`; for drop-only runs, disable `group_poses` and clear `catalog_repo`. Automatic publication is unavailable with the restored grouping. Select an output directory that suits the experiment.

### Run a workpiece

The **Werkstückkatalog** dropdown contains 39 offline STL models, originally imported from source revision `02d3fbcf`. Df1a has since been replaced with the updated component-selector mesh; use each manifest's geometry hash to identify the actual model. Use **Durchsuchen** to choose a custom STL or STEP. In the frequency table, **Bild** opens a 3D sketch and identifies whether it shows an observed final orientation or a catalog reference. [Bundled mesh source](https://github.com/match-BiBaZu/bibazu_geometry_to_pose/tree/02d3fbcfcdc86fcf09ddbe40f39e89a936130447/Werkst%C3%BCcke_STL_grob)

**Simulation starten** starts a headless series. **Falltests sichtbar starten** plays the drops sequentially at about real time in one MuJoCo window. **Letzter Versuch** shows the latest result. Close the viewer or use **Abbrechen** to stop after the completed drops. Optional disturbance tests run headlessly.

```powershell
# Headless series
uv run --no-sync dashas-drop-sim run --config configs/gui-current.json

# One visible trial
uv run --no-sync dashas-drop-sim preview --config config.json --trial 0

# Visible series
uv run --no-sync dashas-drop-sim watch --config config.json
```

### Run a workpiece batch

Under **Mehrere Werkstücke**, use **Mehrere Dateien laden** to select STL or STEP files, or **Aus Katalog laden** to select bundled workpieces. **Aktuelles Werkstück hinzufügen** adds the current model. Each row can optionally specify its own observed pose catalogue; no downstream files are discovered or loaded.

**Werkstück-Batch starten** freezes the shared parameters and processes the queue in order. Headless drops and disturbance tests use separate MuJoCo workers within each workpiece. A failed row is marked and processing continues. **Abbrechen** ends the current series and stops the queue. The batch folder `results/batch_<timestamp>/<number>_<workpiece>/run_*` contains the runs. The parent `batch.json` records settings, statuses, and output folders. Use **Ergebnisse** on a row to open its tables and select its observed pose catalogue.

**Parallele Prozesse** controls the worker count. Each trial keeps its own seed and physics state. One worker runs serially. During a visible series, the setting applies only to disturbance tests. In a Qk1a example with eight drops over 1.3 m, elapsed time was 26.2 s with one worker and 13.2 s with four. The trial, frequency, and disturbance records matched byte for byte.

## Geometry, conveyor, and release

Supply a closed, volume-bearing STL or STEP mesh **in millimetres**. The simulator reads STL directly and triangulates STEP through OpenCascade. It derives mass, centre of mass, and inertia from the mesh and a uniform density. MuJoCo uses convex collision bodies for concave shapes. Geometry quality metrics appear in `manifest.json`.

Paths in a JSON configuration resolve relative to the configuration file. The optional `observed_catalog_path` supplies measured source orientations directly, with xyzw part-to-chute quaternions and explicit pose IDs. Without it, source poses are reconstructed from settled drops and detected workpiece symmetry. No downstream planning files are read or modified. Ambiguous and distant observations retain separate statuses. If the symmetry extra is absent, the manifest reports `symmetry_available: false`.

### Conveyor mode and pose grouping

**Bandbetrieb** selects a moving conveyor (**Laufend**) or a conveyor stationary from release (**Stillstehend ab Abwurf**). Stationary mode holds the belt at 0 mm/s, preserves the saved running speed and variation, and evaluates the drop after 10 s. The final half-second also checks longitudinal motion. Moving mode offers the separate braking and rest check at the virtual exit. Set `belt_mode` to `running` or `stopped` in JSON. Older configurations use the speed-based `auto` mode.

**Posen gruppieren** selects the published observed-pose grouping procedure. It reconstructs symmetry-aware 5° source poses, keeps every measured reference orientation, and uses the first settled trial as its anchor unless a compatible observed catalogue is supplied. It then applies a 0.5 mm seated-lift barrier, a 60° whole-group diameter guard and 0.1° sampled shortest rotations. The 1% frequency cutoff is applied after aggregation.

Grouped execution requires complete member disturbance curves. The GUI visibly enables **Zusätzliche Drehimpuls-Störkurven berechnen** while grouping is selected and includes the barrier level in the assessment. Controls expose `grouping_barrier_mm`, `grouping_max_angle_deg`, `grouping_minimum_frequency_percent` and an optional `observed_catalog_path`. `grouping_enabled` is accepted as a JSON alias for `group_poses`; conflicting explicit values are rejected.

The standard assessment has levels 0, 0.05, 0.1, 0.2, 0.4, 0.5 and 0.8 mm: one zero-kick observation and twelve directions at each positive level, or **73 assessments per source member**. Retention must remain in the source group throughout the settled trace and at the final observation. Representatives use the branch's complete retention-AUC selection. Stability rank, original group ID and frequency-ranked display ID are separate fields.

With `group_poses: false`, each settled drop receives its own row, preview and `trial_NNN` ID. Observed-catalogue matches remain reference metadata. Existing saved GUI preferences retain their physical parameters; grouping visibly requires assessment. Historical drop-only configurations can be run ungrouped or explicitly upgraded to complete assessment. Automatic catalogue publication is disabled for the restored grouping; representative catalogues are exported locally.

To rebuild from an archived run without repeating drops:

```powershell
python -m dashas_drop_sim rebuild-groups --run-dir results/SAVED_RUN --output-dir results/NEW_ASSESSMENT
```

The output must be separate from the source. `source/` preserves the original files byte for byte, and new classifications go to `grouped_trials.jsonl`. Repeating the command resumes outstanding assessment jobs using validated mesh, configuration, observation and simulator-source hashes. A truncated final checkpoint line can be recovered; changed inputs or duplicate job IDs are rejected.


### Release settings

The GUI accepts moving speeds of 0.1–200 mm/s, release heights of 0–200 mm, and lateral offsets of ±100 mm from the bisector between the belt and wall. The default density of 1.15 g/cm³ is within the stated material range of 1.12–1.18 g/cm³. Set the belt and wall friction coefficients independently.

The release height and lateral position each have an optional uniform spread. For example, 100 ± 20 mm gives an 80–120 mm release height, and 0 ± 10 mm gives a −10 to +10 mm offset. Zero spread fixes the value. The full intervals must fit inside the GUI ranges. Separate seeded random streams make the draws reproducible across worker counts and preserve the orientation sequence. Disturbance curves start from an already settled pose.

`actual_drop_height_mm`, `actual_lateral_mm`, and `initial_pos_chute_mm` appear in `trials.csv`, `trials.jsonl`, and `preview.json`. The last field is the initial centre-of-mass position in chute coordinates. The latest-trial tab displays the drawn release values, and the manifest records their configured distribution.

### Example configuration

```json
{
  "mesh_path": "C:/Workpieces/Qk1a.stl",
  "observed_catalog_path": null,
  "output_dir": "C:/DropTests/Results",
  "trials": 100,
  "workers": 4,
  "seed": 42,
  "belt_speed_mm_s": 100.0,
  "belt_speed_variation_mm_s": 3.0,
  "belt_variation_interval_s": 0.2,
  "drop_height_mm": 100.0,
  "drop_height_spread_mm": 20.0,
  "lateral_mm": 0.0,
  "lateral_spread_mm": 10.0,
  "density_g_cm3": 1.15,
  "mu_belt": 0.40,
  "mu_wall": 0.20,
  "alpha_deg": 45.0,
  "beta_deg": 0.0,
  "length_mm": 1300.0,
  "timestep_s": 0.001,
  "roughness_enabled": false,
  "roughness_model": "longitudinal_traction",
  "roughness_wall_height_mm": 0.1,
  "roughness_wall_ramp_mm": 3.0,
  "roughness_wall_spacing_mm": 10.0,
  "roughness_belt_height_mm": 0.0,
  "roughness_belt_ramp_mm": 3.0,
  "roughness_belt_spacing_mm": 10.0,
  "disturbance_levels_mm": [0.0, 0.05, 0.1, 0.2, 0.4, 0.8]
}
```

## Contact disturbances and solver settings

Fresh GUI settings, the `gui-current` profile, and omitted CLI friction values use belt friction **0.10** and wall friction **0.05**. Physical values explicitly supplied in historical files remain unchanged. Grouping uses the restored published procedure and requires complete member assessment. Original trial fields remain in the raw files; new source-pose and basin classifications are exported separately. Display IDs start at 0 in descending simulated frequency, while original membership IDs and stability ranks remain available in the crosswalk.

**Zufällige Kontaktstörungen** enables a statistical model of surface irregularities. The PTFE wall starts with a 0.1 mm maximum effective height, 3 mm effective edge length, and 10 mm mean spacing. The belt starts with zero effective height and has separate settings. The GUI defaults to single impulses with a free-body velocity change of 49.05 mm/s, a mean event interval of 0.2 s, and a one-second excitation window. Older CLI configurations with omitted disturbance fields keep disturbances disabled. When enabled, the selected model applies to drops and the optional disturbance tests.

### Finite force pulses

The `weight_force` model applies a force along chute direction **+X** to a randomly selected loaded contact on the belt or wall. Its magnitude is `roughness_force_fraction × mass × 9.81 m/s²`. The contact point's lever arm produces torque. Starting values are 10% of weight, 50 ms duration, and 200 ms mean pause. The pauses follow an exponential distribution, and pulses run one at a time.

A scheduled pulse starts only if a loaded contact is available. The simulator tracks the application point and can switch to a contact on the other surface if needed. A momentary zero solver load can still count as geometric contact within 0.1 µm. A pulse ends when all chute contacts are lost, with contact checks before each physics step. The external force can act at zero slip and add energy. The height, length, and spacing fields belong to the other roughness models.

```json
{
  "roughness_enabled": true,
  "roughness_model": "weight_force",
  "roughness_force_fraction": 0.1,
  "roughness_force_duration_s": 0.05,
  "roughness_force_interval_s": 0.2,
  "contact_time_constant_s": 0.005,
  "contact_damping_ratio": 0.5,
  "contact_torsional_friction_mm": 5.0,
  "contact_rolling_friction_mm": 0.1
}
```

Add the mesh and output paths to use this fragment. In the viewer, a red point and arrow show the active force pulse, and yellow shows a recently ended pulse. The short yellow display is visual only. The event logs record force, requested and actual duration, actual linear and angular impulses, and stop status. For an uninterrupted pulse, impulse equals force times duration. At 10% of weight for 50 ms the free-body velocity change is 49.05 mm/s. The work reported under `energy_semantics` uses a trapezoidal estimate for the applied force.

### Instantaneous contact impulses

The `weight_impulse` model applies the full impulse **J** at one randomly selected loaded contact. Enter the strength as the equivalent free-body velocity change `J/m` in mm/s. The logged impulse in N·s is `mass × roughness_impulse_delta_v_mm_s / 1000`. The GUI shows the STL mass and corresponding impulse and computes the STEP values after mesh preparation. The default 49.05 mm/s corresponds to `0.1 × g × 50 ms`, and 490.5 mm/s is a stronger diagnostic setting corresponding to `g × 50 ms`. The magnitude sets an impulse, with no dependence on the force-pulse duration or weight fraction fields.

```json
{
  "roughness_enabled": true,
  "roughness_model": "weight_impulse",
  "roughness_impulse_delta_v_mm_s": 49.05,
  "roughness_force_interval_s": 0.2,
  "roughness_active_duration_s": 1.0
}
```

An exponentially timed event needs a loaded geometric contact. The simulator skips events in free flight and applies at most one impulse per step, without catching up missed events. The contact Jacobian and mass matrix convert the impulse to a velocity jump before the physics step. The lever arm changes angular velocity, and the regular MuJoCo contact solver advances the resulting state. The event log records `requested_time_s`, `time_s`, and `scheduling_delay_s`, with rounding of at most one step. `actual_duration_s = 0` denotes the instantaneous jump, and `impulse_ns` records its full magnitude. The energy estimate uses contact-point velocity and effective mass. This external excitation can add energy.

In the Dk1i regression test, a part on its triangular end face reaches the exit with less than 1° of orientation change when undisturbed. One 490.5 mm/s impulse after 1 s tips its long axis by roughly 90°, and it retains the new orientation through the exit. This checks the implementation of the impulse model. [Impulse tipping regression](tests/test_impulse_tipping.py)

### Initial excitation window

`roughness_active_duration_s` limits all random contact disturbances to the start of the trial. The default GUI value is 1 s. The clock starts at drop observation, including free flight, or after reseating for disturbance tests. An active force pulse ends at the window boundary with `pulse_status: window_end`. Later contact cannot restart the window. Belt speed variation continues independently.

Use `0` or **Ganzer Lauf** for excitation throughout the trial. This is also the legacy CLI behavior when the field is absent. The simulator applies excitation only on full integration steps inside the window, so the effective end may be less than one step early. The manifest records the window. Choose enough observation distance after excitation to evaluate the final half-second.

### Contact compliance and angular friction

**Friction cone** selects `elliptic` or `pyramidal` contact friction. Fresh GUI settings and `configs/gui-current.json` use elliptic cones. The choice is saved with GUI defaults and written to each run configuration, manifest, and summary. Set `"friction_cone": "elliptic"` in CLI configurations to reproduce the elliptic-cone reruns. Existing saved settings and older CLI configurations without this field use pyramidal cones. Historical Dk1i and Kf1i profiles retain their recorded cone setting. The cone selection applies to serial, parallel, visible, and disturbance runs.

`contact_damping_ratio` and `contact_time_constant_s` set MuJoCo's `solref`. A damping ratio of 1.2 is overdamped, 1 is critically damped, and values below 1 allow more rebound. A value such as 0.5 is a comparison setting. The time constant must be at least two simulation steps. Older configurations retain 1.2 and 5 ms when these fields are absent. [MuJoCo contact solver parameters](https://mujoco.readthedocs.io/en/3.14.0/modeling.html#solver-parameters)

The torsional and rolling friction parameters are effective lengths, with legacy values of 5 mm and 0.1 mm. They describe resistance to contact rotation and can increase the stability of small workpieces. Set both to zero to retain sliding friction alone. [MuJoCo contact dimensions](https://mujoco.readthedocs.io/en/3.14.0/computation/index.html#condim)

### Longitudinal traction impulses

`longitudinal_traction` approximates local friction changes with short impulses along chute axis **X** at loaded contacts. Wall impulses oppose longitudinal slip. Belt impulses can slow or accelerate a part depending on its motion relative to the belt. The lever arm produces a tipping moment while the MuJoCo solver continues to handle ordinary contact friction.

The statistical model uses `delta_mu = mu × sampled height / edge length` and an effective traversal time `tau = edge length / |X slip|`. The magnitude is capped at `min(delta_mu × normal force × tau, effective mass × |X slip|)`, so the impulse can stop slip at the contact. The normal force comes from the loaded contact. Effective mass accounts for body mass, inertia, and lever arm. Each traversal contributes one integrated impulse. This relation is an uncalibrated approximation of surface irregularities.

Event frequency follows longitudinal slip distance. A zero friction coefficient, absent loaded contact, or zero longitudinal slip yields zero extra impulses. Older saved configurations without `roughness_model` use this model, and the manifest records its version.

### Shallow asperity impacts with friction

`microfacet` generates events at exponentially distributed intervals along relative sliding distance. Doubling sliding distance doubles the expected event count. The simulator selects a loaded solver contact using normal force and local sliding distance. Contact velocity includes translation and rotation, and belt motion is subtracted for belt contacts.

Each effective height is sampled uniformly up to the configured maximum. For shallow slopes, the additional normal impulse is approximately `effective mass × relative sliding speed × height / edge length`. The impact is inelastic and can create rotation when it acts off-centre. Extra tangential friction on the virtual contact plane is capped by `mu × additional normal impulse` and the impulse needed to stop slip. The model uses separate belt and wall coefficients. A moving belt can transfer drive energy, and the impulse stays dissipative in the corresponding surface frame. A relative speed below 0.1 mm/s is treated as numerical noise. Contact travel is sampled about every 5 ms, or once per step for a coarser physics step.

The model computes velocity changes from the contact Jacobian and MuJoCo mass matrix. Independent streams for each trial and surface preserve repeatability in parallel runs. Disturbance curves reuse the random stream for a source pose, with contact and slip driven by the actual trajectory. This model targets small irregularities during sliding. Height divided by edge length must be at most 0.25. Calibrate effective height, length, and spacing against physical transport tests. [MuJoCo Jacobian API](https://mujoco.readthedocs.io/en/3.14.0/APIreference/APIfunctions.html#mj-jac)

The two slip-driven roughness models log the longitudinal impulse, angular impulse, normal force, friction component, effective contact-zone duration, and current belt speed.

### Belt speed variation

`belt_speed_variation_mm_s` sets a symmetric speed range independently of release spread. For example, 100 ± 3 mm/s gives 97–103 mm/s. The full range must lie within 0–200 mm/s. Zero spread fixes belt speed. `belt_variation_interval_s` controls the time between targets, with 0.2 s as the starting estimate.

The nominal speed is the first target. Subsequent targets are uniformly sampled and joined by smooth cosine transitions. The simulator updates MuJoCo's tangential `surfacevel` before each step. Belt contact friction transfers the motion to the part, while the PTFE wall remains stationary. Belt speed variation also works with contact disturbances turned off. [MuJoCo surface velocity](https://mujoco.readthedocs.io/en/3.14.0/XMLreference.html#body-geom-surfacevel)

A separate seed stream keeps the target sequence independent of worker count and timestep. Each drop resets the sequence. A disturbance test starts its sequence after reseating. `belt_speed_control_points` contains time and speed, including the target needed for the final interpolation segment. Configuration and manifest files record the model, and the GUI can save its parameters as defaults. Belt acceleration can generate force and torque even in cases where pure rolling yields zero slip-driven roughness impulses.

## Legacy pose catalog export

The following documents the retained legacy exporter for archived results. The restored grouping procedure disables automatic publication and writes assessed representatives locally under `representatives/`.

Under **Posenkatalog automatisch aktualisieren**, select the local `BiBaZu_StableComponentenPoses` clone and the directory containing the original STL and STEP pairs. Enable export to create a commit for each completed workpiece. The separate automatic-push checkbox publishes those commits. **Alle 39 hinzufügen** queues all bundled workpieces. For drop tests alone, clear **Zusätzliche Drehimpuls-Störkurven berechnen**.

The [39-workpiece catalog profile](configs/catalog39.json) uses 1,000 drops per workpiece, eight workers, a 500 mm observation length, 100 ± 3 mm/s belt speed, a 100 mm release height, zero lateral offset and release spread, longitudinal wall impulses, and disturbance curves turned off. The Dk2a benchmark found eight workers fastest among the tested counts. [Worker benchmark](docs/worker_benchmark.md)

```powershell
uv run --no-sync dashas-drop-sim catalog-batch --config configs/catalog39.json
```

The CLI and GUI share the simulation and export path. Catalog export reuses complete earlier headless results when the geometry, physics settings, and simulator code match. A visible run always runs again. Each run uses its frozen configuration. The CLI writes `results/catalog39/catalog_batch.json` and keeps resume state under `.git/catalog-progress` in the catalog repository. An interrupted batch resumes with the remaining workpieces.

Each workpiece export includes the original STL and STEP, an illustrated README, equivalent `poses.json` and `poses.yaml` files, `frequencies.csv`, `trials.csv`, configuration, and manifest. The pose list contains observed settled orientations. Its primary percentages and Wilson confidence intervals use all completed drops as the denominator. A second percentage uses settled drops only. Unsettled outcomes retain their statuses. Detailed disturbance events and belt-speed targets remain in the raw run.

When the rest check after braking is enabled, the catalog separately classifies settled rest poses using workpiece symmetry and the same 5° tolerance. See `poses_after_stop.json`, `poses_after_stop.yaml`, `frequencies_after_stop.csv`, and `trials_after_stop.csv`. The trial table includes parts still moving and parts that failed to reach the exit. Rest pose IDs contain `_stop_` and use the separate `pose_registry_after_stop.json`. The primary denominator is all drops, and the output also reports shares among arrivals and among settled rest poses. Re-export can add missing classifications to an existing run using its saved observations.

`pose_registry.json` reserves catalog IDs across runs, including IDs for poses absent from the latest run. Orientations are grouped under detected workpiece symmetry, and an observed pose catalogue can provide an additional reference. The exported catalog can recognise orientations independently.

```python
from dashas_drop_sim.catalog_export import recognize_pose

result = recognize_pose(quaternion_xyzw, catalogue_json)
```

`recognition` contains discrete symmetry quaternions or a directed continuous symmetry axis, along with the angular tolerance and ambiguity margin. Quaternions are in **xyzw** order and rotate from workpiece coordinates to chute coordinates. `transform_chute_from_source_mm` maps points from the original CAD or STL frame into the chute, with the translation accounting for the original centre of mass. Chute **X** points along the belt, **Y** away from the wall, and **Z** away from the belt. The images show observed representatives with the longitudinal position omitted for the close-up.

The exporter verifies a complete result and unchanged geometry before replacing the existing workpiece folder. It serializes exports with a process lock and protects unsaved manual changes. If a network error occurs, the local commit has status `pending`. Use **Abgeschlossenen Lauf erneut exportieren …** or the CLI to retry export or push from the saved result.

```powershell
uv run --no-sync dashas-drop-sim export --run-dir results/YOUR_RUN --catalog-repo ../BiBaZu_StableComponentenPoses
```

Use `--no-push` for a local commit only and `--cad-dir` to select a different original CAD directory. The catalog overview reports the trial count, observation length, and observed poses for each workpiece.

## Results and interpretation

Each series creates a directory named `run_<UTC-timestamp>_<seed>` under `output_dir`.

| File | Contents |
| --- | --- |
| `config.json` | Exact run inputs |
| `manifest.json` | File hashes, software versions, and geometry quality |
| `trials.jsonl` | Trial records written during the run and finalized with pose assignments |
| `trials.csv` | Seeds, final orientations, statuses, and pose assignments |
| `frequencies.csv` | Counts, proportions, and 95% confidence intervals by pose or status |
| `stop_observations.csv` | Optional post-braking observations with moving status, rest pose, and coasting distance |
| `disturbances.csv` | Individual disturbance trials by pose, direction, and strength |
| `roughness_events.csv` and `roughness_events.jsonl` | Contact events with phase, trial, contact point, relative velocity, impulse in N·s, and energy change |
| `stability.csv` | Pose retention by disturbance strength |
| `stability_summary.csv` | Normalized area under the disturbance curve and rankings |
| `summary.json` | Machine-readable run summary |

A single-trial preview writes `config.json`, `manifest.json`, and `preview.json` to its own folder. Frequency results include representative orientations for the GUI's pose images.

The moving settlement check looks for stable support and orientation over an observation window while allowing travel along the belt. Unsettled, stalled, and ambiguous outcomes appear separately. Disturbance tests use twelve repeatable directions per positive strength and one baseline run at 0 mm. Strength is an equivalent lifting height in millimetres, with rotational energy `mass × 9.81 m/s² × lifting height`. The curve measures pose retention at the end and at settled intermediate points. Its normalized area gives a relative ranking, with the user choosing any acceptance threshold.

### Rest check after belt braking

**Am Streckenende anhalten und Ruhe prüfen** is enabled in the default GUI profile. After the regular measurement at the virtual exit, the belt decelerates smoothly to zero over a cosine ramp of 0.2 s by default. The simulator then observes the part for 2 s by default with belt variation and contact disturbances disabled. Both durations are configurable. The part keeps its physical state and can coast on the idealized surfaces extending beyond the measurement line.

**Nach Bandstopp** shows the moving and rest statuses and the measured rest pose. A resting part needs stable contact and orientation over the last 0.5 s, with position range at most 1 mm and filtered speed at most 1 mm/s across the three axes. Continued rolling or sliding is classified as `unsettled`. The check runs for the full observation duration even after the part first settles.

The post-braking measurement is stored under `stop_observation` in `trials.jsonl`, in `stop_observations.csv`, and under `belt_stop_observation` in `summary.json`. Moving frequencies and disturbance rankings use their original moving observations. Set `belt_stop_enabled`, `belt_stop_ramp_s`, and `belt_stop_observation_s` in JSON. Legacy configurations with these fields omitted keep the additional rest check off.


## Physical assumptions and limits

- The virtual chute defaults to 45° transverse inclination and 0° longitudinal inclination. The engine's legacy observation length is 1.3 m, and the current GUI profile uses 500 mm. Pose incidence depends on observation length and the release distribution, so compare runs with the same settings.
- The PE belt and PTFE wall each extend 15 cm from the common corner to the free edge. The idealized contact surfaces keep lateral falls outside the modeled outcomes.
- MuJoCo's tangential `surfacevel` moves the PE belt while the PTFE wall stays stationary. The current friction coefficients of 0.10 for the belt and 0.05 for the wall are simulation assumptions. Use physical sliding and drop tests to calibrate both contacts.
- Workpieces are rigid solids with uniform density. Check sensitivity to convex collision decomposition and timestep for the workpieces and conditions of interest.

In an earlier Qk1a check on a 3 m observation length, runs at 1 ms and 0.5 ms used the same ten seeds. Eight of ten paired drops had the same observed orientation class, all 20 drops settled, and the count for each pose differed by at most one. A larger sample and finer-step comparison are needed to assess convergence of the pose distribution. The observed frequencies describe the specified release orientation distribution, drop point, height, and offset.

## Published grouped assessment outputs

- `original_observations.json`: first-trial or supplied anchors and every measured reference orientation.
- `grouping_geometry.json`: sampled barriers, directed basin memberships and review flags.
- `group_members.json`: all source members, trial IDs, orientations and AUCs.
- `groups.json`: published IDs, frequency-ranked display IDs, stability ranks and assessed representatives.
- `group_disturbances.jsonl` and `assessment_checkpoint.json`: resumable assessment records and input/source checksums.
- `group_stability.csv`, `group_stability_summary.csv` and `group_frequencies.csv`: complete curves, ranking and incidence.
- `grouped_trials.jsonl`: original trial fields plus source-pose classification, published group, display group and matching status.
- `representatives/`: the branch-compatible assessed representative catalogue and its original STL.

The comparison plotter consumes these simulator exports directly. It shows all member orientations and orders boxes by simulated frequency; it does not calculate group membership.
