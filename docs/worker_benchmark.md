# Worker process benchmark

On 4 October 2026, the Dk2a benchmark ran on an Intel Core i9-13900E system with 24 cores and 24 logical processors reported. It used the physics settings in `configs/catalog39.json`. The 39-workpiece batch was paused during the benchmark.

Each worker count ran the same 384 drops twice, with the order of worker counts reversed for the second repeat. The geometry was prepared before timing. Elapsed time includes worker startup and shutdown. SHA-256 checks compared the complete sorted trial records, including impulse events.

| Workers | Median time for 384 drops | Trial records |
| ---: | ---: | --- |
| 8 | 34.50 s | Identical |
| 12 | 36.95 s | Identical |
| 16 | 38.80 s | Identical |

All six runs produced 384 complete, numerically valid results. Eight workers were fastest for Dk2a on this machine and were retained for the full batch. The timing applies to this workpiece and machine under the recorded conditions.

In an earlier 96-drop comparison, the 24-worker run failed during initialization. MuJoCo reported `Could not allocate memory`, followed by `BrokenProcessPool`.

Run this command from the repository root to repeat the comparison.

```powershell
uv run --all-extras python -m dashas_drop_sim.benchmark --config configs/catalog39.json --workpiece Dk2a --trials 384 --workers 8 12 16 --repeats 2 --output results/worker_benchmark_long.json
```

The benchmark records failed worker counts and recommends a setting only when results are valid and identical. When timings differ by at most 5%, it prefers the smaller worker count. Apply the recommendation explicitly to the drop-test configuration.
