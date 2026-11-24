# TASOPT fleet batch pipeline

Batch-optimizes a directory of aircraft `.toml` input files with TASOPT.jl,
sizes/converges each one, evaluates a fuel-burn performance envelope, and
saves results in a form designed for fast re-reading later (including from
MATLAB, without needing Julia).

## Usage

```bash
cd /path/to/TASOPT.jl
julia --project=. example/fleet/main_execution.jl
```

Put your `.toml` aircraft definitions in `example/fleet/inputs/` (two
example files are provided: `wide_jetA_baseline.toml`,
`regional_jetA_baseline.toml` — direct copies of
`example/defaults/default_wide.toml`/`default_regional.toml`, renamed).
Add more `.toml` files there (up to ~1000) to scale up.

**Validated baseline:** `wide_jetA_baseline.toml` is the reference case
that has been fully smoke-tested end-to-end (optimization converges
cleanly to a feasible, improved design). `regional_jetA_baseline.toml`'s
UNMODIFIED default design point already violates the minimum climb-
gradient constraint before optimization even starts (0.0118 vs. 0.015
required) — the optimizer correctly spends its budget repairing this, so
PFEI may look like it "gets worse" at low `maxeval` before it improves
(needs ~300-400 evals to reach feasibility + improvement for this
particular file). This is a property of that specific input file, not a
pipeline bug — treat it as a known issue to revisit later, not something
to fix now.

### Environment variable overrides

| Variable | Default | Meaning |
|---|---|---|
| `FLEET_INPUTS_DIR` | `example/fleet/inputs` | directory of `*.toml` inputs (non-recursive) |
| `FLEET_OUTPUTS_DIR` | `example/fleet/outputs` | where results are written |
| `FLEET_OPT_MAXEVAL` | `150` | NLopt function-eval budget per aircraft |
| `FLEET_OPT_FTOL_REL` | `1e-5` | NLopt relative objective tolerance |
| `FLEET_ENVELOPE_STEP_KM` | `100.0` | range step for the fuel-burn envelope |
| `FLEET_PAYLOAD_FRACTION` | `0.8` | fraction of `igWpaymax` used for every envelope point |

Example — quick low-budget smoke test:
```bash
FLEET_OPT_MAXEVAL=30 FLEET_OPT_FTOL_REL=1e-3 \
    julia --project=. example/fleet/main_execution.jl
```

## What it does, per aircraft

1. `read_aircraft_model(path)` — load.
2. `optimize_aircraft!(ac; maxeval, ftol_rel)` (`lib/fleet_optimize.jl`) —
   NLopt (`LN_NELDERMEAD`) design optimization over 14 variables (AR,
   cruise CL, sweep, cruise altitude, inboard/outboard taper, root/span
   t/c, rcls, rclt, Tt4, πhc, πf, BPR), minimizing PFEI subject to 6
   penalty constraints (max span, min climb gradient, max Tt3, max fuel
   volume, max fan diameter, max balanced field length) — same
   optimization pattern as `example/example_gradient_based_opt.jl`, just
   refactored into a reusable function. Internally calls `size_aircraft!`
   on every evaluation; wrapped in `try/catch` so infeasible points return
   a large penalty instead of crashing. Bounds are computed per-aircraft
   by widening (never narrowing) a set of reference bounds so the
   aircraft's own starting point is always inside them — this matters
   because different aircraft/fuel/architecture combinations start from
   very different Tt4/πhc/etc. and clipping the initial point into
   hardcoded absolute bounds can produce an unphysical starting design
   (this was caught and fixed during testing — see git history / SPEC.md).
3. `evaluate_fuel_burn_envelope(ac; payload_fraction, step_km)`
   (`lib/fleet_optimize.jl`) — flies off-design missions from 100 km to
   the design range in `step_km` increments (design range always included
   as an extra point if not already a multiple of the step), each at a
   FIXED payload of `payload_fraction * igWpaymax` (Newtons; 80% of max
   structural payload capacity by default, not 80% of the design
   payload). Uses `fly_mission!` on a duplicated design-mission slot,
   following the exact pattern used internally by `TASOPT.PayloadRange`.
   Any individual range point that fails to converge or is infeasible
   (over max fuel volume / over MTOW) is recorded as `NaN`/`converged =
   false` and skipped — it does not abort the rest of the envelope.
4. Saves:
   - optimized aircraft -> `outputs/aircraft/<name>.toml` (via the stable
     `save_aircraft_model`, human-readable)
   - fuel-burn envelope -> `outputs/fuel_burn/<name>.jld2` (one small file
     per aircraft; plain HDF5 under the hood — JLD2's on-disk format for
     ordinary arrays/scalars/strings/bools IS standard HDF5, so this file
     is directly readable from MATLAB via `h5read`/`h5info`, or from
     Python via `h5py`, with NO Julia dependency; verified in testing).
     Fields: `range_km`, `fuel_burn_MJ`, `converged`, `payload_N`,
     `payload_fraction`, `design_range_km`, `pfei_design`, `name`.
   - one row appended to `outputs/index.csv` (the aircraft-name ->
     output-file-path lookup table; fixed column order, see
     `lib/fleet_io.jl` docstring for `append_index_row!`).

`fuel_burn_MJ` is the **burned fuel energy** (reserve fuel excluded),
computed as `Wburn = Wfuel/(1+fuel_reserve_fraction)`,
`fuel_burn_MJ = (Wburn/g) * LHVfuel / 1e6`. This matches the convention
already used for `PFEI` throughout TASOPT (verified numerically against
`ac.parm[imPFEI,·]` during development).

## Robustness

Every aircraft is processed inside a `try/catch` in `main_execution.jl` —
malformed input TOML, non-converging optimization, or any exception during
sizing/envelope evaluation is caught, logged to stdout and to
`index.csv` (with `converged=false` and an `error_message`), and the loop
continues to the next aircraft. One bad file cannot halt a ~1000-aircraft
batch run.

## Files

```
example/fleet/
  README.md              this file
  SPEC.md                detailed design spec / verified facts (units,
                          formulas, function contracts) used while building
                          this pipeline — read this if extending/debugging
  main_execution.jl       driver: for-loop over inputs/, orchestrates the
                          steps above
  lib/
    fleet_io.jl            discover_input_files, save_fleet_aircraft,
                            save_fleet_envelope, append_index_row!
    fleet_optimize.jl       optimize_aircraft!, evaluate_fuel_burn_envelope
  inputs/                 put your fleet's *.toml files here
  outputs/
    aircraft/*.toml         one optimized aircraft output file each
    fuel_burn/*.jld2        one fuel-burn envelope file each
    index.csv               aircraft -> output-file lookup table
```
