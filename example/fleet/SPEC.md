# Fleet batch-optimization spec (verified facts — do not re-derive, just use)

This file is the shared source of truth for a set of Julia scripts under
`example/fleet/`. It implements a batch pipeline that, for each aircraft
`.toml` input file in a directory:

1. Loads it with `TASOPT.read_aircraft_model`.
2. Runs a design optimization (NLopt derivative-free, PFEI objective with
   penalty constraints) to converge/size the aircraft.
3. Evaluates a fuel-burn performance envelope: missions from 100 km to the
   design range in 100 km increments (i.e. `100:100:R_design_km`, plus the
   exact design range appended as a final point if `R_design_km` is not
   already a multiple of 100), each flown at 80% of `igWpaymax` (max payload
   *capacity*, not the design payload) using `fly_mission!`.
4. Saves the optimized aircraft (TOML, human-readable) and the fuel-burn
   envelope (JLD2, machine-readable, fast, MATLAB-readable via HDF5) into
   two SEPARATE output locations, as the user explicitly requested aircraft
   output and fuel-burn output to be kept separate.
5. Repeats for ~1000 aircraft. Must be fast to write, fast to re-read later
   (ideally from MATLAB), and robust to individual aircraft failing to
   converge (must not crash the whole batch).

## Verified facts (checked by running actual TASOPT.jl code in this sandbox)

- `TASOPT.read_aircraft_model(path)` returns an `aircraft` struct. Units in
  the returned struct's `parg`/`parm`/`para`/`pare` arrays are always SI,
  regardless of the units used in the TOML.
- `size_aircraft!(ac; iter=35, initwgt=false, Ldebug=false, printiter=true)`
  is the (semver-stable) sizing entry point. Call with `printiter=false` in
  batch scripts to suppress noisy iteration printing. Steady-state runtime
  for `default_wide.toml` is ~0.06-0.08 s per call on this machine (after
  JIT warm-up); full first-call (incl. JIT) was ~8.5 s. NLopt-based design
  optimization calls `size_aircraft!` many times (tens to ~100+ times per
  aircraft depending on optimizer settings), so a single aircraft's
  optimization can take from several seconds to a couple of minutes.
  `size_aircraft!` sets `ac.is_sized .= true` on success; it can also throw
  (non-convergence, infeasible geometry, etc.) — ALWAYS wrap in `try/catch`
  in the objective function and return a large penalty value (e.g. `1e6` or
  `Inf`) on failure, matching the pattern in
  `example/example_opt.jl`/`example/example_gradient_based_opt.jl`.
- `fly_mission!(ac, imission; itermax=35, initializes_engine=true,
  opt_prescribed_cruise_parameter="altitude", printTO=true)` evaluates an
  off-design mission (different `ac.parm[imRange, imission]` and
  `ac.parm[imWpay, imission]`) on an ALREADY-SIZED aircraft, without
  re-sizing structure. It requires `ac.is_sized[1] == true` and mutates the
  `parm`/`para`/`pare` slices for `imission`. Set `printTO=false` in batch
  loops to suppress per-call takeoff-convergence table printing. It can
  throw or converge to an infeasible (negative or over-max) fuel/TOW state
  — always wrap in `try/catch` per off-design point, exactly as
  `TASOPT.PayloadRange` does internally (see
  `src/IO/plotting/output_plots.jl`, function `PayloadRange`, for the
  canonical pattern of duplicating the design mission into a second mission
  slot before calling `fly_mission!` repeatedly with different `imRange`/
  `imWpay`).
- Off-design mission pattern (copy this pattern exactly; do not reinvent):
  ```julia
  parm = cat(ac.parm[:,1], ac.parm[:,1], dims=2)
  pare = cat(ac.pare[:,:,1], ac.pare[:,:,1], dims=3)
  para = cat(ac.para[:,:,1], ac.para[:,:,1], dims=3)
  ac_od = TASOPT.aircraft(ac.name, ac.description, ac.options, ac.parg,
      parm, para, pare, [true], ac.fuselage, ac.fuse_tank, ac.wing,
      ac.htail, ac.vtail, ac.engine, ac.landing_gear)
  # NOTE: ac.engine.heat_exchangers also needs duplicating along mission dim
  # if present and non-empty (cryo/heat-exchanger models); mirror
  # PayloadRange's handling:
  for HX in ac_od.engine.heat_exchangers
      HX.HXgas_mission = cat(HX.HXgas_mission[:,1], HX.HXgas_mission[:,1], dims=2)
  end
  ac_od.parm[imRange, 2] = Range_m          # meters
  ac_od.parm[imWpay, 2]  = Wpay_N           # Newtons
  # reset cruise alt/CL to design values before each call (fly_mission!
  # overwrites one of {altitude, CL} depending on
  # opt_prescribed_cruise_parameter, and warns if the other was already
  # off-design; PayloadRange resets both defensively before every call):
  ac_od.para[iaalt, ipcruise1, 2] = ac_od.para[iaalt, ipcruise1, 1]
  ac_od.para[iaCL,  ipcruise1, 2] = ac_od.para[iaCL,  ipcruise1, 1]
  fly_mission!(ac_od, 2; itermax=35, printTO=false)
  ```
  Only ONE off-design aircraft copy (`ac_od`, mission slot 2) is needed for
  the whole envelope sweep of one aircraft — reuse it across all range
  points, resetting `parm[imRange,2]`/`parm[imWpay,2]`/cruise alt/CL each
  time, exactly like the loop body in `PayloadRange`.
- Units, confirmed by direct inspection of `src/IO/read_input.jl` and by
  running the model:
  - `ac.parm[imWpay, imission]` and `ac.parg[igWpaymax]` are **weights in
    Newtons** (N), NOT masses and NOT passenger counts. `igWpaymax` is the
    max structural payload weight capacity (parsed from the TOML's
    `max_payload` field, which can be given as e.g. `"450 pax"` and is
    converted to N using `weight_per_pax` at parse time).
  - `ac.parm[imRange, imission]` and `ac.parg[igRange]` are in **meters**.
  - `ac.parg[igLHVfuel]` is the fuel's lower heating value in **J/kg**
    (fuel-type-dependent: Jet-A ≈ 4.44e7 J/kg, verified numerically).
  - `ac.parm[imWfuel, imission]` is the **total fuel weight loaded** (N),
    INCLUDING reserve fuel: `Wfuel = Wburn * (1 + freserve)` where
    `freserve = ac.parg[igfreserve]` (fraction) and `Wburn` is the actual
    burned fuel weight (N) for that mission.
  - `ac.parm[imPFEI, imission]` is payload-fuel-energy-intensity in
    kJ/kg-km, defined as
    `PFEI = (Wburn/gee) * LHVfuel / (Wpay * Range)`
    (verified by reproducing it numerically from `Wfuel`, `freserve`,
    `LHVfuel`, `Wpay`, `Range` and matching `ac.parm[imPFEI,imission]`
    exactly).
  - `TASOPT.gee` is standard gravity (9.81 m/s^2), used to convert N -> kg.
- **Fuel burn energy in MJ for a mission point** (the quantity this
  pipeline must save) is therefore:
  ```julia
  Wfuel_N   = ac_od.parm[imWfuel, 2]                 # includes reserve
  freserve  = ac.parg[igfreserve]                    # fraction, e.g. 0.07
  Wburn_N   = Wfuel_N / (1 + freserve)                # actual burned fuel
  fuel_burn_MJ = (Wburn_N / TASOPT.gee) * ac.parg[igLHVfuel] / 1e6
  ```
  This was cross-checked against `ac_od.parm[imPFEI,2]` and matches to
  machine precision. Save `fuel_burn_MJ` (burned fuel energy, reserve
  excluded — this is the physically meaningful "fuel burned in flight"
  quantity and matches the PFEI definition already used throughout
  TASOPT). Do NOT instead save total loaded fuel energy (i.e. do not skip
  the `/(1+freserve)` step) unless explicitly asked; note in output
  metadata which convention was used regardless.
- `TASOPT.__TASOPTindices__` is a path to `index.inc`; scripts must
  `include(TASOPT.__TASOPTindices__)` after `using TASOPT` to bring
  `igWpaymax`, `imWpay`, `imRange`, `imWfuel`, `imPFEI`, `igLHVfuel`,
  `igfreserve`, `igRange`, `iaalt`, `iaCL`, `ipcruise1`, etc. into scope as
  plain global `const` Ints. These are explicitly NOT semver-stable
  per-package but are the only practical way to reach these scalars today
  (per `docs/src/misc/public_api.md`); this is consistent with how EVERY
  example script in `example/` already does it.
- `TASOPT.save_aircraft_model(ac, datafile; save_output=false)` is the
  stable, human-readable TOML writer (semver-protected function name).
  Use it to write the optimized aircraft's output TOML.
- `JLD2` is already a declared dependency of this package (see
  `Project.toml`) and is available in the sandbox's package cache — no
  network/Pkg operations needed. It is an HDF5-based format:
  `jldopen(path, "a+") do file; file["key"] = value; end` appends new
  top-level (or nested `"group/key"`) datasets to an existing file without
  rewriting the whole file, and Julia `Vector{Float64}`, `Bool`,
  `Vector{Bool}}`, `String`, and scalars round-trip as plain HDF5
  datasets (verified: plain float64 arrays, scalar float64, scalar/array
  bool, and String all appear as ordinary flat HDF5 datasets/groups when
  inspected with `h5dump`, i.e. NOT as opaque serialized blobs — the file
  IS directly readable from MATLAB's `h5read`/`h5info`, no Julia
  dependency required, PROVIDED we only ever store plain arrays/scalars/
  strings/bools, never custom Julia structs (e.g. never store a whole `ac`
  or `Options` object in this file — only plain numeric/string/bool
  data). CSV/`Tables.jl`/`DelimitedFiles` are also pre-installed and
  suitable for a flat tidy-format companion index if desired, but the
  primary large numeric envelope data should go in JLD2/HDF5 for fast
  read/write at ~1000-aircraft scale.
  **HDF5.jl and MAT.jl are NOT available in this offline sandbox** (no
  network access, and neither package nor its dependencies are present in
  the local package cache — verified by attempting `Pkg.add("HDF5")`
  offline, which fails with "no known versions"). Do not add these as
  dependencies. JLD2 (already a dependency) writing plain arrays/scalars
  is the only viable HDF5-compatible route in this sandbox, and it is
  sufficient because JLD2's on-disk format for plain data types IS
  standard HDF5 (verified with `h5dump` above).
- `ac.wing`, `ac.htail`, `ac.vtail` are typed structs (`Wing`, `Tail`) with
  a custom `Base.getproperty` that forwards unknown field accesses to
  `.layout` (a `WingLayout`) — e.g. `ac.wing.AR` and `ac.wing.layout.AR`
  are equivalent; `ac.wing.span`, `ac.wing.sweep` etc. work directly. This
  matches the pattern already used in `example/example_gradient_based_opt.jl`.
- The three example default TOML files (`example/defaults/default_wide.toml`,
  `default_regional.toml`, `default_input.toml`) are well-commented
  references for TOML schema/units. Any new example fleet-input TOML files
  should be derived from these via full copy + targeted edits (fuel type,
  payload, range, architecture-relevant options), not written from scratch,
  to avoid missing required fields.
- `ac.options.opt_prop_sys_arch` is a `PropSysArch.T` enum
  (`TF`/`TE`/`ConstantTSFC`/`FuelCellWithDuctedFan`); `ac.options.opt_fuel`
  is a `FuelType.T` enum (`JetA`/`LH2`/`CH4`). These reflect what was
  actually parsed from the TOML's `[Options] prop_sys_arch` and
  `[Fuel] fuel_type` fields and are useful metadata to log per-aircraft in
  the batch index (do not need to be *varied* by the optimizer; they are
  architecture choices baked into each input TOML, not continuous design
  variables).
- Directory layout to create (relative to `example/fleet/`):
  ```
  example/fleet/
    SPEC.md                  (this file)
    main_execution.jl        (top-level driver script; written separately)
    lib/
      fleet_io.jl            (manifest discovery + save helpers)
      fleet_optimize.jl       (optimize_aircraft! + fuel-burn envelope eval)
    inputs/                  (*.toml aircraft definitions to batch over)
    outputs/
      aircraft/                (one .toml per optimized aircraft, via
                                save_aircraft_model)
      fuel_burn/               (fuel-burn envelope data, JLD2, one file per
                                aircraft OR one combined file — see the
                                function docstring in fleet_io.jl for the
                                final decision, must support ~1000 aircraft
                                and fast keyed lookup by aircraft name/input
                                filename from Julia AND MATLAB)
      index.csv                (one row per aircraft: name, input file,
                                architecture options, converged?, PFEI,
                                design range, output TOML path, fuel-burn
                                data key/path — the "map from .toml to
                                performance envelope" lookup table the user
                                asked for)
  ```

## Division of labor across parallel sub-agents (each touches disjoint files)

- Sub-agent A writes `lib/fleet_io.jl` only.
- Sub-agent B writes `lib/fleet_optimize.jl` only.
- Sub-agent C writes 2 example input TOMLs into `inputs/` only (copies of
  `default_wide.toml` and `default_regional.toml` with minor renames, for
  smoke-testing `main_execution.jl` before scaling to ~1000 files).
- `main_execution.jl` itself is written by the orchestrating (main) agent,
  not delegated, since it is short and is the integration point that must
  be internally consistent with whatever function signatures A and B
  actually produce.

Each sub-agent MUST re-state, at the top of its output file as a comment
block, the exact function signature(s) it is providing, so the orchestrator
can wire them together without re-reading the full file.

## Mandatory function contracts (implement EXACTLY these signatures)

### `lib/fleet_io.jl` (sub-agent A)

```julia
"""
    discover_input_files(inputs_dir::String) -> Vector{String}

Return sorted absolute paths of all `*.toml` files directly inside
`inputs_dir` (not recursive). Empty vector (with a `@warn`) if none found.
"""
function discover_input_files(inputs_dir::String) end

"""
    save_fleet_aircraft(ac, name::String, aircraft_outdir::String) -> String

Writes the optimized/sized aircraft `ac` to
`joinpath(aircraft_outdir, name * ".toml")` using
`TASOPT.save_aircraft_model(ac, path)`. Creates `aircraft_outdir` with
`mkpath` if needed. Returns the path written.
"""
function save_fleet_aircraft(ac, name::String, aircraft_outdir::String) end

"""
    save_fleet_envelope(name::String, fuel_burn_outdir::String;
        range_km::Vector{Float64}, fuel_burn_MJ::Vector{Float64},
        converged::Vector{Bool}, payload_N::Float64, payload_fraction::Float64,
        design_range_km::Float64, pfei_design::Float64) -> String

Writes ONE JLD2 file per aircraft to
`joinpath(fuel_burn_outdir, name * ".jld2")` (simplest robust scheme at
~1000-aircraft scale: one small file per aircraft avoids any concurrent-
write/append complexity, is trivially parallelizable later, and each file
is independently readable from MATLAB via `h5read`/`h5info` without
touching any other aircraft's data). Store ONLY plain arrays/scalars/
strings/bools as top-level keys (no nested structs), i.e.:
  - "name" (String)
  - "range_km" (Vector{Float64})
  - "fuel_burn_MJ" (Vector{Float64})     -- MJ, parallel to range_km
  - "converged" (Vector{Bool})           -- parallel to range_km
  - "payload_N" (Float64)                -- absolute payload weight used, N
  - "payload_fraction" (Float64)         -- e.g. 0.8
  - "design_range_km" (Float64)
  - "pfei_design" (kJ/kg-km, Float64)    -- design-mission PFEI, for reference
Overwrites any pre-existing file of the same name (use `jldopen(path, "w")`
or equivalent). Creates `fuel_burn_outdir` with `mkpath` if needed. Returns
the path written.

"""
function save_fleet_envelope(name::String, fuel_burn_outdir::String;
    range_km, fuel_burn_MJ, converged, payload_N, payload_fraction,
    design_range_km, pfei_design) end

"""
    append_index_row!(index_csv_path::String, row::Dict)

Appends one row to a tidy CSV index at `index_csv_path` (creates with
header on first call if the file does not yet exist; appends without
re-writing header on subsequent calls — use simple manual `open(path, "a")`
+ manual CSV-escaping/writing of a FIXED, DOCUMENTED column order, do not
rely on CSV.jl's own append/header-diffing, to keep behavior fully
predictable across ~1000 calls from a for loop). Required keys in `row`
(fixed column order, exactly this list):
  "name", "input_toml", "architecture", "fuel_type", "converged",
  "pfei_design_kJ_per_kg_km", "design_range_km", "payload_capacity_N",
  "payload_fraction_used", "output_toml_path", "fuel_burn_jld2_path",
  "opt_return_code", "n_envelope_points", "n_envelope_converged",
  "error_message"
Missing keys => write empty string for that cell. All string fields must
have embedded commas/newlines removed or quoted (reuse the simple
comma/newline-stripping approach already used in
`src/IO/output_csv.jl` — i.e. `replace(str, r"[\\r\\n,]" => " ")` — for
robustness, do NOT attempt full RFC4180 quoting).
"""
function append_index_row!(index_csv_path::String, row::Dict) end
```

Put `using TASOPT, JLD2` at the top of the file (TASOPT is only used for
its already-imported `save_aircraft_model`, `JLD2` for the envelope
writer). Do NOT `include` this file into the TASOPT module or add
`module`/`export` wrappers — it will be `include()`-ed directly by
`main_execution.jl` after `using TASOPT`, as a plain script of function
definitions, exactly like every file in `example/`. Add a short docstring
example at the bottom (in a `#=  ... =#` block, not executed) showing a
call to each of the 4 functions.

### `lib/fleet_optimize.jl` (sub-agent B)

```julia
"""
    optimize_aircraft!(ac; maxeval=150, ftol_rel=1e-5) -> (return_code::Symbol, pfei::Float64)

Runs an NLopt derivative-free (LN_NELDERMEAD) design optimization on `ac`
IN PLACE, minimizing PFEI (`ac.parm[imPFEI,1]`) subject to penalty-based
constraints, following EXACTLY the pattern already in
`example/example_gradient_based_opt.jl` / `example/example_opt.jl` (do not
invent a different optimizer or objective structure; this must stay
consistent with the rest of the codebase's established idiom). Design
variables (14, matching `example/example_gradient_based_opt.jl`):
  AR, cruise CL, sweep [deg], cruise altitude [m], inboard taper λ,
  outboard taper λ, root t/c, spanbreak t/c, rcls, rclt, Tt4 [K],
  pihc, pif, BPR
Read INITIAL values and bounds directly off `ac`'s current state at call
time (do not hardcode absolute bounds independent of the loaded aircraft;
instead use fixed RELATIVE/absolute bound offsets like
`[0.5*AR0, 1.5*AR0]` clipped to sane absolute engineering limits, so the
same function works across very different aircraft (regional/wide/
different fuels) without per-architecture tuning). If unsure how to bound
a variable relative to its initial value, use the ABSOLUTE bounds already
used in `example/example_gradient_based_opt.jl` as a fallback default
range and simply clip/re-center the initial value into that range if it
falls outside (never let `initial` fall outside `[lower,upper]`, NLopt
will error).
Penalty constraints (reuse the exact 6 checks + coefficients from
`example/example_gradient_based_opt.jl`: max span, min climb gradient, max
Tt3, max fuel volume, max fan diameter, max balanced field length).
Must call `size_aircraft!(ac; iter=50, printiter=false)` inside a
`try/catch` in the objective, returning `1e6` on any exception (matching
existing examples).
`maxeval` and `ftol_rel` are exposed as keyword arguments (do not hardcode
`opt.maxeval`/`opt.ftol_rel`, since ~1000 aircraft at high maxeval will be
too slow — the caller decides the budget).
Returns `(return_code, pfei)` where `return_code` is the `Symbol` returned
by `NLopt.optimize` (e.g. `:SUCCESS`, `:FTOL_REACHED`, `:MAXEVAL_REACHED`,
`:FAILURE`, etc.) and `pfei` is `ac.parm[imPFEI,1]` AFTER leaving `ac` set
to its best-found design (i.e., after optimization, explicitly re-apply
the best `optx` found and re-run `size_aircraft!(ac; printiter=false)`
ONE more time outside the try/catch-guarded objective closure, so `ac`'s
final on-disk state is fully consistent with its own `parg`/`para`/`pare`
values, not left in whatever the LAST objective-function evaluation
happened to be — NLopt does not guarantee the last evaluated point equals
the returned optimum). Wrap this final re-apply+re-size in `try/catch`
too; if it fails, return `(:REOPT_FAILED, Inf)` rather than throwing, so
the batch loop in `main_execution.jl` can skip this aircraft gracefully.

"""
function optimize_aircraft!(ac; maxeval=150, ftol_rel=1e-5) end

"""
    evaluate_fuel_burn_envelope(ac; payload_fraction=0.8, step_km=100.0,
        itermax=35) -> NamedTuple

`ac` MUST already be sized (`ac.is_sized[1] == true`, e.g. immediately
after `optimize_aircraft!` or a plain `size_aircraft!`). Builds the
off-design mission-2 aircraft copy EXACTLY per the `PayloadRange`-derived
pattern documented in `../SPEC.md` (duplicate `parm`/`para`/`pare` design-
mission slice into a 2-mission aircraft, duplicate
`engine.heat_exchangers[i].HXgas_mission` if any exist and are non-empty).
Sweeps `range_km` over `100.0:step_km:design_range_km` (design_range_km =
`ac.parg[igRange]/1e3`) and appends `design_range_km` itself as a final
extra point if it is not already included in that range (i.e. not an
exact multiple of `step_km`, matching floating point safely via
`!isapprox` or a small tolerance) — always evaluate the design range
point once. Payload used at every point is the FIXED absolute value
`payload_fraction * ac.parg[igWpaymax]` (Newtons) — do not vary payload
across points, only range varies. For each range point: set
`ac_od.parm[imRange,2]`, `ac_od.parm[imWpay,2]`, reset
`ac_od.para[iaalt,ipcruise1,2]`/`ac_od.para[iaCL,ipcruise1,2]` to the
design-mission values, call `fly_mission!(ac_od, 2; itermax=itermax,
printTO=false)` inside a `try/catch`. On success, check feasibility
exactly like `PayloadRange` does (`WTO <= WMTO + 1.0` tolerance N,
`Wfuel <= Wfmax + 1.0` tolerance N, both weights `>= 0`); compute
`fuel_burn_MJ` via the formula in `../SPEC.md`
(`Wburn=Wfuel/(1+freserve)`, `*LHVfuel/gee/1e6`) and mark
`converged[i]=true`. On failure/exception/infeasibility, set
`fuel_burn_MJ[i] = NaN`, `converged[i] = false`, and CONTINUE to the next
range point (never abort the whole envelope because one point failed).
Returns a `NamedTuple` with fields `(range_km::Vector{Float64},
fuel_burn_MJ::Vector{Float64}, converged::Vector{Bool}, payload_N::Float64,
design_range_km::Float64)` ready to be splatted into
`save_fleet_envelope`'s keyword arguments.

"""
function evaluate_fuel_burn_envelope(ac; payload_fraction=0.8, step_km=100.0,
    itermax=35) end
```

Put `using TASOPT, NLopt` (plus `include(TASOPT.__TASOPTindices__)`) at the
top. Same rules as A: no `module`/`export`, plain script of function
defs, will be `include()`-ed by `main_execution.jl` after `using TASOPT`
(so do NOT re-run `include(TASOPT.__TASOPTindices__)` if it would clash —
actually DO include it here too, `include` of the same file twice in Julia
is idempotent/harmless for `const` re-definition of an identical value —
just confirm no `const` redefinition ERROR occurs; if worried, wrap the
`include(TASOPT.__TASOPTindices__)` line in this file with
`isdefined(Main, :igAR) || include(TASOPT.__TASOPTindices__)` for safety).
Add a short non-executed example usage docstring at the bottom.

