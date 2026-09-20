# CONTEXT.md — TASOPT.jl Codebase Orientation

_Last reviewed: 2025-09 (v3.0.0). This file is for future contributors/agents about to make
significant changes (expected: ~half the codebase rewritten in coming weeks). Update this
file whenever the architecture changes materially._

## 1. What this package is

TASOPT.jl is a **multi-disciplinary conceptual/preliminary aircraft design and optimization
tool**, a Julia reimplementation of Mark Drela's FORTRAN `TASOPT`. It models tube-and-wing
transport aircraft (narrowbody/widebody/regional) using:
- 2D viscous-inviscid boundary-layer aero (airfoil + fuselage BL) and a Trefftz-plane induced
  drag solver
- Beam-theory structural sizing of wing/tail/fuselage
- Thermodynamic-cycle turbofan engine performance + weight correlations
- A fixed-point weight-sizing iteration coupled to a full mission fuel-burn simulation

Primary reference: Drela's technical description PDF at
`docs/drela_TASOPT_2p16/tasopt.pdf` (see also `docs/src/misc/dreladocs.md`). Read this before
touching sizing/aero/structures math — most non-obvious symbols and formulas trace back to it.

## 2. Repo layout

```
src/
  TASOPT.jl              # Module entry point; defines size_aircraft!() wrapper; include order matters
  data_structs/           # aircraft, Wing, Fuselage, Tail, Engine, Options, index.inc (global par array indices)
  sizing/                 # _size_aircraft!() — the core weight-sizing fixed-point loop
  mission/                # _mission_iteration!() (fuel burn sim), fly_mission!(), takeoff!(), AircraftDeck, odperformance
  aero/                   # airfoil polars, wing/fuselage drag, induced drag (Trefftz plane), aero.jl aggregator
  structures/             # fuseW! (fuselage weight), wing_weights!, loads, landing gear sizing, cabin sizing
  engine/                 # turbofan/ (tfsize, tfoper, tfweight, tfcalc), ducted_fan/, fuel_cell/, turbomachinery maps
  propsys/                # electric machine models (PMSM, inverter, cable) for hybrid/electric architectures
  balance/                # balance_aircraft! — CG/trim/tail sizing
  atmos/                  # ISA atmosphere model
  cryo_tank/              # LH2/CH4 cryogenic fuselage tank sizing (thermal + structural)
  cost/                   # cost estimation/valuation
  IO/                     # read_input.jl (TOML -> aircraft), save_model.jl, output_csv.jl, plotting/
  utils/                  # constants, units, aircraft_utils, sensitivity.jl (finite-diff gradients)

docs/src/                 # Documenter.jl source; READ THIS FIRST — mirrors math to code
example/                  # Runnable example scripts + default TOML input files (best "how to use" reference)
test/                     # Unit + regression tests; runtests.jl is the entry; CI runs this on Julia 1 (latest) and 1.10 (LTS)
```

## 3. Core data model

Everything hangs off one big mutable struct: **`aircraft`** (`src/data_structs/aircraft.jl`).
It mixes two storage paradigms (actively being migrated from the first to the second):

1. **Legacy flat arrays** — `parg` (geometry, 1D), `parm` (mission, 2D: `[quantity, mission#]`),
   `para` (aero, 3D: `[quantity, mission_point, mission#]`), `pare` (engine, 3D same shape).
   Indexed by **named integer constants** (e.g. `igAR`, `iaCL`, `ieTt4`, `ipcruise1`) defined in
   `src/data_structs/index.inc` and brought into scope via
   `include(TASOPT.__TASOPTindices__)`. This is how most existing example/optimization scripts
   address parameters. **These indices are explicitly NOT semver-stable and are a named
   refactor target** (see `docs/src/misc/public_api.md`).
2. **Structured objects** — `Wing`, `Fuselage`, `Tail` (htail/vtail), `Engine`,
   `LandingGear`, `Options`, `fuselage_tank`. These are the "modern" typed replacements and
   the direction of travel. New code should prefer extending these over adding new `parg`/`para`
   indices.

Mission "points" (`ip...` indices, e.g. `ipcruise1`, `ipclimbn`, `iprotate`) index specific
flight-profile stations (takeoff, climb steps, cruise start/end, descent, etc.) inside the
`para`/`pare` arrays. `imission` (dimension 2 of `parm`/`para`/`pare`) selects which mission
(1 = design mission; ≥2 = off-design/alternate missions).

## 4. Core execution flow (what happens when you call `size_aircraft!`)

`size_aircraft!(ac)` (thin wrapper, exported, semver-stable) →
`_size_aircraft!(ac)` (`src/sizing/size_aircraft.jl`, ~930 lines, NOT part of stable API):

1. Set atmosphere at design points (`atmos()`), compute fuselage BL drag (`fuselage_drag!`).
2. Initialize weight guesses (`initialize_sizing_loop!`) unless resuming (`initwgt=true`).
3. **Fixed-point loop** (`itermax`, default 35) that on each pass:
   - Sizes fuselage weight (`fusew!`)
   - Recomputes wing geometry (`set_wing_geometry!`) and pitching moment (`wing_CM`)
   - Sizes htail/vtail (`tail_loading!`)
   - Sizes wing/tail structural weight (`wing_weights!`)
   - Sizes fuselage cryo/insulated fuel tank if applicable (`tanksize!`, `update_fuse!`)
   - Trims aircraft (`balance_aircraft!`) — can move wing, resize htail, or adjust trim CL
   - Computes total drag (`aircraft_drag!`)
   - Sizes engine at cruise design point via `engine.enginecalc!()` (currently wraps the
     turbofan model through `tfwrap!` → `tfcalc!`)
   - Runs the full mission fuel-burn simulation (`_mission_iteration!`, mission module) —
     usually the single biggest cost in a sizing run
   - Updates MTOW and checks convergence tolerance (`tolerW = 1e-8`)
4. Runs takeoff/field-length performance (`takeoff!`).
5. Marks `ac.is_sized .= true`.

`fly_mission!(ac, imission)` lets you evaluate a **sized** aircraft on a different mission
(different range/payload/altitude) without re-sizing structure — used for payload-range
diagrams, off-design performance decks (`AircraftDeck.jl`), BADA-style output.

`balance_aircraft!` is directly callable for ad-hoc CG/trim studies outside full sizing.

## 5. Stable vs. unstable API (semver contract, v3.0+)

See `docs/src/misc/public_api.md` for the authoritative list. Summary:

**Stable (semver-protected):**
`read_aircraft_model`, `load_default_model`, `save_aircraft_model`, `size_aircraft!`,
`fly_mission!`, `balance_aircraft!`, the `aircraft` type, `output_csv`, `plot_airf`,
`aeroperf_sweep`, `stickfig`, `plot_details`, `plot_drag_breakdown`, `PayloadRange`,
`DragPolar`, materials types (`StructuralAlloy`, etc.), enums (`EngineLocation`,
`PropSysArch`, `WingMove`, `FuelType`, `TrimVar`, `TailSizing`), unit-conversion helpers,
physical constants.

**Explicitly NOT stable** (fair game to break in minor/patch releases — good rewrite targets):
- `index.inc` / all `ig*`, `ia*`, `ie*`, `ip*`, `im*` constants (flat-array indexing scheme)
- All submodule internals (`TASOPT.engine.*`, `TASOPT.aerodynamics.*`, `TASOPT.CryoTank.*`, etc.)
- `quicksave_aircraft`/`quickload_aircraft` (JLD2 snapshots)
- CSV index helpers (`default_output_indices`, etc.)
- Internal struct field layouts not explicitly documented
- Solver/gas-model/mission-iteration internals

**Implication for the upcoming rewrite:** the flat `parg/parm/para/pare` + `index.inc` system
is the single biggest architectural liability and the maintainers have already flagged it as
priority #1 for refactoring — this is very likely where "half the codebase" pressure is
pointing. Migrating call sites to the structured objects (`Wing`, `Fuselage`, `Tail`,
`Engine`, `Options`) without breaking the 6 stable lifecycle functions above is the safest
path to a large rewrite that doesn't break downstream users.

## 6. Configuration & inputs

- Aircraft definitions are TOML files (see `example/defaults/default_input.toml` — extremely
  well-commented, effectively the parameter reference). Other examples: `default_regional.toml`,
  `default_wide.toml`, `cryo_input.toml` (LH2/CH4 fuselage tank template).
- `read_aircraft_model(path)` parses TOML → populates an `aircraft` struct. Units are
  auto-converted (SI internal); a units table lives at the top of `default_input.toml` and in
  `docs/src/index.md`.
- `load_default_model()` is a convenience synonym loading the packaged default input.
- Enums (`EngineLocation`, `PropSysArch`, `WingMove`, `FuelType`, `TrimVar`, `TailSizing`)
  select major model variants (engine architecture, tail-sizing strategy, wing-move strategy,
  fuel type, trim variable). `PropSysArch` currently: `TF` (turbofan, fully modeled),
  `ConstantTSFC` (simplified), `TE`/`FuelCellWithDuctedFan` (in development — ducted fan +
  PEM fuel cell / electric propulsion; see `src/engine/fuel_cell/`, `src/propsys/`).

## 7. Optimization workflow (how end users optimize designs)

TASOPT.jl itself does **not** ship a built-in optimizer — it exposes `size_aircraft!` as an
expensive, possibly-failing black-box function that external optimizers (NLopt, JuMP+Ipopt)
wrap. Two supported patterns, both demonstrated in `example/`:

1. **Derivative-free** (`example/example_opt.jl`): NLopt `LN_NELDERMEAD`/`LN_BOBYQA`/`LN_COBYLA`
   directly on an objective function that mutates `ac` fields via flat-array indices, calls
   `size_aircraft!`, reads back `ac.parm[imPFEI]`, and adds quadratic penalty terms for
   constraint violations (span, climb gradient, Tt3, fuel volume, fan diameter, field length).
2. **Gradient-based** (`example/example_gradient_based_opt.jl`,
   `docs/src/examples/gradient_based_optimization.md`): JuMP + Ipopt, gradients from
   `TASOPT.get_sensitivity()` (central finite differences over `size_aircraft!`, in
   `src/utils/sensitivity.jl`) rather than true autodiff (a Zygote/ForwardDiff-based autodiff
   sensitivity module is called out as WIP). Uses memoization to avoid redundant sizing calls
   per (objective, constraint) evaluation at the same point.

Both approaches perturb parameters through `ac.parg[...]`, `ac.para[...]`, `ac.pare[...]`, or
the newer struct fields (`ac.wing.layout.AR`, etc.), call `size_aircraft!(ac; printiter=false)`
inside a `try/catch` (sizing can fail to converge for infeasible designs — return `Inf`/large
penalty on failure), and read out scalar metrics like `ac.parm[imPFEI]` as the objective.

## 8. Testing & CI

- `test/runtests.jl` runs unit tests (structures, loads, aero, trefftz, atmos, heat exchanger,
  ducted fan, PEM fuel cell, materials, fuel tank, cryo tank, engine, missions, electric
  machines, outputs, IO) plus `regression_test_size_aircraft.jl` (full sizing regression against
  known-good numbers — **the most important test to watch when refactoring the sizing loop**).
- `test/benchmark_sizing.jl` / `benchmark_elements.jl` — perf benchmarks (`BenchmarkTools.jl`),
  run manually to catch speed regressions.
- CI (`.github/workflows/CI.yml`) runs on Julia `1` (latest stable) and `lts` (1.10) on Ubuntu,
  requires tests to pass before PR review (see README "Collaboration guide").
- Docs are built via Documenter.jl (`docs/make.jl`) and auto-deployed on push to `main`.

## 9. Known WIP / fragile areas (from docs + code comments)

- Flat `par*` array + `index.inc` scheme — active migration target (see §5).
- NPSS-based detailed engine performance references in docs are "non-functional" leftovers;
  being replaced.
- Autodiff-based sensitivity/gradient module is WIP (currently finite-difference only).
- `TE` (turboelectric) and `FuelCellWithDuctedFan` propulsion architectures are newer/less
  mature than the turbofan (`TF`) path — expect more churn here (`src/engine/fuel_cell/`,
  `src/propsys/`, `src/engine/ducted_fan/`).
- `Base.getproperty` on `aircraft` has special-cased shortcuts (`ac.parad`, `ac.pared`,
  `ac.parmd` → view into design-mission slice `[..., 1]`) — easy to miss when reading code
  that doesn't obviously use these fields.
- Type-stability/allocation caveats are extensively documented in
  `docs/src/misc/fordevs.md` (abstract-typed struct fields, `SVector` construction gotchas,
  closure-over-outer-scope-variable type instability). Read before adding new hot-path structs.

## 10. Sandbox environment setup (no internet / no git access)

This dev sandbox has **no outbound network access** and **no working git credentials**
(`git`/`LibGit2` calls fail with `unable to access '~/.gitconfig'` / `403` on
`pkg.julialang.org`). A plain `Pkg.instantiate()` or `Pkg.add()` will fail. This has already
been solved once — do **not** re-diagnose from scratch, just follow this recipe.

**Why it works at all:** a global Julia environment (`~/.julia/environments/v1.13/`) already
has TASOPT's full dependency tree resolved and package sources cached in
`~/.julia/packages/`, plus a packed copy of the General registry in `~/.julia/registries/`.
As long as the repo's `Project.toml` only needs packages/versions already in that cache,
Julia's package manager can resolve everything **offline**, with no clone/download required.

**Recipe (run once per fresh checkout / after adding a new dependency):**

```bash
cd /path/to/TASOPT.jl        # the repo root, containing Project.toml
JULIA_PKG_OFFLINE=true julia --project=. -e 'using Pkg; Pkg.instantiate()'
```

- `JULIA_PKG_OFFLINE=true` stops Pkg from trying (and failing) to hit `pkg.julialang.org` or
  clone from GitHub — it forces resolution against the local package/registry cache only.
- This generates a `Manifest.toml` **inside the repo** (already `.gitignore`d — don't commit
  it). From then on, `julia --project=.` in this repo works normally; you don't need
  `JULIA_PKG_OFFLINE=true` for every run, only when Pkg needs to re-resolve (e.g. after
  editing `Project.toml`).
- Verify it worked: `JULIA_PKG_OFFLINE=true julia --project=. -e 'using TASOPT; println(pathof(TASOPT))'`
  should print a path **inside this repo's `src/`**, confirming you're running the dev
  checkout, not the separately-registered copy.
- Run any script the same way, e.g.:
  `JULIA_PKG_OFFLINE=true julia --project=. example/run_widebody_payload_range.jl`

**If you add a brand-new dependency for a new submodel** (edit `Project.toml`'s `[deps]`):
1. Try the recipe above first (`Pkg.instantiate()` offline). If the new package (or the
   specific version compatible with everything else) is already sitting in
   `~/.julia/packages/`, this just works, no network needed.
2. If offline resolution fails because the exact package/version isn't cached, check
   `~/.julia/environments/v1.13/Manifest.toml` — that environment has pulled in a much wider
   dependency graph (e.g. all of `DifferentialEquations.jl`'s sub-packages) than this repo's
   `Project.toml` declares, so the package you need may already be cached there under a
   different resolved version. Cross-check `[[deps.<PackageName>]]` entries between the two
   manifests.
3. If a package is genuinely not cached anywhere and network access is unavailable, you'll
   need to either: get it added to the sandbox's package cache out-of-band, or temporarily
   stub/avoid the dependency until network access is restored. Flag this rather than
   fighting it silently.

**Do not** try to `Pkg.develop()` into a scratch directory outside the repo (e.g. `/tmp/...`)
as a workaround — an earlier pass did this before finding the cleaner fix above. It works but
is unnecessary indirection; always prefer `--project=.` resolving straight into the repo's own
`Manifest.toml` as shown above.

**Plotting backend note:** headless plot generation (`Plots.jl`/GR backend) prints harmless
`GKSserver`/`GKS: Open failed` warnings on this sandbox (no display server) — these do not
affect `savefig()`/PNG output and can be ignored.

### Use Revise.jl while actively editing source

Once we start editing files under `src/` (new submodels, refactors), plain `using TASOPT` in
a fresh `julia` process will NOT pick up subsequent edits without restarting. To iterate
quickly, start scripts/REPL sessions with `Revise` loaded first so source edits hot-reload:

```julia
using Revise
using TASOPT
```

- Works for edits to existing functions/methods; struct/field changes still usually require a
  fresh session (Revise will warn when it can't hot-patch).
- Cheap to always do this during development — costs nothing when not editing anything.
- Not needed for one-off validation runs (like `run_widebody_payload_range.jl`) where a fresh
  process per run is fine and simpler.

### Git worktrees for a stable fallback while rewriting

Since large chunks of `src/` are about to be rewritten, consider using a `git worktree` to
keep a known-good `main` checked out side-by-side with the active rewrite branch, e.g.:

```bash
git worktree add ../TASOPT.jl-stable main
```

This gives a second working directory pointed at `main` (no duplicate clone/history) that you
can immediately fall back to — run regression tests, regenerate a validation plot, or just
diff behavior — if the active rewrite branch breaks sizing convergence or introduces subtle
bugs. Each worktree can have its own `Manifest.toml`, so the offline-instantiate recipe above
should be re-run once per worktree.

## 11. Where to look first for common tasks

| Task | Start here |
|---|---|
| Add/modify a design parameter | `example/defaults/default_input.toml` (units/comments), `src/IO/read_input.jl`, relevant struct in `src/data_structs/` |
| Understand sizing math | `docs/drela_TASOPT_2p16/tasopt.pdf`, `docs/src/sizing/*.md`, `src/sizing/size_aircraft.jl` |
| Understand mission/fuel burn | `docs/src/sizing/sizing.md` §Mission evaluation, `src/mission/mission_iteration.jl` |
| Add a new engine architecture | `src/engine/engine.jl` dispatch, existing `turbofan/`, `ducted_fan/`, `fuel_cell/` as templates, `Engine` struct in `data_structs/engine.jl` |
| Add an optimization study | `example/example_opt.jl` (derivative-free) or `example/example_gradient_based_opt.jl` (gradient-based) |
| Check what's semver-stable before renaming something | `docs/src/misc/public_api.md` |
| Run tests | `julia -e 'using Pkg; Pkg.test("TASOPT")'` from repo root (with `dev .`'d package) |
