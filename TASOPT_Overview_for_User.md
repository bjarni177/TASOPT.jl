# How TASOPT.jl Works: A User's Guide

## What TASOPT is

TASOPT (Transport Aircraft System OPTimization) is a **conceptual/preliminary aircraft
design tool**. Given a description of an airplane — its fuselage layout, wing planform,
tail geometry, engine cycle, and the missions it needs to fly — TASOPT calculates a fully
consistent, weight-converged aircraft design: how heavy it is, how much fuel it burns, how
it balances and trims, how fast it can climb, how long a runway it needs, and how efficient
it is (measured as fuel burned per unit of payload carried per unit distance flown). This
Julia implementation (`TASOPT.jl`) is a rewrite of Mark Drela's original FORTRAN code from
MIT, and it is actively maintained by the MIT Lab for Aviation and the Environment.

TASOPT is not a CAD tool and not a full CFD/FEA suite. It uses **physics-based, reduced-order
models** for each discipline — 2D viscous-inviscid boundary layer aerodynamics, beam-theory
structural sizing, and thermodynamic cycle analysis for the engine — coupled together tightly
enough that changing one thing (say, wing aspect ratio) automatically propagates through
structural weight, aerodynamic drag, required engine thrust, fuel burn, and balance. This
coupling is what makes it suitable for optimization: you can hand a handful of design
variables to an external optimizer and trust that TASOPT will return a self-consistent,
physically sized aircraft for any combination you try (or report a convergence failure if the
combination is infeasible).

## The core workflow

Using TASOPT from a user's perspective typically looks like this:

1. **Describe the aircraft in a TOML input file.** This is a plain-text configuration file
   specifying everything from fuselage radius and cabin layout, to wing aspect ratio and
   sweep, to engine bypass ratio and turbine inlet temperature, to the design mission (range,
   payload, cruise altitude, cruise Mach number). A fully commented example ships with the
   package (`default_input.toml`), and there are pre-built templates for a regional jet, a
   widebody, and a cryogenic-hydrogen-fueled aircraft.

2. **Load the model.** `load_default_model()` or `read_aircraft_model("myplane.toml")` parses
   the file into an `aircraft` object — a large data structure holding every design parameter
   and every result once computed.

3. **Size the aircraft.** `size_aircraft!(ac)` is the single most important function in the
   package. It runs a fixed-point iteration: guess a takeoff weight, compute structural
   weights for the fuselage, wing, and tails based on beam theory and load factors, compute the
   drag at cruise, size the engine to produce enough thrust to overcome that drag, fly the
   full mission (climb, cruise, descent) to determine fuel burn, update the takeoff weight, and
   repeat until everything converges (typically 10-30 iterations, to a tolerance of 1e-8 on
   weight). It also determines the fuselage/wing/tail geometry needed for the fuel tank (for
   cryogenic fuels), balances the aircraft for a target static margin, and computes takeoff/
   balanced-field-length performance.

4. **Evaluate off-design performance.** Once an aircraft is sized, `fly_mission!(ac,
   mission_number)` flies it on a *different* mission — shorter range, lighter payload, a
   different cruise altitude — without re-sizing the structure. This lets you generate
   payload-range diagrams, drag polars, or an "aircraft performance deck" describing fuel burn
   and emissions across a fleet's typical operations (useful for feeding into higher-level
   fleet/environmental models).

5. **Inspect and export results.** `summary(ac)` prints geometry/weight/aero highlights.
   `stickfig(ac)`, `plot_details(ac)`, `plot_drag_breakdown(ac)`, `PayloadRange(ac)`, and
   `DragPolar(ac)` generate standard engineering plots. `output_csv(ac)` dumps parameters to a
   spreadsheet-friendly format, and `save_aircraft_model(ac, path)` writes a TOML you can hand
   to someone else or re-load later.

6. **Optimize.** TASOPT does not include its own optimizer; instead it plays the role of an
   expensive black-box function that you wrap with a general-purpose optimization package
   (NLopt or JuMP/Ipopt are used in the provided examples). You write an objective function
   that: takes a vector of design variables, writes them into the `ac` object, calls
   `size_aircraft!(ac)`, reads out a scalar metric (almost always **PFEI** — payload-fuel
   energy intensity, essentially fuel energy burned per unit payload per unit distance — the
   standard proxy for fuel efficiency/environmental performance), applies penalty terms for any
   violated constraints, and returns that value. Two supported patterns are demonstrated:
   derivative-free simplex/pattern-search optimization (NLopt, good for a first exploration or
   for noisy/discontinuous problems), and gradient-based optimization (JuMP + Ipopt, using
   finite-difference sensitivities computed by TASOPT's own sensitivity module, which perturbs
   each design variable and re-runs `size_aircraft!` to build a gradient). A fully autodiff-based
   sensitivity pipeline is in development but not yet the default.

## What can be "optimized" versus "sized"

It's worth being precise about the distinction TASOPT draws between quantities that are
**design variables** (things an optimizer or user chooses, held fixed during a sizing run) and
quantities that are **sized outputs** (things TASOPT computes to make the aircraft
self-consistent, given the design variables and the mission).

**Typical design/optimization variables** (these are the knobs turned in the example
optimization scripts and are the natural targets for an outer optimization loop):

- *Wing geometry*: aspect ratio, sweep angle, span, inner/outer panel taper ratios, root and
  spanbreak thickness-to-chord ratios, spar box width.
- *Aerodynamic operating point*: cruise lift coefficient, cruise altitude, cruise Mach number,
  spanwise CL distribution ratios (break/root and tip/root CL ratios) at various flight phases.
- *Engine cycle parameters*: turbine inlet temperature (Tt4), overall/fan/compressor pressure
  ratios (OPR, fan pressure ratio, HPC/LPC pressure ratios), bypass ratio, gear ratio.
- *Tail sizing strategy and parameters*: horizontal/vertical tail volume coefficients (or
  alternative sizing criteria such as max-forward-CG trim or one-engine-out trim), sweep, taper.
- *Fuselage/cabin layout*: radius, seat pitch/width, number of seat decks, cargo container type.
- *Mission definition itself*: design range, payload, cruise altitude — these define the
  problem but are frequently varied in sensitivity or payload-range studies.
- *Material and structural allowables*: stress limits, safety factors, choice of structural
  alloy — less commonly optimized but fully exposed as inputs.
- *Architecture choices* (categorical, not continuous): engine location (wing- vs.
  fuselage-mounted), propulsion architecture (turbofan, constant-TSFC simplified model,
  turboelectric, fuel-cell-driven ducted fan), fuel type (Jet-A, liquid hydrogen, liquid
  methane), wing-movement/tail-sizing strategy.

**Quantities that are "sized" (computed outputs, not free choices)** — these are what
`size_aircraft!` actually solves for, given the variables above:

- **Maximum takeoff weight (MTOW)** and the full weight breakdown: fuselage structural weight
  (shell, floor, tail cone, bending material), wing and tail structural weight (spar caps,
  shear webs, secondary structure like flaps/slats/ribs), bare engine weight plus nacelle/pylon/
  accessories, landing gear weight, and fixed/fractional weights for systems, APU, seats, and
  payload-proportional items.
- **Wing/tail planform dimensions** consistent with the specified aspect ratio, sweep, and
  taper (span, chord distribution, box structure sizing to meet stress/deflection limits under
  specified load factors).
- **Fuel tank capacity/geometry** — including, for cryogenic fuels, the insulated tank's
  thermal and structural sizing and its effect on fuselage length.
- **Engine physical size** — fan diameter, core mass flow, and all resulting engine
  performance (thrust, TSFC, spool speeds, temperatures/pressures at each engine station)
  needed to produce enough thrust to balance drag at the design cruise point and throughout
  climb/descent.
- **Aircraft balance and trim** — wing position (or horizontal tail lift/area, depending on
  the chosen trim strategy), center of gravity travel, tail incidence/elevator deflection
  needed to hit a target static margin.
- **Mission fuel burn** at every flight-profile station (taxi, takeoff, climb, cruise, descent,
  reserves), and the resulting mission-level metrics: total fuel burned, PFEI, and takeoff
  field length / balanced field length.

## Constraints commonly layered on top

Because `size_aircraft!` only guarantees internal consistency, not feasibility against
real-world limits, optimization studies typically add explicit constraints on quantities like:
maximum wing span (gate constraints), minimum climb gradient, maximum turbine metal/gas
temperature, maximum usable fuel volume, maximum fan diameter, and maximum balanced field
length. These reflect operational or certification limits that the underlying physics-based
sizing does not automatically enforce. The example optimization scripts implement these two
different ways, and it matters which one you reach for:

**Hard bounds** are the box constraints (`opt.lower_bounds`/`opt.upper_bounds` in NLopt, or
variable bounds in a JuMP `@variable` declaration) placed directly on the *design variables*
themselves (e.g. `10.5 ≤ AR ≤ 12.0`, `1400 K ≤ Tt4 ≤ 1650 K`). The optimizer is mathematically
prevented from ever evaluating `size_aircraft!` outside this box — it is not a preference, it
is the literal domain being searched. Two consequences follow: (1) the aircraft's *initial*
design-variable values must lie inside these bounds or the optimizer will error before taking
a single step, and (2) if the bounds are set too narrow, or are copied from a different
aircraft's tuned range without checking, they can silently exclude the correct optimum — or
even exclude the aircraft's own valid starting point, which typically manifests as every
single objective-function evaluation failing inside `size_aircraft!`'s engine/aero solvers
(e.g. `TFSIZE: Negative core plume velocity` from clipping an initial turbine inlet
temperature/compressor pressure ratio combination into an untested corner of the box). Hard
bounds should be set per-aircraft, wide enough to contain that aircraft's actual starting
design point, not copy-pasted verbatim from an example tuned for a different airframe.

**Soft constraints** are inequality/equality limits on *derived outputs* of a sized aircraft
(e.g. wing span, climb gradient, Tt3, fuel volume, fan diameter, balanced field length) that
the optimizer is free to violate during its search but is penalized for violating, via a
penalty term added to the objective inside the objective function itself (as in
`example/example_opt.jl`/`example/example_gradient_based_opt.jl`), or via true equality/
inequality constraints registered with a constrained solver (as in the JuMP/Ipopt pattern in
`docs/src/examples/gradient_based_optimization.md`). Because these are evaluated on the
*sized* aircraft (only knowable after `size_aircraft!` has run), the optimizer can and often
does explore through infeasible territory before settling into a feasible, penalty-free
region — this is expected behavior, not a bug, and it means an aircraft's PFEI can temporarily
look *worse* mid-optimization than an unconstrained starting point that happened to already
violate one of these limits.

In short: hard bounds restrict *where the optimizer is allowed to look*; soft constraints
penalize *what it finds once it gets there*. Reach for a hard bound when a design variable has
a genuine physical/engineering floor or ceiling that should never be crossed regardless of the
resulting design's other merits (e.g. a turbine inlet temperature limited by material limits);
reach for a soft penalty when the limit is a property of the sized aircraft that only emerges
after a full sizing pass and where you want the optimizer to be able to trade it off smoothly
against the objective (e.g. climb gradient, fuel volume).

## Summary

In short: you describe an aircraft concept and its mission in a TOML file, TASOPT converts
that into a fully weight-, aero-, structures-, and propulsion-consistent point design via
`size_aircraft!`, and you can then either inspect that single design's performance or wrap the
whole sizing process inside an external optimizer to search over wing/engine/mission design
variables for the most fuel-efficient (lowest-PFEI) aircraft subject to engineering and
operational constraints. `fly_mission!` and `balance_aircraft!` round out the toolkit for
evaluating a sized aircraft's flexibility outside its design point.
