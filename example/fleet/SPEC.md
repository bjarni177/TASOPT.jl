# Fleet batch-optimization spec — concise

This file is the source of truth for the scripts under example/fleet/. The pipeline:
- For each aircraft TOML in inputs/, load with TASOPT.read_aircraft_model.
- Run a design optimization (NLopt, PFEI objective with penalty constraints) to size the aircraft.
- Evaluate a fuel-burn envelope: ranges 100:100:R_design_km (km), plus the design range if not an exact multiple, each flown at 80% of igWpaymax (payload capacity in N) using fly_mission!.
- Save the optimized aircraft (TOML) and the fuel-burn envelope (JLD2/HDF5-compatible plain arrays/scalars/strings/bools) into separate output locations.
- Repeat across many aircraft; be robust to per-aircraft failures (do not crash whole batch).

Verified facts (short)
- TASOPT.read_aircraft_model(path) → aircraft struct. All ac.parg/parm/para/pare arrays use SI units regardless of TOML.
- Use size_aircraft!(ac; iter=35, initwgt=false, Ldebug=false, printiter=true) to size. In batch set printiter=false. size_aircraft! sets ac.is_sized .= true on success and can throw — always try/catch in objective and return a large penalty (e.g. 1e6/Inf) on failure.
- fly_mission!(ac, imission; itermax=35, initializes_engine=true, opt_prescribed_cruise_parameter="altitude", printTO=true) evaluates an off-design mission on an already-sized aircraft (requires ac.is_sized[1] == true). It mutates parm/para/pare for imission, can throw or converge to infeasible states — wrap each call in try/catch.
- Off-design pattern: duplicate the design mission into mission slot 2, update mission-2 range and payload, reset cruise alt/CL before each call, and reuse the single ac_od for the whole sweep. Use the same approach as PayloadRange.

Units and fuel-burn quantity to save
- ac.parg[igWpaymax], ac.parm[imWpay,*] are weights in Newtons (N).
- ac.parg[igRange], ac.parm[imRange,*] are meters.
- ac.parg[igLHVfuel] is J/kg.
- ac.parm[imWfuel,*] is total fuel weight loaded (N), including reserve: Wfuel = Wburn * (1 + freserve).
- ac.parm[imPFEI,*] is kJ/kg-km, computed as PFEI = (Wburn/gee) * LHVfuel / (Wpay * Range).
- TASOPT.gee = 9.81 m/s^2.

Compute fuel burn energy (MJ) per mission point (exclude reserve):

Wfuel_N   = ac_od.parm[imWfuel, 2]
freserve  = ac.parg[igfreserve]
Wburn_N   = Wfuel_N / (1 + freserve)
fuel_burn_MJ = (Wburn_N / TASOPT.gee) * ac.parg[igLHVfuel] / 1e6

Save fuel_burn_MJ (MJ). Document the convention used in metadata.

I/O and tool notes
- include(TASOPT.__TASOPTindices__) to bring ig*/im*/ia* constants into scope.
- TASOPT.save_aircraft_model(ac, datafile; save_output=false) writes optimized aircraft TOML.
- Use JLD2 for envelope output. Store only plain arrays/scalars/strings/bools so files are readable from MATLAB via h5read/h5info. HDF5.jl and MAT.jl are not available in the sandbox; JLD2 is available and sufficient.
- Wing/tail fields forward unknown gets to .layout (e.g. ac.wing.AR works).

Directory layout (relative to example/fleet/)
- SPEC.md (this file)
- main_execution.jl (driver)
- lib/
  - fleet_io.jl
  - fleet_optimize.jl
- inputs/*.toml
- outputs/
  - aircraft/   (.toml per optimized aircraft via save_aircraft_model)
  - fuel_burn/  (.jld2 per aircraft)
  - index.csv   (one row per aircraft mapping input → outputs and metadata)

Mandatory function contracts (exact signatures)

lib/fleet_io.jl (put `using TASOPT, JLD2` at top)

"""
    discover_input_files(inputs_dir::String) -> Vector{String}

Return sorted absolute paths of all `*.toml` files directly inside
`inputs_dir` (not recursive). Empty vector (with a `@warn`) if none found.
"""
function discover_input_files(inputs_dir::String) end

"""
    save_fleet_aircraft(ac, name::String, aircraft_outdir::String) -> String

Writes optimized/sized aircraft `ac` to joinpath(aircraft_outdir, name * ".toml") using TASOPT.save_aircraft_model. Creates aircraft_outdir if needed. Returns written path.
"""
function save_fleet_aircraft(ac, name::String, aircraft_outdir::String) end

"""
    save_fleet_envelope(name::String, fuel_burn_outdir::String;
        range_km::Vector{Float64}, fuel_burn_MJ::Vector{Float64},
        converged::Vector{Bool}, payload_N::Float64, payload_fraction::Float64,
        design_range_km::Float64, pfei_design::Float64) -> String

Writes ONE JLD2 file per aircraft to joinpath(fuel_burn_outdir, name * ".jld2"). Store ONLY plain arrays/scalars/strings/bools as top-level keys:
  - "name" (String)
  - "range_km" (Vector{Float64})
  - "fuel_burn_MJ" (Vector{Float64})
  - "converged" (Vector{Bool})
  - "payload_N" (Float64)
  - "payload_fraction" (Float64)
  - "design_range_km" (Float64)
  - "pfei_design" (Float64)
Overwrite existing file if present (use "w"). Create fuel_burn_outdir if needed. Return written path.
"""
function save_fleet_envelope(name::String, fuel_burn_outdir::String;
    range_km, fuel_burn_MJ, converged, payload_N, payload_fraction,
    design_range_km, pfei_design) end

"""
    append_index_row!(index_csv_path::String, row::Dict)

Append one row to a tidy CSV index at index_csv_path (create with header on first call). Required keys (fixed column order):
  "name", "input_toml", "architecture", "fuel_type", "converged",
  "pfei_design_kJ_per_kg_km", "design_range_km", "payload_capacity_N",
  "payload_fraction_used", "output_toml_path", "fuel_burn_jld2_path",
  "opt_return_code", "n_envelope_points", "n_envelope_converged",
  "error_message"
Missing keys => write empty string. Strip embedded commas/newlines from string fields (e.g. replace(r"[\\r\\n,]" => " ")).
"""
function append_index_row!(index_csv_path::String, row::Dict) end


lib/fleet_optimize.jl (put `using TASOPT, NLopt` and include(TASOPT.__TASOPTindices__) at top)

"""
    optimize_aircraft!(ac; maxeval=150, ftol_rel=1e-5) -> (return_code::Symbol, pfei::Float64)

Run an NLopt derivative-free (LN_NELDERMEAD) optimization in-place, minimizing PFEI (ac.parm[imPFEI,1]) with penalty constraints. Use 14 design variables (same set as examples). Read initial values/bounds from ac using relative offsets (e.g. [0.5*AR0,1.5*AR0] clipped to sane absolute limits). In the objective call size_aircraft!(ac; iter=50, printiter=false) wrapped in try/catch and return 1e6 on exception. Expose maxeval and ftol_rel as kwargs. After NLopt returns, re-apply the best optx to ac and call size_aircraft!(ac; printiter=false) once more in try/catch; if that re-size fails return (:REOPT_FAILED, Inf) rather than throwing. Return (return_code, pfei) where pfei = ac.parm[imPFEI,1] after final sizing.
"""
function optimize_aircraft!(ac; maxeval=150, ftol_rel=1e-5) end

"""
    evaluate_fuel_burn_envelope(ac; payload_fraction=0.8, step_km=100.0, itermax=35) -> NamedTuple

`ac` MUST already be sized. Build ac_od by duplicating mission-1 into mission-2 (including engine.heat_exchangers HXgas_mission if present). Sweep range_km = 100.0:step_km:design_range_km (design_range_km = ac.parg[igRange]/1e3) and append design_range_km if needed. Payload at every point = payload_fraction * ac.parg[igWpaymax] (N). For each point set ac_od.parm[imRange,2], ac_od.parm[imWpay,2], reset cruise alt/CL to design values, call fly_mission!(ac_od, 2; itermax=itermax, printTO=false) inside try/catch. On success check feasibility (WTO <= WMTO + 1 N, Wfuel <= Wfmax + 1 N, weights >= 0). If feasible compute fuel_burn_MJ via Wburn=Wfuel/(1+freserve); fuel_burn_MJ = (Wburn/gee)*LHV/1e6 and mark converged[i]=true. On failure set fuel_burn_MJ[i]=NaN, converged[i]=false and continue. Return NamedTuple (range_km, fuel_burn_MJ, converged, payload_N, design_range_km) ready to splat into save_fleet_envelope.
"""
function evaluate_fuel_burn_envelope(ac; payload_fraction=0.8, step_km=100.0, itermax=35) end

Final notes
- Follow the exact try/catch per-aircraft and per-off-design-point pattern so one failure never aborts the batch.
- Keep JLD2 outputs plain HDF5-compatible types for MATLAB interoperability.
- main_execution.jl should orchestrate: discover inputs → optimize_aircraft! → evaluate_fuel_burn_envelope → save outputs → append_index_row!.
