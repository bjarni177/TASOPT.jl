"""
main_execution.jl

Fleet batch-optimization driver.

For every `*.toml` aircraft input file in `example/fleet/inputs/`:
  1. Load the aircraft (`read_aircraft_model`).
  2. Run a design optimization to convergence (`optimize_aircraft!`, in
     `lib/fleet_optimize.jl`) — this both optimizes and sizes the aircraft.
  3. Evaluate its fuel-burn performance envelope from 100 km to its design
     range in 100 km increments, at 80% of max payload capacity
     (`evaluate_fuel_burn_envelope`, in `lib/fleet_optimize.jl`).
  4. Save the optimized aircraft (TOML) and the fuel-burn envelope (JLD2)
     to SEPARATE output locations (`lib/fleet_io.jl`), and append one row
     to a tidy CSV index that maps input .toml -> output .toml/.jld2 paths
     and key summary metrics. That index is the fast "aircraft -> envelope"
     lookup table for later use (including from MATLAB via readtable() on
     the CSV + h5read()/h5info() on each aircraft's .jld2, which is plain
     HDF5 under the hood — see SPEC.md).

Designed to run unattended over O(1000) aircraft: every aircraft is
processed inside a `try/catch` so one failure (non-convergence, bad input
file, etc.) cannot halt the batch. Progress and failures are logged to
stdout as the loop proceeds; a final summary is printed at the end.

Usage:
    julia --project=. example/fleet/main_execution.jl
    # or, from within the example/fleet/ directory:
    julia --project=../.. main_execution.jl

Optional environment variables (all have sensible defaults for a smoke
test; override for a full ~1000-aircraft production run):
    FLEET_INPUTS_DIR     default: <this dir>/inputs
    FLEET_OUTPUTS_DIR    default: <this dir>/outputs
    FLEET_OPT_MAXEVAL    default: 150   (NLopt function-eval budget per aircraft)
    FLEET_OPT_FTOL_REL   default: 1e-5
    FLEET_ENVELOPE_STEP_KM default: 100.0
    FLEET_PAYLOAD_FRACTION default: 0.8
"""

using TASOPT
include(TASOPT.__TASOPTindices__)

const __fleet_dir__ = @__DIR__
include(joinpath(__fleet_dir__, "lib", "fleet_io.jl"))
include(joinpath(__fleet_dir__, "lib", "fleet_optimize.jl"))

# ---------------------------------------------------------------------
# Configuration (env-overridable; see docstring above)
# ---------------------------------------------------------------------
inputs_dir  = get(ENV, "FLEET_INPUTS_DIR",  joinpath(__fleet_dir__, "inputs"))
outputs_dir = get(ENV, "FLEET_OUTPUTS_DIR", joinpath(__fleet_dir__, "outputs"))
aircraft_outdir  = joinpath(outputs_dir, "aircraft")
fuel_burn_outdir = joinpath(outputs_dir, "fuel_burn")
index_csv_path   = joinpath(outputs_dir, "index.csv")

opt_maxeval  = parse(Int, get(ENV, "FLEET_OPT_MAXEVAL", "150"))
opt_ftol_rel = parse(Float64, get(ENV, "FLEET_OPT_FTOL_REL", "1e-5"))
envelope_step_km = parse(Float64, get(ENV, "FLEET_ENVELOPE_STEP_KM", "100.0"))
payload_fraction = parse(Float64, get(ENV, "FLEET_PAYLOAD_FRACTION", "0.8"))

mkpath(outputs_dir)
mkpath(aircraft_outdir)
mkpath(fuel_burn_outdir)

# ---------------------------------------------------------------------
# Discover fleet
# ---------------------------------------------------------------------
input_files = discover_input_files(inputs_dir)
n_total = length(input_files)
println("="^70)
println("TASOPT fleet batch run")
println("  inputs_dir       = ", inputs_dir)
println("  outputs_dir       = ", outputs_dir)
println("  n aircraft found = ", n_total)
println("  opt_maxeval      = ", opt_maxeval)
println("  opt_ftol_rel     = ", opt_ftol_rel)
println("  envelope_step_km = ", envelope_step_km)
println("  payload_fraction = ", payload_fraction)
println("="^70)

n_ok = 0
n_failed = 0
t_batch = @elapsed for (i, input_path) in enumerate(input_files)

    name = splitext(basename(input_path))[1]
    println("\n[$i/$n_total] ", name, "  (", input_path, ")")

    t_ac = @elapsed try
        # ---- 1) Load -----------------------------------------------------
        ac = read_aircraft_model(input_path)

        # ---- 2) Optimize + size -------------------------------------------
        ret_code, pfei_design = optimize_aircraft!(ac; maxeval=opt_maxeval,
                                                    ftol_rel=opt_ftol_rel)

        if !ac.is_sized[1] || !isfinite(pfei_design)
            error("optimize_aircraft! did not leave aircraft in a sized, " *
                  "finite-PFEI state (return code = $ret_code)")
        end

        # ---- 3) Fuel-burn performance envelope -----------------------------
        env = evaluate_fuel_burn_envelope(ac; payload_fraction=payload_fraction,
                                           step_km=envelope_step_km)
        n_env_pts = length(env.range_km)
        n_env_ok  = count(env.converged)

        # ---- 4) Save aircraft + envelope + index row -----------------------
        out_toml_path = save_fleet_aircraft(ac, name, aircraft_outdir)
        out_jld2_path = save_fleet_envelope(name, fuel_burn_outdir;
            range_km          = env.range_km,
            fuel_burn_MJ      = env.fuel_burn_MJ,
            converged         = env.converged,
            payload_N         = env.payload_N,
            payload_fraction  = payload_fraction,
            design_range_km   = env.design_range_km,
            pfei_design       = pfei_design)

        architecture = string(ac.options.opt_prop_sys_arch)
        fuel_type    = string(ac.options.opt_fuel)

        append_index_row!(index_csv_path, Dict(
            "name"                     => name,
            "input_toml"               => input_path,
            "architecture"             => architecture,
            "fuel_type"                => fuel_type,
            "converged"                => true,
            "pfei_design_kJ_per_kg_km" => pfei_design,
            "design_range_km"          => env.design_range_km,
            "payload_capacity_N"       => ac.parg[igWpaymax],
            "payload_fraction_used"    => payload_fraction,
            "output_toml_path"         => out_toml_path,
            "fuel_burn_jld2_path"      => out_jld2_path,
            "opt_return_code"          => string(ret_code),
            "n_envelope_points"        => n_env_pts,
            "n_envelope_converged"     => n_env_ok,
            "error_message"            => "",
        ))

        println("  -> OK  PFEI=", round(pfei_design, digits=4),
                 " kJ/kg-km  envelope=", n_env_ok, "/", n_env_pts, " pts converged")
        global n_ok += 1

    catch e
        # Never let one aircraft's failure kill the batch.
        msg = sprint(showerror, e)
        println("  -> FAILED: ", msg)
        try
            append_index_row!(index_csv_path, Dict(
                "name"        => name,
                "input_toml"  => input_path,
                "converged"   => false,
                "error_message" => msg,
            ))
        catch e2
            println("  -> also failed to log this failure to the index: ", e2)
        end
        global n_failed += 1
    end

    println("  (", round(t_ac, digits=1), " s)")
end

println("\n" * "="^70)
println("Fleet batch run complete in ", round(t_batch/60, digits=2), " min")
println("  succeeded: ", n_ok, " / ", n_total)
println("  failed:    ", n_failed, " / ", n_total)
println("  aircraft outputs -> ", aircraft_outdir)
println("  fuel-burn outputs -> ", fuel_burn_outdir)
println("  index            -> ", index_csv_path)
println("="^70)
