"""
main_execution.jl - Fleet batch-optimization driver

For every *.toml aircraft input file in example/fleet/inputs/:
  1. Load the aircraft (read_aircraft_model)
  2. Run a design optimization to convergence with stickfig output
  3. Evaluate its fuel-burn performance envelope  
  4. Save the optimized aircraft (TOML), parameters (JLD2), figure (PNG), 
     fuel-burn envelope (JLD2) and append summary row to CSV index

Usage:
    julia --project=. example/fleet/main_execution.jl

Environment variables (all have sensible defaults):
    FLEET_INPUTS_DIR     default: this_dir/inputs
    FLEET_OUTPUTS_DIR    default: this_dir/outputs
    FLEET_OPT_MAXEVAL    default: 150
    FLEET_OPT_FTOL_REL   default: 1e-5
    FLEET_ENVELOPE_STEP_KM default: 100.0
    FLEET_PAYLOAD_FRACTION default: 0.8
"""

# Suppress GKS interim file creation during plotting
ENV["GKS_WSTYPE"] = "png"
ENV["GR_OUTPUT_FILE"] = "/dev/null"  # Suppress GR interim output files
using TASOPT, Plots, GR
include(TASOPT.__TASOPTindices__)

const __fleet_dir__ = @__DIR__
include(joinpath(__fleet_dir__, "lib", "fleet_io.jl"))
include(joinpath(__fleet_dir__, "lib", "fleet_optimize.jl"))

# Configuration
inputs_dir  = get(ENV, "FLEET_INPUTS_DIR",  joinpath(__fleet_dir__, "inputs"))
outputs_dir = get(ENV, "FLEET_OUTPUTS_DIR", joinpath(__fleet_dir__, "outputs"))
aircraft_models_outdir = joinpath(outputs_dir, "aircraft_models")
aircraft_params_outdir = joinpath(outputs_dir, "aircraft_params")
fuel_burn_outdir = joinpath(outputs_dir, "fuel_burn")
payload_outdir   = joinpath(outputs_dir, "payload_range")
aircraft_fig_outdir = joinpath(outputs_dir, "aircraft_figures")
index_csv_path   = joinpath(outputs_dir, "index.csv")

opt_maxeval  = parse(Int, get(ENV, "FLEET_OPT_MAXEVAL", "150"))
opt_ftol_rel = parse(Float64, get(ENV, "FLEET_OPT_FTOL_REL", "1e-5"))
envelope_step_km = parse(Float64, get(ENV, "FLEET_ENVELOPE_STEP_KM", "100.0"))
payload_fraction = parse(Float64, get(ENV, "FLEET_PAYLOAD_FRACTION", "0.8"))

mkpath(outputs_dir)
mkpath(aircraft_models_outdir)
mkpath(aircraft_params_outdir)
mkpath(fuel_burn_outdir)
mkpath(payload_outdir)
mkpath(aircraft_fig_outdir)

# Preload GR backend
gr()

# Discover and process fleet
input_files = discover_input_files(inputs_dir)
n_total = length(input_files)
println("="^70)
println("TASOPT fleet batch run")
println("  inputs_dir       = ", inputs_dir)
println("  outputs_dir      = ", outputs_dir)
println("  n aircraft found = ", n_total)
println("  opt_maxeval      = ", opt_maxeval)
println("  opt_ftol_rel     = ", opt_ftol_rel)
println("="^70)

n_ok = 0
n_failed = 0
t_start = time()
t_batch = @elapsed for (i, input_path) in enumerate(input_files)

    name = splitext(basename(input_path))[1]
    println("\n[$i/$n_total] ", name, "  (", input_path, ")")

    t_ac = @elapsed try
        ac = read_aircraft_model(input_path)

        ret_code, pfei_design = optimize_aircraft!(ac; maxeval=opt_maxeval,
                                                    ftol_rel=opt_ftol_rel)

        if !ac.is_sized[1] || !isfinite(pfei_design)
            error("optimize_aircraft! did not leave aircraft in a sized, " *
                  "finite-PFEI state (return code = $ret_code)")
        end

        env = evaluate_fuel_burn_envelope(ac; payload_fraction=payload_fraction,
                                           step_km=envelope_step_km)
        n_env_pts = length(env.range_km)
        n_env_ok  = count(env.converged)

        out_toml_path = save_fleet_aircraft(ac, name, aircraft_models_outdir)
        
        out_jld2_params_path = joinpath(aircraft_params_outdir, string(name, "_params.jld2"))
        try
            TASOPT.quicksave_aircraft(ac, out_jld2_params_path)
            println("  -> Saved aircraft parameters to ", basename(out_jld2_params_path))
        catch e
            println("  -> WARNING: failed to quicksave: ", sprint(showerror, e))
        end
        
        aircraft_fig_path = joinpath(aircraft_fig_outdir, string(name, "_aircraft.pdf"))
        try
            println("  -> Creating aircraft stick figure...")
#            open("/dev/null", "w") do devnull
#                redirect_stdout(devnull) do
#                    redirect_stderr(devnull) do
                        fig = TASOPT.stickfig(ac)
                        Plots.savefig(fig, aircraft_fig_path)
#                    end
#                end
#            end
            println("  -> Saved aircraft figure to ", basename(aircraft_fig_path))
        catch e
            println("  -> WARNING: failed to create aircraft figure: ", sprint(showerror, e))
        end

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
                 " kJ/kg-km  envelope=", n_env_ok, "/", n_env_pts, " converged")
        global n_ok += 1

        create_payload_plot = lowercase(get(ENV, "FLEET_CREATE_PAYLOAD_PLOT", "true")) in ("1","true","yes")
        if create_payload_plot
            payload_outfile = joinpath(payload_outdir, string(name, ".pdf"))
            try
                println("  -> Creating payload-range diagram...")
                open("/dev/null", "w") do devnull
                    redirect_stdout(devnull) do
                        redirect_stderr(devnull) do
                            fig = TASOPT.PayloadRange(ac; filename=payload_outfile, Ldebug=false)
                        end
                    end
                end
                println("  -> Saved payload diagram to ", basename(payload_outfile))
            catch e
                println("  -> WARNING: failed to create payload diagram: ", sprint(showerror, e))
            end
        end

    catch e
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
            println("  -> also failed to log this failure: ", e2)
        end
        global n_failed += 1
    end

    total_elapsed = time() - t_start
    println("  (", round(t_ac, digits=1), " s)  total elapsed= ", round(total_elapsed, digits=1), " s")
end

println("\n" * "="^70)
println("Fleet batch run complete in ", round(t_batch/60, digits=2), " min")
println("  succeeded: ", n_ok, " / ", n_total)
println("  failed:    ", n_failed, " / ", n_total)
println("  aircraft models  -> ", aircraft_models_outdir)
println("  aircraft params  -> ", aircraft_params_outdir)
println("  aircraft figures -> ", aircraft_fig_outdir)
println("  fuel-burn outputs -> ", fuel_burn_outdir)
println("  index            -> ", index_csv_path)
println("="^70)

# Clean up any interim GKS plot files (gks-*.png) that Plots.jl may have created
# (Commented out to preserve interim files for inspection)
# try
#     for gks_file in readdir(".")
#         if startswith(gks_file, "gks-") && endswith(gks_file, ".png")
#             rm(gks_file)
#         end
#     end
#     println("\nCleaned up interim GKS plot files.")
# catch e
#     println("\nNote: Could not clean up interim GKS files: ", sprint(showerror, e))
# end
