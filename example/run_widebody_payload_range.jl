"""
run_widebody_payload_range.jl

Validation script: loads the wide-body TASOPT input model, sizes the aircraft,
exercises `fly_mission!` (internally, via `PayloadRange`) over a fleet of
off-design missions, and saves the resulting payload-range / PFEI diagram
as a PNG.

Usage:
    julia --project=. example/run_widebody_payload_range.jl
"""

using TASOPT
using Plots

# Bring in the named par-array indices (igAR, imPFEI, imWfuel, etc.)
include(TASOPT.__TASOPTindices__)

# ---------------------------------------------------------------------
# 1) Select and load the input TOML file
# ---------------------------------------------------------------------
input_file = joinpath(TASOPT.__TASOPTroot__, "..", "example", "defaults", "default_wide.toml")
println("Loading aircraft model from: ", input_file)

ac = read_aircraft_model(input_file)
println("Loaded aircraft: ", ac.name)

# ---------------------------------------------------------------------
# 2) Size the aircraft on its design mission
# ---------------------------------------------------------------------
println("Sizing aircraft...")
time_size = @elapsed size_aircraft!(ac)
println("Aircraft sized in $(round(time_size, digits=2)) s")

summary(ac)

# ---------------------------------------------------------------------
# 3) Sanity check: fly the design mission explicitly with fly_mission!
#    (PayloadRange also calls fly_mission! internally for every
#    range/payload combination, but we do one explicit call here as a
#    direct validation of that entry point)
# ---------------------------------------------------------------------
println("\nRunning fly_mission! on the design mission (mission 1) as a validation check...")
fly_mission!(ac, 1; itermax=35)
println("Design mission PFEI = ", round(ac.parm[imPFEI, 1], digits=4), " kJ/kg-km")
println("Design mission fuel burn = ", round(ac.parm[imWfuel, 1]/9.81/1e3, digits=2), " tonnes")

# ---------------------------------------------------------------------
# 4) Generate and save the payload-range diagram
# ---------------------------------------------------------------------
outdir = joinpath(TASOPT.__TASOPTroot__, "..", "example")
outfile = joinpath(outdir, "widebody_payload_range.png")

println("\nGenerating payload-range diagram...")
fig = TASOPT.PayloadRange(ac; filename=outfile, Ldebug=false)

println("Saved payload-range diagram to: ", outfile)
