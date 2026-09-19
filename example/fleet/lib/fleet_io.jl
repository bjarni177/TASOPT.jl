using TASOPT, JLD2

#=
  Function signatures (for orchestrator reference):
    discover_input_files(inputs_dir::String) -> Vector{String}
    save_fleet_aircraft(ac, name::String, aircraft_outdir::String) -> String
    save_fleet_envelope(name::String, fuel_burn_outdir::String; ...) -> String
    append_index_row!(index_csv_path::String, row::Dict)
=#

"""
    discover_input_files(inputs_dir::String) -> Vector{String}

Return sorted absolute paths of all `*.toml` files directly inside
`inputs_dir` (not recursive). Empty vector (with a `@warn`) if none found.
"""
function discover_input_files(inputs_dir::String)
    if !isdir(inputs_dir)
        @warn "discover_input_files: inputs_dir does not exist: $inputs_dir"
        return String[]
    end
    
    toml_files = String[]
    for filename in readdir(inputs_dir)
        if endswith(filename, ".toml")
            push!(toml_files, joinpath(inputs_dir, filename))
        end
    end
    
    if isempty(toml_files)
        @warn "discover_input_files: no .toml files found in $inputs_dir"
    end
    
    sort!(toml_files)
    return toml_files
end

"""
    save_fleet_aircraft(ac, name::String, aircraft_outdir::String) -> String

Writes the optimized/sized aircraft `ac` to
`joinpath(aircraft_outdir, name * ".toml")` using
`TASOPT.save_aircraft_model(ac, path)`. Creates `aircraft_outdir` with
`mkpath` if needed. Returns the path written.
"""
function save_fleet_aircraft(ac, name::String, aircraft_outdir::String)
    mkpath(aircraft_outdir)
    outpath = joinpath(aircraft_outdir, name * ".toml")
    TASOPT.save_aircraft_model(ac, outpath)
    return outpath
end

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
    design_range_km, pfei_design)
    mkpath(fuel_burn_outdir)
    outpath = joinpath(fuel_burn_outdir, name * ".jld2")
    
    jldopen(outpath, "w") do file
        file["name"] = name
        file["range_km"] = range_km
        file["fuel_burn_MJ"] = fuel_burn_MJ
        file["converged"] = converged
        file["payload_N"] = payload_N
        file["payload_fraction"] = payload_fraction
        file["design_range_km"] = design_range_km
        file["pfei_design"] = pfei_design
    end
    
    return outpath
end

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
function append_index_row!(index_csv_path::String, row::Dict)
    # Fixed column order per spec
    column_order = [
        "name", "input_toml", "architecture", "fuel_type", "converged",
        "pfei_design_kJ_per_kg_km", "design_range_km", "payload_capacity_N",
        "payload_fraction_used", "output_toml_path", "fuel_burn_jld2_path",
        "opt_return_code", "n_envelope_points", "n_envelope_converged",
        "error_message"
    ]
    
    # Check if file exists (to decide whether to write header)
    file_exists = isfile(index_csv_path)
    
    open(index_csv_path, "a") do file
        # Write header on first call
        if !file_exists
            header = join(column_order, ",")
            write(file, header * "\n")
        end
        
        # Extract and sanitize values in fixed column order
        values = String[]
        for col in column_order
            val = get(row, col, "")
            
            # Convert to string if not already
            val_str = if val isa AbstractString
                val
            elseif val isa Bool
                string(val)
            elseif val === nothing || val == ""
                ""
            else
                string(val)
            end
            
            # Strip embedded newlines, carriage returns, and commas
            val_str = replace(val_str, r"[\r\n,]" => " ")
            
            push!(values, val_str)
        end
        
        # Write row
        row_str = join(values, ",")
        write(file, row_str * "\n")
    end
end

#=
EXAMPLE USAGE (not executed):

    inputs_dir = "example/fleet/inputs"
    aircraft_outdir = "example/fleet/outputs/aircraft"
    fuel_burn_outdir = "example/fleet/outputs/fuel_burn"
    index_csv_path = "example/fleet/outputs/index.csv"

    # Find all input aircraft
    input_files = discover_input_files(inputs_dir)

    # For each aircraft, after optimization and envelope evaluation:
    ac = ...  # optimized aircraft from optimize_aircraft!
    name = "my_aircraft_001"
    
    # Save optimized aircraft
    ac_path = save_fleet_aircraft(ac, name, aircraft_outdir)
    
    # Save fuel-burn envelope
    envelope_result = evaluate_fuel_burn_envelope(ac; payload_fraction=0.8)
    env_path = save_fleet_envelope(name, fuel_burn_outdir;
        range_km=envelope_result.range_km,
        fuel_burn_MJ=envelope_result.fuel_burn_MJ,
        converged=envelope_result.converged,
        payload_N=envelope_result.payload_N,
        payload_fraction=0.8,
        design_range_km=envelope_result.design_range_km,
        pfei_design=ac.parm[imPFEI, 1]
    )
    
    # Append index row
    append_index_row!(index_csv_path, Dict(
        "name" => name,
        "input_toml" => input_file,
        "architecture" => "TF",
        "fuel_type" => "JetA",
        "converged" => "true",
        "pfei_design_kJ_per_kg_km" => ac.parm[imPFEI, 1],
        "design_range_km" => ac.parg[igRange] / 1e3,
        "payload_capacity_N" => ac.parg[igWpaymax],
        "payload_fraction_used" => 0.8,
        "output_toml_path" => ac_path,
        "fuel_burn_jld2_path" => env_path,
        "opt_return_code" => "SUCCESS",
        "n_envelope_points" => length(envelope_result.range_km),
        "n_envelope_converged" => sum(envelope_result.converged),
        "error_message" => ""
    ))
=#
