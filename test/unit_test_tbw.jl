# Logging is not a test dependency, so use the copy in Base
const CoreLogging = Base.CoreLogging

# Smoke test: sizing the default aircraft at high wing aspect ratio must either
# converge, stop with a "max iterations" warning, or fail with a specific diagnostic.
# Bare DomainErrors and other uncontrolled crashes are regressions.

# Messages of the guards that are allowed to stop sizing
const clean_failures = (
    ArgumentError => r"negative tail area",
    ErrorException => r"cap sizing infeasible",
)

"Size the default aircraft at aspect ratio `AR`; return `(outcome, detail)`."
function size_at_aspect_ratio(AR)
    ac = CoreLogging.with_logger(CoreLogging.NullLogger()) do
        load_default_model()
    end
    ac.wing.layout.AR = AR

    logger = Test.TestLogger(min_level = CoreLogging.Warn)
    try
        CoreLogging.with_logger(logger) do
            size_aircraft!(ac, printiter = false)
        end
    catch e
        msg = sprint(showerror, e)
        for (type, pattern) in clean_failures
            e isa type && occursin(pattern, msg) && return (:clean_error, msg)
        end
        return (:uncontrolled_error, msg)
    end
    hit_itermax = any(r -> occursin("max iterations", r.message), logger.logs)
    return (hit_itermax ? :itermax_warning : :converged, "")
end

@testset "TBW aspect ratio smoke" verbose=true begin
    for AR in (10, 12, 14, 16, 18, 20)
        @testset "AR = $AR" begin
            outcome, detail = size_at_aspect_ratio(AR)
            @info "AR = $AR: $outcome" detail
            @test outcome in (:converged, :itermax_warning, :clean_error)
            outcome == :uncontrolled_error && @error "Uncontrolled failure at AR = $AR" detail
        end
    end
    # strut-on cases to be enabled later
    # ac = read strut-braced model (wing_config = "strut_braced"); ac.options.has_strut is then true
    # outcome, detail = size_at_aspect_ratio(AR)
end

"Default input text with the wing configuration lines removed, so `edit` controls them."
default_toml_without_wing_config() = replace(
    read(joinpath(TASOPT.__TASOPTroot__, "../example/defaults/default_input.toml"), String),
    r"^[ \t]*(wing_config|has_strut)[ \t]*=.*\n"m => "")

"Insert `line` right below the `[table]` header in `txt`."
insert_below_header(txt, table, line) = replace(txt, "[$table]" => "[$table]\n    $line"; count = 1)

"Default input text with `wing_config = \"config\"` set in [Options]."
toml_with_wing_config(config) =
    insert_below_header(default_toml_without_wing_config(), "Options", "wing_config = \"$config\"")

"Read an aircraft from TOML text; return the aircraft and the log records emitted while reading."
function read_aircraft_from_text(txt)
    logger = Test.TestLogger(min_level = CoreLogging.Info)
    ac = mktempdir() do dir
        path = joinpath(dir, "wing_config_test.toml")
        write(path, txt)
        CoreLogging.with_logger(logger) do
            read_aircraft_model(path)
        end
    end
    return ac, logger.logs
end

read_aircraft_with_wing_config(config) = first(read_aircraft_from_text(toml_with_wing_config(config)))

@testset "TBW wing configuration option" verbose=true begin
    @testset "WingConfig string names" begin
        @test string(WingConfig.Cantilever)  == "cantilever"
        @test string(WingConfig.StrutBraced) == "strut_braced"
        @test string(WingConfig.TrussBraced) == "truss_braced"
    end

    @testset "StrutForceModel string names" begin
        @test Set(Symbol.(instances(TASOPT.StrutForceModel.T))) == Set([:Compatibility, :Prescribed])
        @test string(TASOPT.StrutForceModel.Compatibility) == "compatibility"
        @test string(TASOPT.StrutForceModel.Prescribed)    == "prescribed"
    end

    ac_default = CoreLogging.with_logger(CoreLogging.NullLogger()) do
        load_default_model()
    end

    @testset "wing_config key selects the wing configuration" begin
        @test !ac_default.options.has_strut
        @test ac_default.options.opt_wing_config == WingConfig.Cantilever
        @test_throws Exception (ac_default.options.has_strut = true)

        for (name, config, strut) in (("cantilever",   WingConfig.Cantilever,  false),
                                      ("strut_braced", WingConfig.StrutBraced, true),
                                      ("truss_braced", WingConfig.TrussBraced, true))
            ac = read_aircraft_with_wing_config(name)
            @test ac.options.opt_wing_config == config
            @test ac.options.has_strut == strut
        end

        @test_throws ErrorException read_aircraft_with_wing_config("biplane")
    end

    @testset "legacy has_strut key is a deprecated alias" begin
        legacy_txt = insert_below_header(default_toml_without_wing_config(), "Wing", "has_strut = true")

        ac_legacy, logs = read_aircraft_from_text(legacy_txt)
        @test ac_legacy.options.opt_wing_config == WingConfig.StrutBraced
        @test any(r -> r.level == CoreLogging.Info && occursin("has_strut", r.message) &&
                       occursin("wing_config", r.message), logs)

        # wing_config takes precedence when both are given
        both_txt = insert_below_header(legacy_txt, "Options", "wing_config = \"cantilever\"")
        ac_both, _ = read_aircraft_from_text(both_txt)
        @test ac_both.options.opt_wing_config == WingConfig.Cantilever

        # the new key alone gives no deprecation message
        _, logs_new = read_aircraft_from_text(toml_with_wing_config("strut_braced"))
        @test !any(r -> occursin("has_strut", r.message), logs_new)
    end

    @testset "Wing no longer stores has_strut" begin
        @test !hasfield(TASOPT.structures.Wing, :has_strut)
    end

    @testset "save and read keep the wing configuration" begin
        for name in ("strut_braced", "truss_braced")
            ac = read_aircraft_with_wing_config(name)
            mktempdir() do dir
                path = joinpath(dir, "wing_config_roundtrip.toml")
                save_aircraft_model(ac, path)
                saved = TASOPT.TOML.parsefile(path)
                @test saved["Options"]["wing_config"] == name
                @test !haskey(saved["Wing"], "has_strut")

                ac_reread = CoreLogging.with_logger(CoreLogging.NullLogger()) do
                    read_aircraft_model(path)
                end
                @test ac_reread.options.opt_wing_config == ac.options.opt_wing_config
                @test ac_reread.wing.strut.z ≈ ac.wing.strut.z
            end
        end
    end

    @testset "Options summary and show report the wing configuration" begin
        options = ac_default.options
        summary_text = mktemp() do path, io
            redirect_stdout(() -> summary(options), io)
            close(io)
            read(path, String)
        end
        @test occursin("cantilever", summary_text)
        @test occursin("cantilever", sprint(show, options))

        options_strut = deepcopy(options)
        options_strut.opt_wing_config = WingConfig.StrutBraced
        @test occursin("strut_braced", sprint(show, options_strut))
    end
end

# Strut input keys in [Wing.Strut] and the `ac.wing.strut` fields they populate
const strut_block_fields = (
    z_fuselage_attach = :z,
    y_fuselage_attach = :y_root,
    thickness_to_chord = :thickness_to_chord,
    local_velocity_ratio = :local_velocity_ratio,
    wall_thickness_to_depth = :wall_thickness_to_depth,
    box_width_to_chord = :box_width_to_chord,
    web_to_box_height = :web_to_box_height,
    end_fixity_K = :end_fixity_K,
    truss_inertia_efficiency = :truss_inertia_efficiency,
    weight_frac_added = :weight_frac_added,
)

# Strut keys that used to sit directly in [Wing], and the [Wing.Strut] keys that replace them
const legacy_flat_strut_keys = (
    z_strut = :z_fuselage_attach,
    strut_toc = :thickness_to_chord,
    strut_local_velocity_ratio = :local_velocity_ratio,
)

# [Wing.Strut] body with values that differ from the defaults; distances carry units
const custom_strut_block = """
    z_fuselage_attach = "170 in"
    y_fuselage_attach = "1.5 ft"
    thickness_to_chord = 0.12
    local_velocity_ratio = 1.1
    wall_thickness_to_depth = 0.07
    box_width_to_chord = 0.45
    web_to_box_height = 0.7
    end_fixity_K = 0.8
    truss_inertia_efficiency = 0.6
    weight_frac_added = 0.5
"""

"Default input text without the strut inputs, whether given as flat [Wing] keys or as a [Wing.Strut] table."
function default_toml_without_strut_inputs(config)
    return replace(toml_with_wing_config(config),
        r"^[ \t]*(z_strut|strut_toc|strut_local_velocity_ratio)[ \t]*=.*\n"m => "",
        r"^\[Wing\.Strut\]\n(?:(?!\[).*\n)*"m => "")
end

"Default input text for `config` with a [Wing.Strut] table of `block` and flat [Wing] keys of `flat`."
function toml_with_strut_inputs(config; block = "", flat = "")
    txt = default_toml_without_strut_inputs(config)
    isempty(flat) || (txt = insert_below_header(txt, "Wing", flat))
    isempty(block) || (txt = replace(txt, "[Wing.Aero]" => "[Wing.Strut]\n$block\n[Wing.Aero]"; count = 1))
    return txt
end

mentions_legacy_strut_key(log, key) = log.level == CoreLogging.Info && occursin(string(key), log.message)

@testset "TBW strut TOML block" verbose=true begin
    custom_values = (
        z_fuselage_attach = TASOPT.Distance("170 in"),
        y_fuselage_attach = TASOPT.Distance("1.5 ft"),
        thickness_to_chord = 0.12,
        local_velocity_ratio = 1.1,
        wall_thickness_to_depth = 0.07,
        box_width_to_chord = 0.45,
        web_to_box_height = 0.7,
        end_fixity_K = 0.8,
        truss_inertia_efficiency = 0.6,
        weight_frac_added = 0.5,
    )

    @testset "[Wing.Strut] populates the strut fields" begin
        for config in ("strut_braced", "truss_braced")
            ac, logs = read_aircraft_from_text(toml_with_strut_inputs(config; block = custom_strut_block))
            for (key, field) in pairs(strut_block_fields)
                @test getfield(ac.wing.strut, field) == custom_values[key]
            end
            @test ac.wing.strut.z ≈ 170 * 0.0254
            @test ac.wing.strut.y_root ≈ 1.5 * 0.3048
            @test !any(r -> any(key -> mentions_legacy_strut_key(r, key), keys(legacy_flat_strut_keys)), logs)
        end
    end

    @testset "flat [Wing] strut keys are deprecated but honoured" begin
        flat = """
            z_strut = "160 in"
            strut_toc = 0.13
            strut_local_velocity_ratio = 1.2
        """
        ac, logs = read_aircraft_from_text(toml_with_strut_inputs("strut_braced"; flat))
        @test ac.wing.strut.z ≈ 160 * 0.0254
        @test ac.wing.strut.thickness_to_chord == 0.13
        @test ac.wing.strut.local_velocity_ratio == 1.2
        for key in keys(legacy_flat_strut_keys)
            @test any(r -> mentions_legacy_strut_key(r, key), logs)
        end

        # [Wing.Strut] wins over the flat keys; flat keys not in the block are still honoured
        partial_block = """
            z_fuselage_attach = "170 in"
            thickness_to_chord = 0.12
        """
        ac_both, _ = read_aircraft_from_text(toml_with_strut_inputs("strut_braced"; flat, block = partial_block))
        @test ac_both.wing.strut.z ≈ 170 * 0.0254
        @test ac_both.wing.strut.thickness_to_chord == 0.12
        @test ac_both.wing.strut.local_velocity_ratio == 1.2
    end

    @testset "default input gives the previous strut values" begin
        ac, logs = read_aircraft_from_text(toml_with_wing_config("strut_braced"))
        strut = ac.wing.strut
        @test strut.z ≈ 154 * 0.0254
        @test strut.thickness_to_chord == 0.15
        @test strut.local_velocity_ratio == 1.0
        @test strut.wall_thickness_to_depth == 0.05
        @test strut.box_width_to_chord == 0.5
        @test strut.web_to_box_height == 0.75
        @test strut.end_fixity_K == 1.0
        @test strut.truss_inertia_efficiency == 0.92
        @test strut.weight_frac_added == 0.60
        @test !any(r -> any(key -> mentions_legacy_strut_key(r, key), keys(legacy_flat_strut_keys)), logs)

        # A cantilever wing does not read the strut inputs
        _, logs_cantilever = read_aircraft_from_text(toml_with_wing_config("cantilever"))
        @test !any(r -> any(key -> mentions_legacy_strut_key(r, key), keys(legacy_flat_strut_keys)), logs_cantilever)
    end

    @testset "save and read reproduce the strut inputs exactly" begin
        for config in ("strut_braced", "truss_braced")
            ac = first(read_aircraft_from_text(toml_with_strut_inputs(config; block = custom_strut_block)))
            mktempdir() do dir
                path = joinpath(dir, "strut_roundtrip.toml")
                save_aircraft_model(ac, path)
                saved = TASOPT.TOML.parsefile(path)
                @test saved["Wing"]["Strut"] isa AbstractDict
                for key in keys(strut_block_fields)
                    @test haskey(saved["Wing"]["Strut"], string(key))
                end
                for key in keys(legacy_flat_strut_keys)
                    @test !haskey(saved["Wing"], string(key))
                end

                ac_reread, logs = read_aircraft_from_text(read(path, String))
                for field in values(strut_block_fields)
                    @test getfield(ac_reread.wing.strut, field) == getfield(ac.wing.strut, field)
                end
                @test !any(r -> any(key -> mentions_legacy_strut_key(r, key), keys(legacy_flat_strut_keys)), logs)
            end
        end
    end

    @testset "opt_strut_force selects the strut force model" begin
        # compatibility is the default, with or without a [Wing.Strut] table
        for txt in (toml_with_wing_config("strut_braced"),
                    toml_with_strut_inputs("strut_braced"),
                    toml_with_strut_inputs("strut_braced"; block = "thickness_to_chord = 0.12"))
            @test first(read_aircraft_from_text(txt)).wing.strut.opt_strut_force ==
                  TASOPT.StrutForceModel.Compatibility
        end

        for (name, model) in (("prescribed",    TASOPT.StrutForceModel.Prescribed),
                              ("compatibility", TASOPT.StrutForceModel.Compatibility))
            for config in ("strut_braced", "truss_braced")
                block = "opt_strut_force = \"$name\""
                ac = first(read_aircraft_from_text(toml_with_strut_inputs(config; block)))
                @test ac.wing.strut.opt_strut_force == model
            end
        end

        @test_throws Exception read_aircraft_from_text(
            toml_with_strut_inputs("strut_braced"; block = "opt_strut_force = \"rigid\""))
    end

    @testset "save and read reproduce opt_strut_force exactly" begin
        for model in (TASOPT.StrutForceModel.Prescribed, TASOPT.StrutForceModel.Compatibility)
            ac = read_aircraft_with_wing_config("strut_braced")
            ac.wing.strut.opt_strut_force = model
            mktempdir() do dir
                path = joinpath(dir, "strut_force_model_roundtrip.toml")
                save_aircraft_model(ac, path)
                saved = TASOPT.TOML.parsefile(path)
                @test saved["Wing"]["Strut"]["opt_strut_force"] == string(model)
                @test first(read_aircraft_from_text(read(path, String))).wing.strut.opt_strut_force == model
            end
        end
    end

    @testset "example inputs use the [Wing.Strut] table" begin
        example_dir = joinpath(TASOPT.__TASOPTroot__, "../example")
        # The ethanol inputs already fail to load for a reason unrelated to the strut
        files = [joinpath(root, f) for (root, _, fs) in walkdir(example_dir) for f in fs
                 if endswith(f, ".toml") && !occursin("ethanol", f)]
        @test !isempty(files)
        has_flat_keys(file) = occursin(r"^[ \t]*(z_strut|strut_toc|strut_local_velocity_ratio)[ \t]*="m, read(file, String))
        has_strut_table(file) = occursin(r"^\[Wing\.Strut\]"m, read(file, String))
        @test isempty(relpath.(filter(has_flat_keys, files), example_dir))
        @test isempty(relpath.(filter(!has_strut_table, files), example_dir))
    end
end

# Wing sizing load cases in [Wing.SizingLoads.<case>] tables

const sizing_load_case_names = (:pullup, :pushover, :pullup_light, :pushover_light,
                                :taxi, :gust_up_light, :gust_down_light)

"Parsed default input, with `edit!` applied to the Dict; return the aircraft and the log records."
function read_aircraft_from_default_dict(edit!)
    d = TASOPT.TOML.parsefile(joinpath(TASOPT.__TASOPTroot__, "../example/defaults/default_input.toml"))
    edit!(d)
    return read_aircraft_from_text(sprint(TASOPT.TOML.print, d))
end

"The [Wing.SizingLoads.`case`] table of the parsed input `d`, created if missing."
sizing_load_table(d, case) =
    get!(get!(d["Wing"], "SizingLoads", Dict{String,Any}()), string(case), Dict{String,Any}())

"The inputs of load case `name` that the TOML block carries."
function case_inputs(loads, name)
    case = getfield(loads, name)
    return (case.load_factor, case.mass_state, case.active, case.tail_load_scale)
end

mentions_nlift_deprecation(log) =
    log.level == CoreLogging.Info && occursin("Nlift", log.message) && occursin("deprecat", log.message)

@testset "TBW wing sizing loads TOML block" verbose=true begin
    WingMassState = TASOPT.structures.WingMassState
    WingSizingLoads = TASOPT.structures.WingSizingLoads

    @testset "default input gives the WingSizingLoads defaults" begin
        defaults = TASOPT.TOML.parsefile(joinpath(TASOPT.__TASOPTroot__, "../example/defaults/default_input.toml"))
        @test defaults["Wing"]["SizingLoads"] isa AbstractDict
        for name in sizing_load_case_names
            @test haskey(defaults["Wing"]["SizingLoads"], string(name))
        end

        ac, _ = read_aircraft_from_default_dict(d -> nothing)
        expected = WingSizingLoads()
        for name in sizing_load_case_names
            @test case_inputs(ac.wing.sizing_loads, name) == case_inputs(expected, name)
        end
    end

    @testset "templates without [Wing.SizingLoads] give the WingSizingLoads defaults" begin
        for file in ("default_regional.toml", "default_wide.toml")
            ac, _ = read_aircraft_from_text(read(joinpath(TASOPT.__TASOPTroot__, "../example/defaults", file), String))
            for name in sizing_load_case_names
                @test case_inputs(ac.wing.sizing_loads, name) == case_inputs(WingSizingLoads(), name)
            end
        end
    end

    @testset "[Wing.SizingLoads] populates the load cases" begin
        ac, _ = read_aircraft_from_default_dict() do d
            pushover = sizing_load_table(d, :pushover)
            pushover["active"] = true
            pushover["load_factor"] = -1.5
            pushover["mass_state"] = "light"
            taxi = sizing_load_table(d, :taxi)
            taxi["active"] = true
            taxi["tail_load_scale"] = 0.8
        end
        loads = ac.wing.sizing_loads

        @test loads.pushover.active
        @test loads.pushover.load_factor == -1.5
        @test loads.pushover.mass_state == WingMassState.Light
        @test loads.taxi.active
        @test loads.taxi.tail_load_scale == 0.8
        @test loads.taxi.load_factor == -2.0
        @test loads.taxi.mass_state == WingMassState.MTOW_MaxFuel

        # cases that the input does not touch keep their defaults
        for name in (:pullup, :pullup_light, :pushover_light, :gust_up_light, :gust_down_light)
            @test case_inputs(loads, name) == case_inputs(WingSizingLoads(), name)
        end

        # missing keys fall back to the defaults
        ac_missing, _ = read_aircraft_from_default_dict() do d
            delete!(sizing_load_table(d, :pushover_light), "load_factor")
            delete!(sizing_load_table(d, :pushover_light), "mass_state")
        end
        @test case_inputs(ac_missing.wing.sizing_loads, :pushover_light) ==
              case_inputs(WingSizingLoads(), :pushover_light)
    end

    @testset "save and read reproduce the load cases exactly" begin
        ac, _ = read_aircraft_from_default_dict() do d
            sizing_load_table(d, :pullup)["load_factor"] = 2.6
            sizing_load_table(d, :pushover)["active"] = true
            sizing_load_table(d, :pushover)["mass_state"] = "mzfw"
            sizing_load_table(d, :pullup_light)["active"] = true
            sizing_load_table(d, :pullup_light)["load_factor"] = 2.25
            sizing_load_table(d, :taxi)["active"] = true
            sizing_load_table(d, :taxi)["tail_load_scale"] = 0.75
            sizing_load_table(d, :gust_up_light)["mass_state"] = "DesignCruise"
            sizing_load_table(d, :gust_down_light)["load_factor"] = -0.5
        end

        mktempdir() do dir
            path = joinpath(dir, "sizing_loads_roundtrip.toml")
            save_aircraft_model(ac, path)
            saved = TASOPT.TOML.parsefile(path)

            @test !haskey(saved["Mission"], "Nlift")
            @test saved["Wing"]["SizingLoads"] isa AbstractDict
            for name in sizing_load_case_names
                table = saved["Wing"]["SizingLoads"][string(name)]
                for key in ("load_factor", "mass_state", "active", "tail_load_scale")
                    @test haskey(table, key)
                end
                for key in ("kind", "n_computed", "gust_velocity_EAS", "gust_speed_EAS")
                    @test !haskey(table, key)
                end
            end

            ac_reread, logs = read_aircraft_from_text(read(path, String))
            for name in sizing_load_case_names
                @test case_inputs(ac_reread.wing.sizing_loads, name) == case_inputs(ac.wing.sizing_loads, name)
            end
            @test !any(mentions_nlift_deprecation, logs)
        end
    end

    @testset "Mission.Nlift is a deprecated alias of the pull-up load factor" begin
        ac, logs = read_aircraft_from_default_dict() do d
            delete!(sizing_load_table(d, :pullup), "load_factor")
            d["Mission"]["Nlift"] = 2.7
        end
        @test ac.wing.sizing_loads.pullup.load_factor == 2.7
        @test ac.parg[igNlift] == 0.0
        @test any(mentions_nlift_deprecation, logs)

        # the table wins when both are given
        ac_both, _ = read_aircraft_from_default_dict() do d
            sizing_load_table(d, :pullup)["load_factor"] = 2.4
            d["Mission"]["Nlift"] = 2.7
        end
        @test ac_both.wing.sizing_loads.pullup.load_factor == 2.4
    end

    # The user input with the `active` keys removed, so that only the defaults decide
    function without_active_keys!(d)
        for name in sizing_load_case_names
            delete!(sizing_load_table(d, name), "active")
        end
    end
    braced_extra_cases = (:pushover, :pullup_light, :pushover_light, :taxi)
    active_cases(ac) = Set(n for n in sizing_load_case_names if getfield(ac.wing.sizing_loads, n).active)

    @testset "braced wings activate the extra load cases by default" begin
        for config in ("strut_braced", "truss_braced")
            ac, _ = read_aircraft_from_default_dict() do d
                d["Options"]["wing_config"] = config
                without_active_keys!(d)
            end
            @test active_cases(ac) == Set([:pullup, braced_extra_cases...])
        end

        # default_tbw.toml has no [Wing.SizingLoads] table
        ac_tbw, _ = read_aircraft_from_text(read(joinpath(TASOPT.__TASOPTroot__,
                                                          "../example/defaults/default_tbw.toml"), String))
        @test active_cases(ac_tbw) == Set([:pullup, braced_extra_cases...])

        # a cantilever wing keeps only the pull-up case
        ac_cantilever, _ = read_aircraft_from_default_dict(without_active_keys!)
        @test active_cases(ac_cantilever) == Set([:pullup])
    end

    @testset "an explicit active key wins over the braced defaults" begin
        ac, _ = read_aircraft_from_default_dict() do d
            d["Options"]["wing_config"] = "strut_braced"
            without_active_keys!(d)
            sizing_load_table(d, :taxi)["active"] = false
            sizing_load_table(d, :gust_up_light)["active"] = true
        end
        @test active_cases(ac) == Set([:pullup, :pushover, :pullup_light, :pushover_light, :gust_up_light])

        # the cases of a braced wing survive save and read
        mktempdir() do dir
            path = joinpath(dir, "braced_cases_roundtrip.toml")
            save_aircraft_model(ac, path)
            @test active_cases(first(read_aircraft_from_text(read(path, String)))) == active_cases(ac)
        end
    end

    @testset "unknown mass_state is an error" begin
        @test_throws Exception read_aircraft_from_default_dict() do d
            sizing_load_table(d, :pushover)["mass_state"] = "HalfFull"
        end
    end
end

const braced_min_cap_thickness = 1.5e-3 # [m]
const braced_min_web_thickness = 1.0e-3 # [m]
const wing_config_names = ("cantilever", "strut_braced", "truss_braced")

"Aircraft read from the default input for `config`, with the minimum gauge keys removed from the user input and `edit!` applied to the Dict."
function read_aircraft_with_gauge_inputs(edit!, config)
    return first(read_aircraft_from_default_dict() do d
        d["Options"]["wing_config"] = config
        delete!(d["Wing"], "min_cap_thickness")
        delete!(d["Wing"], "min_web_thickness")
        edit!(d)
    end)
end

read_aircraft_with_gauge_inputs(config) = read_aircraft_with_gauge_inputs(d -> nothing, config)

"`(min_cap_thickness, min_web_thickness)` of a wing section."
gauge_floors(section) = (section.min_cap_thickness, section.min_web_thickness)

@testset "TBW minimum wing gauge" verbose=true begin
    @testset "wing sections default to no floor" begin
        @test gauge_floors(TASOPT.structures.WingSection()) == (0.0, 0.0)
        ac = CoreLogging.with_logger(CoreLogging.NullLogger()) do
            load_default_model()
        end
        @test gauge_floors(ac.wing.inboard) == (0.0, 0.0)
        @test gauge_floors(ac.wing.outboard) == (0.0, 0.0)
    end

    @testset "defaults depend on the wing configuration" begin
        for config in ("strut_braced", "truss_braced")
            ac = read_aircraft_with_gauge_inputs(config)
            @test gauge_floors(ac.wing.inboard)  == (braced_min_cap_thickness, braced_min_web_thickness)
            @test gauge_floors(ac.wing.outboard) == (braced_min_cap_thickness, braced_min_web_thickness)
            # tails have no floor
            for tail in (ac.htail, ac.vtail)
                @test gauge_floors(tail.inboard) == (0.0, 0.0)
                @test gauge_floors(tail.outboard) == (0.0, 0.0)
            end
        end

        ac_cantilever = read_aircraft_with_gauge_inputs("cantilever")
        @test gauge_floors(ac_cantilever.wing.inboard) == (0.0, 0.0)
        @test gauge_floors(ac_cantilever.wing.outboard) == (0.0, 0.0)

        # default_tbw.toml is a braced wing
        ac_tbw, _ = read_aircraft_from_text(read(joinpath(TASOPT.__TASOPTroot__,
                                                          "../example/defaults/default_tbw.toml"), String))
        @test all(>(0), gauge_floors(ac_tbw.wing.inboard))
        @test gauge_floors(ac_tbw.wing.outboard) == gauge_floors(ac_tbw.wing.inboard)
    end

    @testset "explicit [Wing] keys set both sections and win over the defaults" begin
        for config in wing_config_names
            ac = read_aircraft_with_gauge_inputs(config) do d
                d["Wing"]["min_cap_thickness"] = 0.002
                d["Wing"]["min_web_thickness"] = 0.0007
            end
            @test gauge_floors(ac.wing.inboard)  == (0.002, 0.0007)
            @test gauge_floors(ac.wing.outboard) == (0.002, 0.0007)
        end

        # distance strings use the units of the other [Wing] distances
        ac = read_aircraft_with_gauge_inputs("strut_braced") do d
            d["Wing"]["min_cap_thickness"] = "0.1 in"
            d["Wing"]["min_web_thickness"] = "0.05 in"
        end
        @test collect(gauge_floors(ac.wing.inboard))  ≈ [0.1 * 0.0254, 0.05 * 0.0254]
        @test collect(gauge_floors(ac.wing.outboard)) ≈ [0.1 * 0.0254, 0.05 * 0.0254]

        # an explicit zero switches the braced default off; a single key leaves the other at its default
        ac_zero = read_aircraft_with_gauge_inputs("strut_braced") do d
            d["Wing"]["min_cap_thickness"] = 0.0
        end
        @test gauge_floors(ac_zero.wing.inboard)  == (0.0, braced_min_web_thickness)
        @test gauge_floors(ac_zero.wing.outboard) == (0.0, braced_min_web_thickness)

        ac_web = read_aircraft_with_gauge_inputs("truss_braced") do d
            d["Wing"]["min_web_thickness"] = 0.0
        end
        @test gauge_floors(ac_web.wing.inboard)  == (braced_min_cap_thickness, 0.0)
        @test gauge_floors(ac_web.wing.outboard) == (braced_min_cap_thickness, 0.0)
    end

    @testset "save and read reproduce the floors exactly" begin
        edits = (d -> nothing, # the defaults of the configuration
                 d -> (d["Wing"]["min_cap_thickness"] = 0.0021; d["Wing"]["min_web_thickness"] = 0.00073),
                 d -> (d["Wing"]["min_cap_thickness"] = 0.0; d["Wing"]["min_web_thickness"] = 0.0))
        for config in wing_config_names, edit! in edits
            ac = read_aircraft_with_gauge_inputs(edit!, config)
            mktempdir() do dir
                path = joinpath(dir, "min_gauge_roundtrip.toml")
                save_aircraft_model(ac, path)
                saved = TASOPT.TOML.parsefile(path)
                @test saved["Wing"]["min_cap_thickness"] == ac.wing.inboard.min_cap_thickness
                @test saved["Wing"]["min_web_thickness"] == ac.wing.inboard.min_web_thickness

                ac_reread, _ = read_aircraft_from_text(read(path, String))
                @test gauge_floors(ac_reread.wing.inboard)  == gauge_floors(ac.wing.inboard)
                @test gauge_floors(ac_reread.wing.outboard) == gauge_floors(ac.wing.outboard)
            end
        end
    end
end

@testset "TBW reference input" verbose=true begin
    defaults_dir = joinpath(TASOPT.__TASOPTroot__, "../example/defaults")
    tbw_path = joinpath(defaults_dir, "default_tbw.toml")
    @test isfile(tbw_path)

    if isfile(tbw_path)
        ac, logs = read_aircraft_from_text(read(tbw_path, String))
        wing = ac.wing

        @testset "reads as a strut-braced high wing with fuel in the wing" begin
            @test ac.options.opt_wing_config == WingConfig.StrutBraced
            @test ac.options.has_strut
            @test !any(r -> occursin("has_strut", r.message) ||
                            any(key -> mentions_legacy_strut_key(r, key), keys(legacy_flat_strut_keys)), logs)
            @test wing.layout.AR ≈ 20
            @test wing.layout.sweep ≈ 20
            @test 0.09 ≤ wing.inboard.cross_section.thickness_to_chord ≤ 0.11
            @test 0.09 ≤ wing.outboard.cross_section.thickness_to_chord ≤ 0.11
            @test 0.55 ≤ wing.layout.ηs ≤ 0.65
            @test wing.layout.z > 0
            @test ac.options.opt_fuel == TASOPT.FuelType.JetA
            @test ac.options.has_wing_fuel
        end

        @testset "strut inputs are physically sensible" begin
            @test 0 < wing.strut.thickness_to_chord < 1
            @test wing.strut.local_velocity_ratio > 0
            @test 0 < wing.strut.wall_thickness_to_depth < 1
            @test 0 < wing.strut.web_to_box_height < 1
            @test wing.strut.end_fixity_K > 0
            @test wing.strut.truss_inertia_efficiency > 0
        end

        @testset "unrelated inputs stay as in default_input.toml" begin
            ac_default = CoreLogging.with_logger(CoreLogging.NullLogger()) do
                read_aircraft_model(joinpath(defaults_dir, "default_input.toml"))
            end
            @test ac.fuselage.layout.cross_section.radius == ac_default.fuselage.layout.cross_section.radius
            @test ac.fuselage.layout.l_cabin_cylinder == ac_default.fuselage.layout.l_cabin_cylinder
            @test ac.parm[imRange, 1] == ac_default.parm[imRange, 1]
            @test ac.parm[imWpay, 1] == ac_default.parm[imWpay, 1]
            @test ac.htail.layout.AR == ac_default.htail.layout.AR
        end

        @testset "save and read reproduce the wing and strut inputs exactly" begin
            mktempdir() do dir
                path = joinpath(dir, "tbw_roundtrip.toml")
                save_aircraft_model(ac, path)
                ac_reread, _ = read_aircraft_from_text(read(path, String))
                @test ac_reread.options.opt_wing_config == ac.options.opt_wing_config
                @test ac_reread.wing.layout.AR == wing.layout.AR
                @test ac_reread.wing.layout.sweep == wing.layout.sweep
                for field in values(strut_block_fields)
                    @test getfield(ac_reread.wing.strut, field) == getfield(wing.strut, field)
                end
            end
        end
    end
end

@testset "TBW strut struct" verbose=true begin
    # Default aircraft with the strut switched on, sized with the legacy strut model:
    # prescribed strut force and only the pull-up case
    function read_strut_aircraft()
        ac = read_aircraft_with_wing_config("strut_braced")
        ac.wing.strut.opt_strut_force = TASOPT.StrutForceModel.Prescribed
        for name in (:pushover, :pullup_light, :pushover_light, :taxi, :gust_up_light, :gust_down_light)
            getfield(ac.wing.sizing_loads, name).active = false
        end
        return ac
    end

    ac = read_strut_aircraft()
    size_aircraft!(ac, printiter = false)
    wing = ac.wing
    strut = wing.strut

    @testset "Defaults of the extended Strut" begin
        default_strut = TASOPT.structures.Strut()
        @test default_strut.material.name == "TASOPT-Al"
        @test default_strut.y_root == 0
        @test default_strut.wall_thickness_to_depth == 0.05
        @test default_strut.box_width_to_chord == 0.5
        @test default_strut.web_to_box_height == 0.75
        @test default_strut.end_fixity_K == 1.0
        @test default_strut.truss_inertia_efficiency == 0.92
        @test default_strut.dihedral == 0
        @test default_strut.axial_force_compression == 0
        @test default_strut.buckling_load == 0
        @test default_strut.EI_min == 0
        @test default_strut.sizing_criterion === :none
        @test default_strut.CL == 0
        @test default_strut.lift_fraction == 0
        @test default_strut.weight_frac_added == 0.60
        @test default_strut.area == 0
        @test default_strut.axial_force == 0
        @test default_strut.opt_strut_force == TASOPT.StrutForceModel.Compatibility
    end

    @testset "JuryStrut defaults" begin
        jury = TASOPT.structures.JuryStrut()
        @test jury.material.name == "TASOPT-Al"
        @test jury.weight_frac_added == 0.60
        @test jury.eta_strut == 0.5
        for field in (:eta_wing, :length, :area, :axial_force, :buckling_load, :weight, :dxW)
            @test getfield(jury, field) == 0
        end
    end

    @testset "Strut-on sizing matches the legacy results" begin
        # Reference values from the strut model before axial_force became a force
        @test wing.weight             ≈ 104682.33132567903  rtol=1e-10
        @test strut.weight            ≈ 1544.7255555813356  rtol=1e-10
        @test ac.parm[imPFEI]         ≈ 0.9311145282956051  rtol=1e-10
        @test ac.parg[igWMTO]         ≈ 850444.9472357186   rtol=1e-10
        @test strut.chord             ≈ 0.5946475578487581  rtol=1e-10
        @test strut.S                 ≈ 6.538342414355748   rtol=1e-10
        @test strut.area              ≈ 0.005304085770832381 rtol=1e-10
    end

    @testset "Prescribed strut records the pull-up case" begin
        sl = wing.sizing_loads
        @test Set(keys(sl.results)) == Set([:pullup])
        @test sl.governing_strut == :pullup
        @test sl.results[:pullup].strut_force > 0
        @test sl.results[:pullup].P_axial_inboard < 0
    end

    @testset "Strut length, force and weight" begin
        layout = wing.layout
        cosΛ = cosd(layout.sweep)
        length_3d = sqrt(strut.z^2 + (0.5 * layout.span * (layout.ηs - layout.ηo) / cosΛ)^2)
        @test strut.length > 0
        @test strut.length ≈ length_3d rtol=1e-6

        # Tension force [N] is the area times the allowable stress
        @test strut.axial_force > 0
        @test strut.axial_force ≈ strut.area * strut.material.σmax * ac.parg[igsigfac] rtol=1e-10

        # Two struts of uniform area
        @test strut.weight ≈ 2 * strut.material.ρ * TASOPT.gee * strut.area * strut.length rtol=1e-6
        @test strut.S ≈ 2 * strut.chord * strut.length rtol=1e-10
    end

    @testset "wall_thickness_to_depth sets the strut chord" begin
        @test strut.wall_thickness_to_depth == 0.05
        @test strut.chord ≈ sqrt(0.5 * strut.area / (0.05 * strut.thickness_to_chord)) rtol=1e-10

        ac_thicker = read_strut_aircraft()
        ac_thicker.wing.strut.wall_thickness_to_depth = 0.08
        size_aircraft!(ac_thicker, printiter = false)
        strut_thicker = ac_thicker.wing.strut
        @test strut_thicker.chord ≈ sqrt(0.5 * strut_thicker.area / (0.08 * strut_thicker.thickness_to_chord)) rtol=1e-10
        @test strut_thicker.chord < strut.chord
    end
end

# Sizing with the strut force from the compatibility of the wing and strut deflections

@testset "strut-braced sizing with the compatibility strut model" verbose=true begin
    ac = read_aircraft_with_wing_config("strut_braced")
    @test ac.wing.strut.opt_strut_force == TASOPT.StrutForceModel.Compatibility
    ac.wing.layout.ηs = 0.9   # the legacy strut model sizes a nearly weightless wing for this break location
    CoreLogging.with_logger(CoreLogging.NullLogger()) do
        size_aircraft!(ac, printiter = false)
    end
    wing, strut, sl = ac.wing, ac.wing.strut, ac.wing.sizing_loads

    @testset "the wing is no longer free at a far outboard strut" begin
        # the legacy model gives about 3 kN for this wing (TBW_PROPOSAL.md, section 2.1)
        @test isfinite(wing.weight)
        @test wing.weight > 20e3
    end

    @testset "strut tension and area" begin
        @test strut.axial_force > 0
        @test strut.area > 0
        @test strut.weight > 0
        # the tension area is a lower bound: Euler buckling may enlarge the strut beyond it
        @test strut.axial_force <= strut.area * strut.material.σmax * ac.parg[igsigfac] * (1 + 1e-10)
    end

    @testset "strut compression is recorded" begin
        compressive = (:pushover, :pushover_light, :taxi)
        forces = [r.strut_force for r in values(sl.results)]
        @test sl.governing_strut_compression in compressive
        @test sl.results[sl.governing_strut_compression].strut_force == minimum(forces)
        @test strut.axial_force_compression < 0

        # tension and compression are scaled from the strut length in the plane of the strut to the real length
        @test strut.axial_force_compression / strut.axial_force ≈ minimum(forces) / maximum(forces) rtol = 1e-8
    end

    @testset "the sizing cases are evaluated and the strut has a governing case" begin
        active = Set(name for name in keys(sl.results))
        @test Set([:pullup, :pushover, :pullup_light, :pushover_light, :taxi]) == active
        @test sl.governing_strut in active
        @test sl.results[sl.governing_strut].strut_force == maximum(r.strut_force for r in values(sl.results))
        @test all(r -> abs(r.M_inboard_extreme) >= abs(r.M_root), values(sl.results))
    end
end

@testset "strut compression of the Prescribed strut model and of a cantilever wing" verbose=true begin
    @testset "Prescribed strut force has no compression" begin
        ac = read_aircraft_with_wing_config("strut_braced")
        ac.wing.strut.opt_strut_force = TASOPT.StrutForceModel.Prescribed
        ac.wing.layout.ηs = 0.9
        CoreLogging.with_logger(CoreLogging.NullLogger()) do
            size_aircraft!(ac, printiter = false)
        end
        strut, sl = ac.wing.strut, ac.wing.sizing_loads
        # the push-over, light and taxi cases are active and compressive, but they do not enter the strut
        @test Set(keys(sl.results)) == Set([:pullup, :pushover, :pullup_light, :pushover_light, :taxi])
        @test sl.results[:pushover].strut_force < 0
        @test strut.axial_force_compression == 0
        @test strut.axial_force > 0
        @test strut.axial_force ≈ strut.area * strut.material.σmax * ac.parg[igsigfac] rtol = 1e-10
    end

    @testset "cantilever wing has no strut compression" begin
        ac = CoreLogging.with_logger(CoreLogging.NullLogger()) do
            load_default_model()
        end
        size_aircraft!(ac, printiter = false)
        @test ac.wing.strut.axial_force_compression == 0
        @test ac.wing.sizing_loads.governing_strut_compression == :none
    end
end

# Stress and safety factors of the structural materials are kept in StructuralAlloy

@testset "structural material factors round trip through save and read" verbose=true begin
    @testset "StructuralAlloy keeps its factors" begin
        alloy = StructuralAlloy("TASOPT-Al", max_avg_stress = 1.2, safety_factor = 1.7)
        @test alloy.max_avg_stress == 1.2
        @test alloy.safety_factor == 1.7
        @test alloy.σmax ≈ alloy.YTS / 1.2 / 1.7
        @test StructuralAlloy("TASOPT-Al").max_avg_stress == 1.1
        @test StructuralAlloy("TASOPT-Al").safety_factor == 1.5
    end

    material_getters = (
        caps = ac -> ac.wing.inboard.caps.material,
        webs = ac -> ac.wing.inboard.webs.material,
        strut = ac -> ac.wing.strut.material,
        skin = ac -> ac.fuselage.skin.material,
        cone = ac -> ac.fuselage.cone.material,
        bending = ac -> ac.fuselage.bendingmaterial_h.material,
        floor = ac -> ac.fuselage.floor.material,
    )
    # distinct factors per material, which no combined value could reproduce
    factors = Dict(name => (1.2 + 0.01i, 1.7 + 0.01i) for (i, name) in enumerate(keys(material_getters)))

    ac, _ = read_aircraft_from_default_dict() do d
        for (name, (max_avg, safety)) in factors
            d["Structures"]["$(name)_max_avg_stress"] = max_avg
            d["Structures"]["$(name)_safety_factor"] = safety
        end
    end

    @testset "the input keys set the factors of each material" begin
        for (name, getter) in pairs(material_getters)
            material = getter(ac)
            max_avg, safety = factors[name]
            @test material.max_avg_stress == max_avg
            @test material.safety_factor == safety
            @test material.σmax ≈ material.YTS / max_avg / safety
        end
    end

    @testset "save and read reproduce the factors exactly" begin
        mktempdir() do dir
            path = joinpath(dir, "material_factors_roundtrip.toml")
            save_aircraft_model(ac, path)
            saved = TASOPT.TOML.parsefile(path)["Structures"]
            ac_reread, _ = read_aircraft_from_text(read(path, String))
            for (name, getter) in pairs(material_getters)
                max_avg, safety = factors[name]
                @test saved["$(name)_max_avg_stress"] == max_avg
                @test saved["$(name)_safety_factor"] == safety
                @test getter(ac_reread).max_avg_stress == max_avg
                @test getter(ac_reread).safety_factor == safety
                @test getter(ac_reread).σmax ≈ getter(ac).σmax rtol = 1e-14
            end
        end
    end
end

# Euler buckling sizing of the strut: A = max(yield area, buckling area), with the yield area from the larger of
# the tension and the compression

@testset "strut Euler buckling sizing" verbose=true begin
    st = TASOPT.structures
    E = TASOPT.structures.Strut().material.E

    @testset "stiffness and buckling area helpers match the hand calculation" begin
        strut = st.Strut()
        strut.thickness_to_chord = 0.15
        strut.wall_thickness_to_depth = 0.05
        strut.end_fixity_K = 1.0
        strut.truss_inertia_efficiency = 1.0   # ideal section, so that k is the hand value below
        k = 0.15 / (8 * 0.05)   # I_min = k A^2 for two caps of width c, thickness toh*toc*c and depth toc*c
        l, Pc = 16.0, -3.0e5
        fu = strut.material.safety_factor   # ultimate factor on the compression

        @test st.strut_bending_stiffness(strut, 0.004) ≈ E * k * 0.004^2 rtol = 1e-12
        @test st.strut_bending_stiffness(strut, 0.0) == 0

        area = st.strut_buckling_area(strut, Pc, l)
        @test area ≈ sqrt(fu * abs(Pc) * l^2 / (pi^2 * E * k)) rtol = 1e-12
        # round trip: the Euler load of that area is the ultimate compression
        @test pi^2 * st.strut_bending_stiffness(strut, area) / (strut.end_fixity_K * l)^2 ≈ fu * abs(Pc) rtol = 1e-10

        strut.end_fixity_K = 2.0
        @test pi^2 * st.strut_bending_stiffness(strut, area) / (strut.end_fixity_K * l)^2 ≈ fu * abs(Pc) / 4 rtol = 1e-10
        @test st.strut_buckling_area(strut, Pc, l) ≈ 2 * area rtol = 1e-12

        @test st.strut_buckling_area(strut, 0.0, l) == 0.0
    end

    @testset "the strut safety factor is the ultimate factor of the buckling area" begin
        strut = st.Strut()
        strut.thickness_to_chord = 0.15
        strut.wall_thickness_to_depth = 0.05
        area_15 = st.strut_buckling_area(strut, -3.0e5, 16.0)
        strut.material = StructuralAlloy("TASOPT-Al"; safety_factor = 2.0)
        @test st.strut_buckling_area(strut, -3.0e5, 16.0) ≈ sqrt(2.0 / 1.5) * area_15 rtol = 1e-12
    end

    @testset "truss_inertia_efficiency scales the bending stiffness and the buckling area" begin
        strut = st.Strut()
        strut.thickness_to_chord = 0.15
        strut.wall_thickness_to_depth = 0.05
        strut.truss_inertia_efficiency = 1.0   # reference: ideal section
        EI_1 = st.strut_bending_stiffness(strut, 0.004)
        area_1 = st.strut_buckling_area(strut, -3.0e5, 16.0)
        @test EI_1 ≈ E * 0.15 / (8 * 0.05) * 0.004^2 rtol = 1e-12

        strut.truss_inertia_efficiency = 0.5
        @test st.strut_bending_stiffness(strut, 0.004) ≈ 0.5 * EI_1 rtol = 1e-12
        @test st.strut_buckling_area(strut, -3.0e5, 16.0) ≈ sqrt(2) * area_1 rtol = 1e-12
    end

    # Size the strut-braced aircraft with the compatibility strut force; `edit!` changes the strut first.
    # Probe values at K = 1 (toc = 0.15, toh = 0.05): tension area 4.67e-3 m^2, buckling area 2.23e-2 m^2
    # (ratio 4.8), P_c = -3.34e5 N, l = 15.9 m. The ratio is proportional to K.
    function size_braced(edit! = strut -> nothing)
        ac = read_aircraft_with_wing_config("strut_braced")
        ac.wing.layout.ηs = 0.9
        edit!(ac.wing.strut)
        CoreLogging.with_logger(CoreLogging.NullLogger()) do
            size_aircraft!(ac, printiter = false)
        end
        return ac
    end
    # the strut yields in the larger of the tension and the compression
    yield_area(ac) = max(ac.wing.strut.axial_force, abs(ac.wing.strut.axial_force_compression)) /
                     (ac.wing.strut.material.σmax * ac.parg[igsigfac])

    @testset "sized strut satisfies yield and buckling and records the governing criterion" begin
        ac = size_braced()
        strut = ac.wing.strut
        A_buckling = st.strut_buckling_area(strut, strut.axial_force_compression, strut.length)

        @test strut.axial_force_compression < 0
        @test strut.area >= yield_area(ac) * (1 - 1e-8)
        @test strut.area >= A_buckling * (1 - 1e-8)
        @test strut.area ≈ max(yield_area(ac), A_buckling) rtol = 1e-8
        @test strut.sizing_criterion === (A_buckling > yield_area(ac) ? :buckling : :tension)
        @test strut.EI_min ≈ st.strut_bending_stiffness(strut, strut.area) rtol = 1e-10
        @test strut.buckling_load ≈ pi^2 * strut.EI_min / (strut.end_fixity_K * strut.length)^2 rtol = 1e-10
        @test strut.buckling_load >= strut.material.safety_factor * abs(strut.axial_force_compression) * (1 - 1e-8)
        @test strut.area >= abs(strut.axial_force_compression) / (strut.material.σmax * ac.parg[igsigfac]) * (1 - 1e-8)
        @test strut.area >= strut.axial_force / (strut.material.σmax * ac.parg[igsigfac]) * (1 - 1e-8)
    end

    @testset "a slender strut is governed by buckling" begin
        ac = size_braced(strut -> strut.end_fixity_K = 2.0)   # buckling area about 9.6 times the tension area
        strut = ac.wing.strut
        @test strut.sizing_criterion === :buckling
        @test strut.area > 1.1 * yield_area(ac)
        @test strut.area ≈ st.strut_buckling_area(strut, strut.axial_force_compression, strut.length) rtol = 1e-8
        @test strut.buckling_load ≈ strut.material.safety_factor * abs(strut.axial_force_compression) rtol = 1e-8

        # a larger area makes a heavier strut
        stocky = size_braced(strut -> strut.end_fixity_K = 0.1).wing.strut
        @test strut.weight > stocky.weight
    end

    @testset "a stocky strut is governed by tension" begin
        ac = size_braced(strut -> strut.end_fixity_K = 0.1)   # buckling area about 0.48 times the tension area
        strut = ac.wing.strut
        @test strut.sizing_criterion === :tension
        @test strut.area ≈ yield_area(ac) rtol = 1e-8
        @test strut.area > 1.1 * st.strut_buckling_area(strut, strut.axial_force_compression, strut.length)
        @test strut.axial_force ≈ strut.area * strut.material.σmax * ac.parg[igsigfac] rtol = 1e-10
        @test strut.buckling_load > strut.material.safety_factor * abs(strut.axial_force_compression)
    end

    @testset "the compressive yield sets the area when the compression exceeds the tension" begin
        # |P_c| is about 0.35 times the tension for the default sizing cases, so a heavier push-over case
        # (load factor -4, |P_c| about 1.4 times the tension at K = 0.1) is used to make the compression larger
        ac = read_aircraft_with_wing_config("strut_braced")
        ac.wing.layout.ηs = 0.9
        ac.wing.strut.end_fixity_K = 0.1
        ac.wing.sizing_loads.pushover.load_factor = -4.0
        CoreLogging.with_logger(CoreLogging.NullLogger()) do
            size_aircraft!(ac, printiter = false)
        end
        strut = ac.wing.strut
        σ_allowable = strut.material.σmax * ac.parg[igsigfac]
        @test abs(strut.axial_force_compression) > strut.axial_force
        @test strut.sizing_criterion === :tension
        @test strut.area ≈ abs(strut.axial_force_compression) / σ_allowable rtol = 1e-8
        @test strut.area > strut.axial_force / σ_allowable
        @test strut.area > st.strut_buckling_area(strut, strut.axial_force_compression, strut.length)
    end

    @testset "the strut safety factor of the input sets the ultimate factor" begin
        ac = size_braced(strut -> strut.material = StructuralAlloy("TASOPT-Al", max_avg_stress = 1.1, safety_factor = 2.0))
        strut = ac.wing.strut
        @test strut.material.safety_factor == 2.0
        @test strut.sizing_criterion === :buckling
        @test strut.buckling_load >= 2.0 * abs(strut.axial_force_compression) * (1 - 1e-8)
        @test strut.buckling_load ≈ 2.0 * abs(strut.axial_force_compression) rtol = 1e-8
    end

    @testset "a lower truss_inertia_efficiency enlarges a strut governed by buckling" begin
        full = size_braced().wing.strut
        ac_half = size_braced(strut -> strut.truss_inertia_efficiency = 0.5)
        half = ac_half.wing.strut
        @test full.sizing_criterion === :buckling
        @test half.sizing_criterion === :buckling
        @test half.area > full.area
        @test half.weight > full.weight
        @test half.EI_min ≈ st.strut_bending_stiffness(half, half.area) rtol = 1e-10
        @test half.buckling_load ≈ half.material.safety_factor * abs(half.axial_force_compression) rtol = 1e-8
    end

    @testset "Prescribed strut force is sized by tension only" begin
        ac = size_braced(strut -> strut.opt_strut_force = TASOPT.StrutForceModel.Prescribed)
        strut = ac.wing.strut
        @test strut.axial_force_compression == 0
        @test strut.sizing_criterion === :tension
        @test strut.area ≈ strut.axial_force / (strut.material.σmax * ac.parg[igsigfac]) rtol = 1e-10
        @test strut.EI_min ≈ st.strut_bending_stiffness(strut, strut.area) rtol = 1e-10
    end

    @testset "cantilever wing has no buckling data" begin
        ac = CoreLogging.with_logger(CoreLogging.NullLogger()) do
            load_default_model()
        end
        size_aircraft!(ac, printiter = false)
        strut = ac.wing.strut
        @test strut.EI_min == 0
        @test strut.buckling_load == 0
        @test strut.sizing_criterion === :none
    end
end

# Truss-braced wing: a vertical jury strut at the fraction eta_strut of the strut length, from the fuselage end,
# holds the main strut laterally, so the main strut buckles over the longer of its two parts

"Default input text for `config` with a [Wing.Jury] table of `block` (no table if `block` is empty)."
function toml_with_jury_inputs(config, block = "")
    txt = replace(toml_with_wing_config(config), r"^\[Wing\.Jury\]\n(?:(?!\[).*\n)*"m => "")
    isempty(block) && return txt
    return replace(txt, "[Wing.Aero]" => "[Wing.Jury]\n$block\n[Wing.Aero]"; count = 1)
end

@testset "jury strut sizing" verbose=true begin
    st = TASOPT.structures
    jury_load_fraction = 0.02   # jury load as a fraction of the strut compression --- temporary assumption
    jury_default_fields = (:eta_wing, :length, :area, :axial_force, :buckling_load, :weight, :dxW)

    @testset "strut_buckling_length is the longer part of the strut cut at the jury" begin
        L = 15.0
        @test st.strut_buckling_length(L, 0.5) ≈ 0.5 * L rtol = 1e-12
        @test st.strut_buckling_length(L, 0.3) ≈ 0.7 * L rtol = 1e-12
        @test st.strut_buckling_length(L, 0.8) ≈ 0.8 * L rtol = 1e-12
        @test st.strut_buckling_length(L, nothing) == L
    end

    # Size a wing with the strut attached at ηs = 0.7 (the default ηs does not converge with a strut);
    # `eta_strut` sets the jury of a truss-braced wing
    function size_wing(config, eta_strut = 0.5)
        ac = read_aircraft_with_wing_config(config)
        ac.wing.layout.ηs = 0.7
        config == "truss_braced" && (ac.wing.jury.eta_strut = eta_strut)
        CoreLogging.with_logger(CoreLogging.NullLogger()) do
            size_aircraft!(ac, printiter = false)
        end
        return ac
    end
    truss = Dict(η => size_wing("truss_braced", η) for η in (0.3, 0.5, 0.8))
    braced = size_wing("strut_braced")

    @testset "the main strut buckles over the longer part, whatever the jury position" begin
        for (η, ac) in truss
            strut = ac.wing.strut
            l_eff = max(η, 1 - η) * strut.length
            @test strut.buckling_load ≈ pi^2 * strut.EI_min / (strut.end_fixity_K * l_eff)^2 rtol = 1e-8
            @test strut.EI_min ≈ st.strut_bending_stiffness(strut, strut.area) rtol = 1e-10
            @test strut.buckling_load >= strut.material.safety_factor * abs(strut.axial_force_compression) * (1 - 1e-8)
        end
    end

    @testset "the jury geometry follows the strut" begin
        for (η, ac) in truss
            layout = ac.wing.layout
            jury = ac.wing.jury
            @test jury.eta_strut == η
            @test jury.length ≈ ac.wing.strut.z * (1 - η) rtol = 1e-8
            @test jury.eta_wing ≈ layout.ηo + η * (layout.ηs - layout.ηo) rtol = 1e-8
        end
    end

    @testset "the jury is sized by Euler buckling under a fraction of the strut compression" begin
        for (η, ac) in truss
            strut, jury = ac.wing.strut, ac.wing.jury
            @test jury.axial_force ≈ -jury_load_fraction * abs(strut.axial_force_compression) rtol = 1e-8

            # a pinned member of the jury material with the section idealisation of the main strut
            member = deepcopy(strut)
            member.material = jury.material
            member.end_fixity_K = 1.0
            A_buckling = st.strut_buckling_area(member, jury.axial_force, jury.length)
            A_yield = abs(jury.axial_force) / (jury.material.σmax * ac.parg[igsigfac])
            @test jury.area > 0
            @test jury.area >= A_yield * (1 - 1e-8)
            @test jury.area ≈ max(A_yield, A_buckling) rtol = 1e-8
            @test jury.buckling_load ≈ pi^2 * st.strut_bending_stiffness(member, jury.area) / jury.length^2 rtol = 1e-8
            @test jury.buckling_load >= jury.material.safety_factor * abs(jury.axial_force) * (1 - 1e-8)
            if A_buckling > A_yield
                @test jury.buckling_load ≈ jury.material.safety_factor * abs(jury.axial_force) rtol = 1e-8
            end
            @test jury.weight ≈ 2 * jury.material.ρ * TASOPT.gee * jury.area * jury.length rtol = 1e-8
        end
    end

    @testset "the jury lets the strut be lighter than a strut governed by buckling over its full length" begin
        @test braced.wing.strut.sizing_criterion === :buckling
        for (η, ac) in truss
            @test ac.wing.strut.area < braced.wing.strut.area
            @test ac.wing.strut.sizing_criterion in (:buckling, :tension)
        end
    end

    @testset "a strut-braced wing has no jury and buckles over the full strut length" begin
        strut, jury = braced.wing.strut, braced.wing.jury
        @test strut.buckling_load ≈ pi^2 * strut.EI_min / (strut.end_fixity_K * strut.length)^2 rtol = 1e-10
        @test jury.eta_strut == 0.5
        for field in jury_default_fields
            @test getfield(jury, field) == 0
        end
    end

    @testset "a cantilever wing has no jury" begin
        ac = CoreLogging.with_logger(CoreLogging.NullLogger()) do
            load_default_model()
        end
        size_aircraft!(ac, printiter = false)
        for field in jury_default_fields
            @test getfield(ac.wing.jury, field) == 0
        end
    end

    @testset "[Wing.Jury] sets eta_strut of a truss-braced wing" begin
        # the jury attachment is 0.5 without a table
        @test first(read_aircraft_from_text(toml_with_jury_inputs("truss_braced"))).wing.jury.eta_strut == 0.5
        @test first(read_aircraft_from_text(toml_with_jury_inputs("truss_braced", "eta_strut = 0.3"))).wing.jury.eta_strut == 0.3

        # a strut-braced wing does not read it
        ac = first(read_aircraft_from_text(toml_with_jury_inputs("strut_braced", "eta_strut = 0.3")))
        @test ac.wing.jury.eta_strut == 0.5
    end

    @testset "save and read reproduce eta_strut of the jury exactly" begin
        ac = first(read_aircraft_from_text(toml_with_jury_inputs("truss_braced", "eta_strut = 0.3")))
        mktempdir() do dir
            path = joinpath(dir, "jury_roundtrip.toml")
            save_aircraft_model(ac, path)
            saved = TASOPT.TOML.parsefile(path)
            @test saved["Wing"]["Jury"]["eta_strut"] == 0.3
            @test first(read_aircraft_from_text(read(path, String))).wing.jury.eta_strut == 0.3
        end

        # only a truss-braced wing has a jury table
        mktempdir() do dir
            path = joinpath(dir, "no_jury.toml")
            save_aircraft_model(read_aircraft_with_wing_config("strut_braced"), path)
            @test !haskey(TASOPT.TOML.parsefile(path)["Wing"], "Jury")
        end
    end
end

@testset "Tail has no strut" verbose=true begin
    @testset "Tail struct" begin
        @test !hasfield(TASOPT.structures.Tail, :has_strut)
        @test !hasfield(TASOPT.structures.Tail, :strut)
    end

    @testset "Wing keeps its strut" begin
        @test hasfield(TASOPT.structures.Wing, :strut)
    end

    @testset "Default aircraft tails" begin
        ac = CoreLogging.with_logger(CoreLogging.NullLogger()) do
            load_default_model()
        end
        size_aircraft!(ac, printiter = false)
        @test !hasproperty(ac.htail, :strut)
        @test !hasproperty(ac.vtail, :strut)
        @test ac.wing.strut isa TASOPT.structures.Strut
    end
end

# Wing sizing load cases: container types and their defaults on `Wing`

@testset "TBW wing sizing load cases" verbose=true begin
    # local bindings, so the names cannot clash with anything TASOPT exports
    WingMassState = TASOPT.structures.WingMassState
    WingLoadKind = TASOPT.structures.WingLoadKind
    WingLoadCase = TASOPT.structures.WingLoadCase
    WingCaseLoads = TASOPT.structures.WingCaseLoads
    WingSizingLoads = TASOPT.structures.WingSizingLoads

    case_names = (:pullup, :pushover, :pullup_light, :pushover_light,
                  :taxi, :gust_up_light, :gust_down_light)

    @testset "enum members" begin
        @test Set(Symbol.(instances(WingMassState.T))) ==
              Set([:DesignCruise, :MTOW_MaxFuel, :MZFW, :Light])
        @test Set(Symbol.(instances(WingLoadKind.T))) == Set([:Maneuver, :Gust, :Ground])
    end

    @testset "wing sizing load cases defaults" begin
        loads = WingSizingLoads()

        @test Tuple(n for n in fieldnames(WingSizingLoads)
                    if fieldtype(WingSizingLoads, n) === WingLoadCase) == case_names

        # only the legacy pull-up case is active
        for name in case_names
            @test getfield(loads, name).active == (name == :pullup)
        end

        @test loads.pullup.kind == WingLoadKind.Maneuver
        @test loads.pullup.load_factor == 3.0
        @test loads.pullup.mass_state == WingMassState.DesignCruise

        expected = (pushover        = (WingLoadKind.Maneuver, -1.0, WingMassState.MTOW_MaxFuel),
                    pullup_light    = (WingLoadKind.Maneuver,  3.0, WingMassState.Light),
                    pushover_light  = (WingLoadKind.Maneuver, -1.0, WingMassState.Light),
                    taxi            = (WingLoadKind.Ground,   -2.0, WingMassState.MTOW_MaxFuel),
                    gust_up_light   = (WingLoadKind.Gust,      1.0, WingMassState.Light),
                    gust_down_light = (WingLoadKind.Gust,     -1.0, WingMassState.Light))
        for (name, (kind, load_factor, mass_state)) in pairs(expected)
            case = getfield(loads, name)
            @test case.kind == kind
            @test case.load_factor == load_factor
            @test case.mass_state == mass_state
        end

        for name in case_names
            case = getfield(loads, name)
            @test case.gust_velocity_EAS == 15.24
            @test case.gust_speed_EAS == 0.0
            @test case.tail_load_scale == 1.0
            @test case.n_computed == 0.0
        end

        @test isempty(loads.results)
        @test loads.results isa Dict{Symbol,WingCaseLoads}
        @test loads.governing_root == :pullup
        @test loads.governing_break == :pullup
        @test loads.governing_strut == :none
        @test loads.governing_strut_compression == :none
    end

    @testset "WingCaseLoads holds the section loads and the strut loads" begin
        @test Set(fieldnames(WingCaseLoads)) == Set([:S_root, :M_root, :S_break, :M_break,
                                                     :M_inboard_extreme, :P_axial_inboard, :strut_force])
        case_loads = WingCaseLoads()
        for name in fieldnames(WingCaseLoads)
            @test getfield(case_loads, name) === 0.0
        end
        @test WingCaseLoads(M_root = 5.0e6).M_root == 5.0e6
        @test WingCaseLoads(strut_force = 2.0e5).strut_force == 2.0e5
    end

    @testset "Wing carries its own sizing loads" begin
        @test fieldtype(TASOPT.structures.Wing, :sizing_loads) === WingSizingLoads

        wing_a = TASOPT.structures.Wing()
        wing_b = TASOPT.structures.Wing()
        @test wing_a.sizing_loads isa WingSizingLoads
        @test wing_a.sizing_loads !== wing_b.sizing_loads

        wing_a.sizing_loads.pullup.load_factor = 2.5
        wing_a.sizing_loads.pushover.active = true
        wing_a.sizing_loads.results[:pullup] = WingCaseLoads(M_root = 1.0)
        wing_a.sizing_loads.governing_root = :taxi

        @test wing_b.sizing_loads.pullup.load_factor == 3.0
        @test !wing_b.sizing_loads.pushover.active
        @test isempty(wing_b.sizing_loads.results)
        @test wing_b.sizing_loads.governing_root == :pullup
        @test WingSizingLoads().pullup.load_factor == 3.0
    end

    @testset "sizing loads of the default aircraft" begin
        ac = Base.CoreLogging.with_logger(Base.CoreLogging.NullLogger()) do
            load_default_model()
        end
        @test ac.wing.sizing_loads isa WingSizingLoads
        @test ac.wing.sizing_loads.pullup.active
        @test count(n -> getfield(ac.wing.sizing_loads, n).active, case_names) == 1
    end

    @testset "sizing responds to the pull-up load factor" begin
        quiet_sized_wing_weight(load_factor) = CoreLogging.with_logger(CoreLogging.NullLogger()) do
            ac = load_default_model()
            ac.wing.sizing_loads.pullup.load_factor = load_factor
            size_aircraft!(ac, printiter = false)
            ac.wing.weight
        end
        @test quiet_sized_wing_weight(2.5) < quiet_sized_wing_weight(3.0)
    end
end

# Wing mass states: lift weight and fuel split of each sizing case

@testset "wing mass states" verbose=true begin
    WingMassState = TASOPT.structures.WingMassState
    wing_mass_state = TASOPT.structures.wing_mass_state

    weights = (W_cruise = 900e3, W_MTO = 1000e3, W_fuel = 200e3, W_reserve = 30e3,
               W_empty = 500e3, W_fuel_cap = 300e3, W_fuel_at_MTO = 150e3, r_fuel_design = 0.6)

    # expected (W_lift, r_fuel_wing, W_fuel_fuse) with fuel in the fuselage
    fuselage_fuel = (
        (WingMassState.DesignCruise, 900e3, 0.0, 200e3),
        (WingMassState.MTOW_MaxFuel, 1000e3, 0.0, 150e3),
        (WingMassState.MZFW, 830e3, 0.0, 30e3),
        (WingMassState.Light, 530e3, 0.0, 30e3),
    )
    # expected with fuel in the wing
    wing_fuel = (
        (WingMassState.DesignCruise, 900e3, 0.6, 0.0),
        (WingMassState.MTOW_MaxFuel, 1000e3, 0.5, 0.0),
        (WingMassState.MZFW, 830e3, 0.1, 0.0),
        (WingMassState.Light, 530e3, 0.1, 0.0),
    )

    @testset "fuel in the wing" begin
        for (state, W_lift, r_fuel_wing, W_fuel_fuse) in wing_fuel
            result = wing_mass_state(state, true; weights...)
            @test result.W_lift ≈ W_lift
            @test result.r_fuel_wing ≈ r_fuel_wing
            @test result.W_fuel_fuse == W_fuel_fuse
        end
    end

    @testset "fuel in the fuselage" begin
        for (state, W_lift, r_fuel_wing, W_fuel_fuse) in fuselage_fuel
            result = wing_mass_state(state, false; weights...)
            @test result.W_lift ≈ W_lift
            @test result.r_fuel_wing == r_fuel_wing
            @test result.W_fuel_fuse ≈ W_fuel_fuse
        end
    end

    @testset "no wing tank capacity without fuel in the wing" begin
        for state in instances(WingMassState.T)
            result = wing_mass_state(state, false; weights..., W_fuel_cap = 0.0)
            @test all(isfinite, values(result))
            @test result.r_fuel_wing == 0.0
        end
    end

    @testset "MTOW wing fuel fraction is capped at one" begin
        result = wing_mass_state(WingMassState.MTOW_MaxFuel, true; weights..., W_fuel_at_MTO = 400e3)
        @test result.r_fuel_wing == 1.0
        @test result.W_fuel_fuse == 0.0
    end
end

# Wing root loading of each sizing case

@testset "wing sizing case root loading" verbose=true begin
    wing_case_root_loading = TASOPT.wing_case_root_loading
    wing_loading = TASOPT.wing_loading

    wing = CoreLogging.with_logger(CoreLogging.NullLogger()) do
        load_default_model().wing
    end
    wing.layout.span = 60.0   # the span is only set when the aircraft is sized
    cases = wing.sizing_loads
    rclt, rcls = 1.0, 1.0
    W_lift, Lhtail, n_pullup = 5.0e5, 2.0e4, 3.0
    root_loading(case) = wing_case_root_loading(case, wing, rclt, rcls, W_lift, Lhtail, n_pullup)

    @testset "pull-up case equals wing_loading at the pull-up load factor" begin
        loads = root_loading(cases.pullup)
        @test loads.Nload == 3.0
        @test loads.po == wing_loading(wing, rclt, rcls, n_pullup, W_lift, Lhtail)
    end

    @testset "push-over case scales the tail load with the load factor" begin
        loads = root_loading(cases.pushover)
        @test loads.Nload == -1.0
        @test loads.po == wing_loading(wing, rclt, rcls, -1.0, W_lift, (-1.0 / n_pullup) * Lhtail)
    end

    @testset "tail_load_scale scales the tail load of the case" begin
        half = deepcopy(cases.pushover)
        half.tail_load_scale = 0.5
        @test root_loading(half).po ==
              wing_loading(wing, rclt, rcls, -1.0, W_lift, 0.5 * (-1.0 / n_pullup) * Lhtail)
        @test root_loading(half).po != root_loading(cases.pushover).po
    end

    @testset "ground case has no wing lift" begin
        loads = root_loading(cases.taxi)
        @test loads.Nload == -2.0
        @test loads.po == 0.0
    end

    @testset "gust cases are not supported yet" begin
        @test_throws ArgumentError root_loading(cases.gust_up_light)
    end
end

# Envelope of the wing sizing cases

@testset "wing sizing case envelope" verbose=true begin
    Cantilever = TASOPT.structures.Cantilever
    wing_case_loads = TASOPT.wing_case_loads
    wing_case_root_loading = TASOPT.wing_case_root_loading

    base_wing = CoreLogging.with_logger(CoreLogging.NullLogger()) do
        load_default_model().wing
    end
    base_wing.layout.span = 60.0   # the span is only set when the aircraft is sized
    # section weights and moments are zero until the wing is sized
    base_wing.inboard.weight, base_wing.inboard.dyW = 4.0e4, 6.0e4
    base_wing.outboard.weight, base_wing.outboard.dyW = 2.0e4, 8.0e4
    # sentinel section loads, which the evaluation must leave untouched
    base_wing.inboard.max_shear_load, base_wing.inboard.moment = 111.0, 222.0
    base_wing.outboard.max_shear_load, base_wing.outboard.moment = 333.0, 444.0

    masses = (W_cruise = 5.0e5, W_MTO = 5.5e5, W_fuel = 1.0e5, W_reserve = 2.0e4,
              W_empty = 3.0e5, W_fuel_cap = 1.5e5, W_fuel_at_MTO = 8.0e4, r_fuel_design = 0.5)
    rclt, rcls, Lhtail = 1.0, 1.0, 2.0e4
    gammat, gammas, We, dyeinn = 0.5, 0.75, 1.0e4, 5.0
    # lift weight of each mass state used below, from `masses`
    W_lift_cruise, W_lift_MTO, W_lift_light = 5.0e5, 5.5e5, 3.2e5

    evaluate(wing) = TASOPT.evaluate_wing_sizing_cases!(wing, masses, true, rclt, rcls, Lhtail,
                                                        gammat, gammas, We, dyeinn)
    # loads of a case computed directly: one inboard engine per wing, on a copy of the wing
    direct_loads(wing, Nload, po) = wing_case_loads(Cantilever(), deepcopy(wing), po, gammat, gammas,
                                                    Nload, We, 0, 0.0, 1, dyeinn)
    maneuver_loads(wing, case, W_lift) = begin
        Nload, po = wing_case_root_loading(case, wing, rclt, rcls, W_lift, Lhtail, 3.0)
        direct_loads(wing, Nload, po)
    end
    section_loads(wing) = (wing.inboard.max_shear_load, wing.inboard.moment,
                           wing.outboard.max_shear_load, wing.outboard.moment)
    same_loads(a, b) = all(getfield(a, f) == getfield(b, f) for f in fieldnames(typeof(a)))

    @testset "default: only the pull-up case" begin
        wing = deepcopy(base_wing)
        envelope = evaluate(wing)
        sl = wing.sizing_loads
        expected = maneuver_loads(wing, sl.pullup, W_lift_cruise)

        @test Set(keys(sl.results)) == Set([:pullup])
        @test same_loads(sl.results[:pullup], expected)
        @test sl.governing_root == :pullup
        @test sl.governing_break == :pullup
        @test envelope == (S_root = abs(expected.S_root), M_root = abs(expected.M_root),
                           S_break = abs(expected.S_break), M_break = abs(expected.M_break))
        @test sl.pullup.n_computed == 3.0
        @test section_loads(wing) == section_loads(base_wing)
    end

    @testset "maneuver and ground cases" begin
        wing = deepcopy(base_wing)
        sl = wing.sizing_loads
        for name in (:pushover, :pullup_light, :pushover_light, :taxi)
            getfield(sl, name).active = true
        end
        envelope = evaluate(wing)
        active = (:pullup, :pushover, :pullup_light, :pushover_light, :taxi)

        @test Set(keys(sl.results)) == Set(active)
        @test same_loads(sl.results[:pullup], maneuver_loads(wing, sl.pullup, W_lift_cruise))
        @test same_loads(sl.results[:pushover], maneuver_loads(wing, sl.pushover, W_lift_MTO))
        @test same_loads(sl.results[:pullup_light], maneuver_loads(wing, sl.pullup_light, W_lift_light))
        @test same_loads(sl.results[:pushover_light], maneuver_loads(wing, sl.pushover_light, W_lift_light))
        @test same_loads(sl.results[:taxi], direct_loads(wing, -2.0, 0.0))
        @test [getfield(sl, name).n_computed for name in active] == [3.0, -1.0, 3.0, -1.0, -2.0]
        @test section_loads(wing) == section_loads(base_wing)

        # the cases differ, so the governing cases are not trivially the first one
        M_root = [abs(sl.results[name].M_root) for name in active]
        M_break = [abs(sl.results[name].M_break) for name in active]
        @test length(unique(M_root)) == length(active)
        @test sl.governing_root == active[argmax(M_root)]
        @test sl.governing_break == active[argmax(M_break)]
        governing_root, governing_break = sl.results[sl.governing_root], sl.results[sl.governing_break]
        @test envelope == (S_root = abs(governing_root.S_root), M_root = abs(governing_root.M_root),
                           S_break = abs(governing_break.S_break), M_break = abs(governing_break.M_break))

        # evaluating again gives the same results
        results = deepcopy(sl.results)
        @test evaluate(wing) == envelope
        @test Set(keys(sl.results)) == Set(keys(results))
        @test all(same_loads(sl.results[name], results[name]) for name in active)
    end

    @testset "active gust cases are skipped" begin
        wing = deepcopy(base_wing)
        sl = wing.sizing_loads
        sl.gust_up_light.active = true
        envelope = evaluate(wing)

        @test Set(keys(sl.results)) == Set([:pullup])
        @test sl.gust_up_light.n_computed == 0.0
        @test envelope.M_root == abs(sl.results[:pullup].M_root)
    end

    @testset "stale results of inactive cases are removed" begin
        wing = deepcopy(base_wing)
        sl = wing.sizing_loads
        sl.results[:taxi] = TASOPT.structures.WingCaseLoads(M_root = 1.0e12)
        evaluate(wing)
        @test Set(keys(sl.results)) == Set([:pullup])
    end

    @testset "no active case" begin
        wing = deepcopy(base_wing)
        sl = wing.sizing_loads
        sl.pullup.active = false
        sl.results[:pullup] = TASOPT.structures.WingCaseLoads(M_root = 1.0)
        @test evaluate(wing) === nothing
        @test isempty(sl.results)
    end
end

# Section loads of a braced wing

@testset "braced wing section loads" verbose=true begin
    st = TASOPT.structures
    Braced, Cantilever = st.Braced, st.Cantilever
    StrutForceModel = TASOPT.StrutForceModel
    wing_case_loads = TASOPT.wing_case_loads
    wing_case_root_loading = TASOPT.wing_case_root_loading

    base_wing = CoreLogging.with_logger(CoreLogging.NullLogger()) do
        load_default_model().wing
    end
    base_wing.layout.span = 60.0   # the span is only set when the aircraft is sized
    base_wing.inboard.weight, base_wing.inboard.dyW = 4.0e4, 6.0e4
    base_wing.outboard.weight, base_wing.outboard.dyW = 2.0e4, 8.0e4
    base_wing.strut.z = 154 * in_to_m
    base_wing.strut.area = 5.0e-3
    # stiffness of the previous sizing iteration
    base_wing.inboard.EI[1], base_wing.outboard.EI[1] = 5.0e9, 2.5e9

    span, ηs, ηo = base_wing.layout.span, base_wing.layout.ηs, base_wing.layout.ηo
    z = base_wing.strut.z
    L = 0.5 * span * (ηs - ηo)    # length of the inboard bay
    ls = sqrt(z^2 + L^2)          # strut length in the plane of the strut
    gammat, gammas, We, dyeinn = 0.5, 0.75, 1.0e4, 5.0
    W_lift = (pullup = 5.0e5, pushover = 5.5e5, pullup_light = 3.2e5, pushover_light = 3.2e5, taxi = 5.5e5)
    Lhtail = 2.0e4

    "A copy of the base wing with the strut force model `model` and `edit!` applied."
    function wing_with(model; edit! = w -> nothing)
        wing = deepcopy(base_wing)
        wing.strut.opt_strut_force = model
        edit!(wing)
        return wing
    end
    "Root loading `(Nload, po)` of the named case"
    root_loading(wing, name) = wing_case_root_loading(getfield(wing.sizing_loads, name), wing,
                                                      1.0, 1.0, W_lift[name], Lhtail, 3.0)
    "Braced loads of the named case, computed on a copy of `wing`"
    function braced_loads(wing, name; neinn = 1)
        Nload, po = root_loading(wing, name)
        return wing_case_loads(Braced(), deepcopy(wing), po, gammat, gammas, Nload, We, 0, 0.0, neinn, dyeinn)
    end
    case_names = (:pullup, :pushover, :pullup_light, :pushover_light, :taxi)

    # A wing without inboard weight and engines, loaded by the lift only
    lift_only(model) = wing_with(model; edit! = w -> (w.inboard.weight = 0.0; w.inboard.dyW = 0.0))

    @testset "prescribed strut force is the legacy rule" begin
        wing = lift_only(StrutForceModel.Prescribed)
        Nload, po = root_loading(wing, :pullup)
        loads = braced_loads(wing, :pullup; neinn = 0)

        R = (po * span / 12) * (ηs - ηo) * (1 + 2gammas) + loads.S_break
        @test loads.strut_force ≈ R * ls / z rtol = 1e-12
        @test loads.P_axial_inboard ≈ -loads.strut_force * L / ls rtol = 1e-12
        # the legacy force makes the root moment equal the moment at the strut attachment
        @test loads.M_root ≈ loads.M_break rtol = 1e-9

        # outboard loads are those of the cantilever
        cantilever = wing_case_loads(Cantilever(), deepcopy(wing), po, gammat, gammas, Nload, We, 0, 0.0, 0, dyeinn)
        @test loads.S_break == cantilever.S_break
        @test loads.M_break == cantilever.M_break
    end

    @testset "compatibility strut force follows the strut_force_compatibility formula" begin
        wing = lift_only(StrutForceModel.Compatibility)
        Nload, po = root_loading(wing, :pullup)
        loads = braced_loads(wing, :pullup; neinn = 0)

        # unbraced bay: clamped at the root, linear spanload from po to po*gammas, outboard loads at the break
        outboard = wing_case_loads(Cantilever(), deepcopy(wing), po, gammat, gammas, Nload, We, 0, 0.0, 0, dyeinn)
        function M0(x)
            u = L - x
            a, c = po * (1 - (1 - gammas) * x / L), po * (1 - gammas) / L
            return outboard.M_break + outboard.S_break * u + a * u^2 / 2 - c * u^3 / 3
        end
        EI(x) = base_wing.inboard.EI[1] + (base_wing.outboard.EI[1] - base_wing.inboard.EI[1]) * x / L
        sinθ = z / ls
        f_strut = ls / (base_wing.strut.material.E * base_wing.strut.area * sinθ^2)
        R = st.strut_force_compatibility(M0, EI, L, f_strut)

        @test loads.strut_force ≈ R * ls / z rtol = 1e-9
        @test loads.P_axial_inboard ≈ -loads.strut_force * L / ls rtol = 1e-9
        @test loads.M_root ≈ M0(0) - R * L rtol = 1e-9
        @test loads.S_root ≈ outboard.S_break + po * L * (1 + gammas) / 2 - R rtol = 1e-9
    end

    @testset "strut stiffness sets the share of the load the strut carries" begin
        pullup(area) = braced_loads(wing_with(StrutForceModel.Compatibility;
                                              edit! = w -> w.strut.area = area), :pullup)
        flexible, nominal, stiff = pullup(1e-9), pullup(5e-3), pullup(1.0)

        @test 0 < abs(flexible.strut_force) < 1e-3 * nominal.strut_force
        @test 0 < nominal.strut_force < stiff.strut_force
        # M_root = M0(0) - R L falls as the strut takes more of the load
        @test stiff.M_root < nominal.M_root < flexible.M_root

        # a strut that carries nothing leaves the root loads of the cantilever
        wing = wing_with(StrutForceModel.Compatibility)
        Nload, po = root_loading(wing, :pullup)
        cantilever = wing_case_loads(Cantilever(), deepcopy(wing), po, gammat, gammas, Nload, We, 0, 0.0, 1, dyeinn)
        @test flexible.M_root ≈ cantilever.M_root rtol = 1e-4
        @test flexible.S_root ≈ cantilever.S_root rtol = 1e-4
    end

    @testset "load sign decides tension or compression of the strut" begin
        for model in (StrutForceModel.Compatibility, StrutForceModel.Prescribed)
            wing = wing_with(model)
            pullup, pushover = braced_loads(wing, :pullup), braced_loads(wing, :pushover)
            @test pullup.strut_force > 0
            @test pullup.P_axial_inboard < 0
            @test pushover.strut_force < 0
            @test pushover.P_axial_inboard > 0
        end
    end

    @testset "the inboard moment extreme is at least the root moment" begin
        for model in (StrutForceModel.Compatibility, StrutForceModel.Prescribed), name in case_names
            loads = braced_loads(wing_with(model), name)
            @test abs(loads.M_inboard_extreme) >= abs(loads.M_root)
            @test isfinite(loads.M_inboard_extreme) && loads.M_inboard_extreme != 0
        end
    end

    @testset "evaluate_wing_sizing_cases! records the strut loads of a braced wing" begin
        masses = (W_cruise = 5.0e5, W_MTO = 5.5e5, W_fuel = 1.0e5, W_reserve = 2.0e4,
                  W_empty = 3.0e5, W_fuel_cap = 1.5e5, W_fuel_at_MTO = 8.0e4, r_fuel_design = 0.5)
        evaluate(wing; kwargs...) = TASOPT.evaluate_wing_sizing_cases!(wing, masses, true, 1.0, 1.0, Lhtail,
                                                                       gammat, gammas, We, dyeinn; kwargs...)
        section_loads(w) = (w.inboard.max_shear_load, w.inboard.moment, w.outboard.max_shear_load, w.outboard.moment)

        for model in (StrutForceModel.Compatibility, StrutForceModel.Prescribed)
            wing = wing_with(model)
            sl = wing.sizing_loads
            for name in (:pushover, :pullup_light, :pushover_light, :taxi)
                getfield(sl, name).active = true
            end
            sections_before = section_loads(wing)
            envelope = evaluate(wing; has_strut = true)
            results = sl.results

            @test Set(keys(results)) == Set(case_names)
            @test section_loads(wing) == sections_before
            forces = Dict(name => results[name].strut_force for name in case_names)
            @test forces[:pullup] > 0 && forces[:pushover] < 0
            @test results[:pullup].P_axial_inboard ≈ -forces[:pullup] * L / ls rtol = 1e-12

            # the case with the largest tension governs the strut
            @test sl.governing_strut in case_names
            @test sl.governing_strut == case_names[argmax([forces[name] for name in case_names])]
            @test envelope.T_strut ≈ maximum(values(forces)) rtol = 1e-12

            # the case with the most negative strut force governs the compression of the strut
            @test envelope.T_strut_compression ≈ minimum(values(forces)) rtol = 1e-12
            @test envelope.T_strut_compression < 0
            @test sl.governing_strut_compression in case_names
            @test sl.governing_strut_compression == case_names[argmin([forces[name] for name in case_names])]
            @test abs(envelope.P_axial) ≈ maximum(abs(results[name].P_axial_inboard) for name in case_names) rtol = 1e-12

            # the inboard section is sized for the largest moment and shear over the cases
            @test envelope.M_root ≈ maximum(max(abs(results[name].M_root), abs(results[name].M_inboard_extreme))
                                            for name in case_names)
            @test envelope.S_root ≈ maximum(max(abs(results[name].S_root), abs(results[name].S_break))
                                            for name in case_names)
        end
    end

    @testset "a cantilever wing has no strut loads" begin
        wing = deepcopy(base_wing)
        envelope = TASOPT.evaluate_wing_sizing_cases!(wing, (W_cruise = 5.0e5, W_MTO = 5.5e5, W_fuel = 1.0e5,
            W_reserve = 2.0e4, W_empty = 3.0e5, W_fuel_cap = 1.5e5, W_fuel_at_MTO = 8.0e4, r_fuel_design = 0.5),
            true, 1.0, 1.0, Lhtail, gammat, gammas, We, dyeinn)
        @test wing.sizing_loads.governing_strut == :none
        @test wing.sizing_loads.results[:pullup].strut_force == 0.0
        @test wing.sizing_loads.results[:pullup].P_axial_inboard == 0.0
        @test !haskey(envelope, :T_strut)
        @test !haskey(envelope, :T_strut_compression)
        @test wing.sizing_loads.governing_strut_compression == :none
    end
end

# Text summary of the wing sizing cases

@testset "wing sizing loads summary" verbose=true begin
    WingCaseLoads = TASOPT.structures.WingCaseLoads
    ac = CoreLogging.with_logger(CoreLogging.NullLogger()) do
        load_default_model()
    end
    sl = ac.wing.sizing_loads

    function summary_text(ac)
        io = IOBuffer()
        TASOPT.wing_sizing_loads_summary(ac; io = io)
        String(take!(io))
    end
    # first line that names the case, so that pullup does not match pullup_light
    case_row(text, name) = findfirst(line -> occursin(Regex("\\b$name\\b"), line), split(text, '\n'))
    row_text(text, name) = split(text, '\n')[case_row(text, name)]
    # the leading digits of a value appear in the text, whatever the number format
    shows_value(row, x) = occursin(first(replace(string(round(abs(x), sigdigits = 3)), r"\D" => ""), 3),
                                   replace(row, r"[.,]" => ""))
    marked(row) = occursin(r"\*|root|break|governing"i, row)

    @testset "no evaluated cases" begin
        @test isempty(sl.results)
        text = summary_text(ac)
        @test occursin(r"no .*cases"i, text)
        @test all(isnothing(case_row(text, name)) for name in fieldnames(typeof(sl))[1:7])
    end

    @testset "table of the evaluated cases" begin
        sl.pushover.n_computed, sl.taxi.n_computed = -1.0, -2.0
        # inserted out of order: the rows follow the order of the case fields
        sl.results[:taxi] = WingCaseLoads(S_root = 1.11e5, M_root = 2.22e6, S_break = 3.33e4, M_break = 4.44e5)
        sl.results[:pushover] = WingCaseLoads(S_root = 5.55e5, M_root = 6.66e6, S_break = 7.77e4, M_break = 8.88e5)
        sl.results[:pullup] = WingCaseLoads(S_root = 1.23e5, M_root = 4.56e6, S_break = 7.89e4, M_break = 9.87e5)
        sl.governing_root, sl.governing_break = :pushover, :taxi
        text = summary_text(ac)
        rows = Dict(name => row_text(text, name) for name in (:pullup, :pushover, :taxi))

        @test isnothing(case_row(text, :pullup_light))
        @test case_row(text, :pullup) < case_row(text, :pushover) < case_row(text, :taxi)
        for (name, loads) in sl.results
            @test all(shows_value(rows[name], getfield(loads, f)) for f in (:S_root, :M_root, :S_break, :M_break))
        end
        @test occursin("-1", rows[:pushover])
        @test occursin("-2", rows[:taxi])
        @test marked(rows[:pushover])
        @test marked(rows[:taxi])
        @test !marked(rows[:pullup])
    end

    @testset "geometry does not print the summary" begin
        io = IOBuffer()
        TASOPT.geometry(ac; io = io)
        @test isnothing(case_row(String(take!(io)), :pushover))
    end
end

# Mass inputs of the wing sizing cases, from the aircraft weights of a sizing iteration

@testset "wing sizing case masses" verbose=true begin
    # 1.7e5 = W_pay - W_paymax + W_fuel is the fuel that fits at MTOW with maximum payload
    sizing_masses(; W_fuel_cap = 3.0e5, has_wing_fuel = true) = TASOPT.wing_sizing_masses(;
        W_cruise = 9.0e5, W_MTO = 1.0e6, W_pay = 1.5e5, W_paymax = 2.0e5, W_fuel = 2.2e5,
        freserve = 0.1, W_fuel_cap, has_wing_fuel)

    @testset "fuel in the wing" begin
        m = sizing_masses()
        @test m.W_cruise == 9.0e5
        @test m.W_MTO == 1.0e6
        @test m.W_fuel == 2.2e5
        @test m.W_fuel_cap == 3.0e5
        @test m.W_reserve ≈ 2.0e4
        @test m.W_empty ≈ 6.3e5
        @test m.W_fuel_at_MTO ≈ 1.7e5
        @test m.r_fuel_design ≈ 1.7e5 / 3.0e5
    end

    @testset "the tank capacity caps the fuel at MTOW" begin
        m = sizing_masses(W_fuel_cap = 1.0e5)
        @test m.W_fuel_at_MTO == 1.0e5
        @test m.r_fuel_design ≈ 1.7
    end

    @testset "fuel in the fuselage" begin
        m = sizing_masses(W_fuel_cap = 0.0, has_wing_fuel = false)
        @test m.W_fuel_at_MTO ≈ 1.7e5
        @test m.r_fuel_design == 0.0
        @test m.W_fuel_cap == 0.0
        @test all(isfinite, values(m))
    end
end

# Sizing records the load cases of the wing

@testset "sized wing records the sizing cases" verbose=true begin
    # The recorded case loads use the panel weights of the previous sizing iteration, so they
    # equal the loads of the sized wing only to the convergence tolerance.
    function test_recorded_pullup(ac)
        sl = ac.wing.sizing_loads
        @test Set(keys(sl.results)) == Set([:pullup])
        @test sl.governing_root == :pullup
        @test sl.governing_break == :pullup
        @test sl.pullup.n_computed == 3.0
        @test sl.governing_strut == :none
        pullup = sl.results[:pullup]
        @test abs(pullup.M_root) ≈ abs(ac.wing.inboard.moment) rtol = 1e-3
        @test abs(pullup.M_break) ≈ abs(ac.wing.outboard.moment) rtol = 1e-3
        @test abs(pullup.S_root) ≈ abs(ac.wing.inboard.max_shear_load) rtol = 1e-3
        @test abs(pullup.S_break) ≈ abs(ac.wing.outboard.max_shear_load) rtol = 1e-3
    end

    @testset "fuel in the wing" begin
        ac = CoreLogging.with_logger(CoreLogging.NullLogger()) do
            ac = load_default_model()
            size_aircraft!(ac, printiter = false)
            ac
        end
        test_recorded_pullup(ac)
    end

    @testset "fuel in the fuselage" begin
        ac = CoreLogging.with_logger(CoreLogging.NullLogger()) do
            ac = read_aircraft_model(joinpath(TASOPT.__TASOPTroot__, "../example/cryo_input.toml"))
            size_aircraft!(ac, iter = 50, printiter = false)
            ac
        end
        test_recorded_pullup(ac)
    end
end

# On a cantilever wing the pull-up governs, so the extra cases must not change the sized wing

@testset "extra sizing cases on a cantilever wing" verbose=true begin
    extra_cases = (:pushover, :pullup_light, :pushover_light, :taxi)

    function size_with(load_model; extra)
        CoreLogging.with_logger(CoreLogging.NullLogger()) do
            ac = load_model()
            if extra
                for name in extra_cases
                    getfield(ac.wing.sizing_loads, name).active = true
                end
            end
            size_aircraft!(ac, iter = 50, printiter = false)
            ac
        end
    end

    function test_extra_cases_do_not_govern(load_model)
        base = size_with(load_model; extra = false)
        extra = size_with(load_model; extra = true)
        sl = extra.wing.sizing_loads
        @test Set(keys(sl.results)) == Set((:pullup, extra_cases...))
        @test sl.governing_root == :pullup
        @test sl.governing_break == :pullup
        @test sl.governing_strut == :none
        @test extra.wing.weight == base.wing.weight
        @test extra.parg[igWMTO] == base.parg[igWMTO]
    end

    @testset "fuel in the wing" begin
        test_extra_cases_do_not_govern(load_default_model)
    end

    @testset "fuel in the fuselage" begin
        test_extra_cases_do_not_govern(() ->
            read_aircraft_model(joinpath(TASOPT.__TASOPTroot__, "../example/cryo_input.toml")))
    end
end

# Gust load factor increment: hand calculations

@testset "wing lift curve slope" verbose=true begin
    wing_lift_curve_slope = TASOPT.wing_lift_curve_slope

    @testset "default_tbw.toml wing at cruise Mach" begin
        ac, _ = read_aircraft_from_text(read(joinpath(TASOPT.__TASOPTroot__,
                                                      "../example/defaults/default_tbw.toml"), String))
        AR, sweep, Mach = ac.wing.layout.AR, ac.wing.layout.sweep, ac.para[iaMach, ipcruise1, 1]
        @test (AR, sweep, Mach) == (20.0, 20.0, 0.8)

        # β = sqrt(1 - 0.8^2) = 0.6 and tand(20)^2 = 0.13247433
        # CLα = 2π·20 / (2 + sqrt(4 + (20·0.6)^2 · (1 + 0.13247433/0.6^2)))
        #     = 125.66370614 / (2 + sqrt(200.98973257)) = 125.66370614 / (2 + 14.17708477) = 7.7680069 1/rad
        @test wing_lift_curve_slope(AR, sweep, Mach) ≈ 7.768006901803908 rtol = 1e-6
    end

    @testset "limits" begin
        # a thin infinite wing in incompressible flow has the 2D slope 2π
        @test wing_lift_curve_slope(1e6, 0.0, 0.0) ≈ 2π rtol = 1e-5
        # incompressible, the slope stays below 2π and grows with the aspect ratio
        @test issorted([wing_lift_curve_slope(AR, 20.0, 0.0) for AR in (5.0, 10.0, 20.0, 30.0)])
        @test all(wing_lift_curve_slope(AR, 20.0, 0.0) < 2π for AR in (5.0, 10.0, 20.0, 30.0))
        # sweep lowers the slope
        @test wing_lift_curve_slope(10.0, 30.0, 0.8) < wing_lift_curve_slope(10.0, 0.0, 0.8)
    end
end

@testset "gust load increment" verbose=true begin
    gust_load_increment = TASOPT.gust_load_increment
    U_de, V_EAS, CL_alpha, c_mean = 15.24, 154.3, 4.5, 4.0   # m/s EAS (50 ft/s, 300 kt), 1/rad, m
    ρ0 = TASOPT.ρSL   # 1.2255627 kg/m³

    # Pratt–Walker: μ = 2(W/S) / (ρ0·c_mean·CLα·g),  K_g = 0.88μ/(5.3+μ),  Δn = ρ0·U_de·V·CLα·K_g / (2(W/S))
    @testset "hand calculation" begin
        # W/S = 5500 N/m²: μ = 11000 / (1.2255627·4·4.5·9.81) = 11000/216.40986 = 50.829476
        #   K_g = 0.88·50.829476/(5.3+50.829476) = 0.79690640
        #   Δn  = 1.2255627·15.24·154.3·4.5·0.79690640 / 11000 = 12968.7746·0.79690640/11000 = 0.93953632
        @test gust_load_increment(5500.0, c_mean, CL_alpha, U_de, V_EAS) ≈ 0.9395363236301301 rtol = 1e-6
        # W/S = 3500 N/m²: μ = 7000/216.40986 = 32.346030, K_g = 0.75610912
        #   Δn  = 12968.7746·0.75610912/7000 = 1.40082982
        @test gust_load_increment(3500.0, c_mean, CL_alpha, U_de, V_EAS) ≈ 1.4008298164662953 rtol = 1e-6
    end

    @testset "increment grows as the wing loading falls" begin
        Δn = [gust_load_increment(WS, c_mean, CL_alpha, U_de, V_EAS) for WS in (6000.0, 5000.0, 4000.0, 3000.0)]
        @test issorted(Δn)
        @test all(Δn .> 0)
    end
end

# Switching sizing cases on and off changes the governing cases of a sized wing

@testset "governing sizing case follows the active cases" verbose=true begin
    # The strut-braced default_tbw.toml has no cantilever load cases, and at AR = 20 a cantilever
    # wing cannot be sized, so the reference wing is made a cantilever with AR = 13.
    path = joinpath(TASOPT.__TASOPTroot__, "../example/defaults/default_tbw.toml")
    txt = replace(read(path, String), "wing_config = \"strut_braced\"" => "wing_config = \"cantilever\"",
                  "AR    = 20.0" => "AR    = 13.0")
    ac, _ = read_aircraft_from_text(txt)
    @test !ac.options.has_strut
    @test ac.wing.layout.AR == 13.0

    sl = ac.wing.sizing_loads
    function size_with_active(active)
        for name in (:pullup, :pushover)
            getfield(sl, name).active = name in active
        end
        CoreLogging.with_logger(CoreLogging.NullLogger()) do
            size_aircraft!(ac, iter = 50, printiter = false)
        end
        return (Set(keys(sl.results)), sl.governing_root, sl.governing_break)
    end

    @test size_with_active((:pullup,)) == (Set([:pullup]), :pullup, :pullup)
    # the push-over case (-1 g) has smaller loads than the pull-up (3 g), so it governs once alone
    @test size_with_active((:pushover,)) == (Set([:pushover]), :pushover, :pushover)
    @test size_with_active((:pullup, :pushover)) == (Set([:pullup, :pushover]), :pullup, :pullup)
end
