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
    # Default aircraft with the strut switched on, sized with the legacy strut model
    read_strut_aircraft() = read_aircraft_with_wing_config("strut_braced")

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
    end

    @testset "JuryStrut defaults" begin
        jury = TASOPT.structures.JuryStrut()
        @test jury.material.name == "TASOPT-Al"
        @test jury.weight_frac_added == 0.60
        for field in (:eta_wing, :eta_strut, :length, :area, :axial_force,
                      :buckling_load, :weight, :dxW)
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
        @test !hasfield(WingSizingLoads, :governing_strut)

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
    end

    @testset "WingCaseLoads holds only root and break shear and moment" begin
        @test fieldnames(WingCaseLoads) == (:S_root, :M_root, :S_break, :M_break)
        case_loads = WingCaseLoads()
        for name in fieldnames(WingCaseLoads)
            @test getfield(case_loads, name) === 0.0
        end
        @test WingCaseLoads(M_root = 5.0e6).M_root == 5.0e6
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

    @testset "Mission.Nlift sets the pull-up load factor" begin
        default_txt = read(joinpath(TASOPT.__TASOPTroot__, "../example/defaults/default_input.toml"), String)
        txt = replace(default_txt, r"^([ \t]*Nlift[ \t]*=)[ \t]*[0-9.]+"m => s"\1 2.5")
        @test txt != default_txt

        ac, _ = read_aircraft_from_text(txt)
        @test ac.wing.sizing_loads.pullup.load_factor == 2.5
        @test ac.parg[igNlift] == 0.0
        @test read_aircraft_from_text(default_txt)[1].wing.sizing_loads.pullup.load_factor == 3.0

        mktempdir() do dir
            path = joinpath(dir, "nlift_roundtrip.toml")
            save_aircraft_model(ac, path)
            @test TASOPT.TOML.parsefile(path)["Mission"]["Nlift"] == 2.5
            ac_reread, _ = read_aircraft_from_text(read(path, String))
            @test ac_reread.wing.sizing_loads.pullup.load_factor == 2.5
        end
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
