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
    wall_to_depth = :wall_to_depth,
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
    wall_to_depth = 0.07
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
        wall_to_depth = 0.07,
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
        @test strut.wall_to_depth == 0.05
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
            @test 0 < wing.strut.wall_to_depth < 1
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
        @test default_strut.wall_to_depth == 0.05
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

    @testset "wall_to_depth sets the strut chord" begin
        @test strut.wall_to_depth == 0.05
        @test strut.chord ≈ sqrt(0.5 * strut.area / (0.05 * strut.thickness_to_chord)) rtol=1e-10

        ac_thicker = read_strut_aircraft()
        ac_thicker.wing.strut.wall_to_depth = 0.08
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
