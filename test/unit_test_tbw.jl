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
    # ac.wing.has_strut = true
    # outcome, detail = size_at_aspect_ratio(AR)
end

@testset "TBW wing configuration option" verbose=true begin
    @testset "WingConfig string names" begin
        @test string(WingConfig.Cantilever)  == "cantilever"
        @test string(WingConfig.StrutBraced) == "strut_braced"
        @test string(WingConfig.TrussBraced) == "truss_braced"
    end

    ac_default = CoreLogging.with_logger(CoreLogging.NullLogger()) do
        load_default_model()
    end

    @testset "opt_wing_config follows has_strut on read" begin
        # Read the default input after applying `edit` to its text
        read_default_toml_edit(edit) = mktempdir() do dir
            path = joinpath(dir, "wing_config_test.toml")
            write(path, edit(read(joinpath(TASOPT.__TASOPTroot__, "../example/defaults/default_input.toml"), String)))
            CoreLogging.with_logger(CoreLogging.NullLogger()) do
                read_aircraft_model(path)
            end
        end

        @test !ac_default.wing.has_strut
        @test ac_default.options.opt_wing_config == WingConfig.Cantilever

        ac_strut = read_default_toml_edit(txt -> replace(txt, "has_strut = false" => "has_strut = true"))
        @test ac_strut.wing.has_strut
        @test ac_strut.options.opt_wing_config == WingConfig.StrutBraced
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

@testset "TBW strut struct" verbose=true begin
    # Default aircraft with the strut switched on, sized with the legacy strut model
    read_strut_aircraft() = mktempdir() do dir
        path = joinpath(dir, "strut_test.toml")
        txt = read(joinpath(TASOPT.__TASOPTroot__, "../example/defaults/default_input.toml"), String)
        write(path, replace(txt, "has_strut = false" => "has_strut = true"))
        CoreLogging.with_logger(CoreLogging.NullLogger()) do
            read_aircraft_model(path)
        end
    end

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
