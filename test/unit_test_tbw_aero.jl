const CoreLogging = Base.CoreLogging
# Tests for the Trefftz plane generalisation to N lifting surfaces (wing, tail and, for braced
# wings, a strut). The wing-only-plus-tail (cantilever) results must stay identical to the
# two-surface implementation.

const trefftz_aero = TASOPT.aerodynamics

"Load an aircraft model from a TOML file without printing log messages."
function load_aircraft_quietly(path = joinpath(TASOPT.__TASOPTroot__, "../example/defaults/default_input.toml"))
    return CoreLogging.with_logger(CoreLogging.NullLogger()) do
        read_aircraft_model(path)
    end
end

load_braced_aircraft_quietly() =
    load_aircraft_quietly(joinpath(TASOPT.__TASOPTroot__, "../example/defaults/default_tbw.toml"))

"""
Set a fixed wing and tail geometry and loading on an unsized aircraft, and return the
`para` vector that `induced_drag!` reads. Geometry values are the ones used in the Trefftz
integration test of `unit_test_trefftz.jl`; the lift split gives a wing CL of 1.19 and a tail CL of 0.01.
"""
function set_trefftz_test_case!(ac)
    wing, htail = ac.wing, ac.htail
    wing.layout.S = 125.0
    wing.layout.span = 35.486921631434697
    wing.layout.root_span = 3.6068
    wing.layout.ηs = 10.113772664958887 / wing.layout.span
    wing.inboard.λ = 1.0
    wing.outboard.λ = 1.0

    htail.layout.S = 25.0
    htail.layout.span = 15.958117796995291
    htail.layout.root_span = 1.524
    htail.layout.ηs = 1.524 / htail.layout.span
    htail.outboard.λ = 0.25

    para = zeros(size(ac.para, 1))
    para[iaCL] = 1.2
    para[iaCLh] = 0.05 # tail CL referenced to the tail area; normalised by the wing area it is 0.01
    para[iarcls] = 0.77
    para[iarclt] = 0.15
    return para
end

"""
Run `induced_drag!` on `ac` for the fixed test case; return `(CDi, span efficiency)`.
The cached wake geometry is discarded first unless `reuse_cached_geometry` is true, so that each
test starts from a freshly built wake system.
"""
function trefftz_drag_for_test_case!(ac; reuse_cached_geometry = false)
    para = set_trefftz_test_case!(ac)
    reuse_cached_geometry || (trefftz_aero.TREFFTZ_GEOMETRY_HASH[] = UInt64(0))
    trefftz_aero.induced_drag!(para, ac, ac.options.trefftz_config)
    return para[iaCDi], para[iaspaneff]
end

@testset "Trefftz surfaces" verbose=true begin

    @testset "Configuration without strut panels" begin
        config = trefftz_aero.get_trefftz_config("MEDIUM")
        @test trefftz_aero.n_surfaces(config) == 2
        @test trefftz_aero.n_total(config.strut_panels) == 0

        # MEDIUM: wing 40+12+6 panels, tail 20+0+4 panels, one dummy point after each surface
        @test trefftz_aero.i_first_wing(config) == 1
        @test trefftz_aero.i_last_wing(config) == 59
        @test trefftz_aero.i_first_tail(config) == 60
        @test trefftz_aero.i_last_tail(config) == 84
        @test trefftz_aero.n_points_used(config) == 84

        # braced = false is the same as not asking for a braced layout
        unbraced = trefftz_aero.get_trefftz_config("MEDIUM"; braced = false)
        @test trefftz_aero.n_surfaces(unbraced) == 2
        @test trefftz_aero.n_points_used(unbraced) == 84
    end

    @testset "Configuration with strut panels" begin
        # quality => (strut panels, points without strut, points with strut)
        # The strut has the tail's total panel count: 10+2, 20+4 and 80+16 panels
        cases = ("COARSE" => (12, 43, 56),
                 "MEDIUM" => (24, 84, 109),
                 "FINE"   => (96, 330, 427))
        for (quality, (n_strut_panels, n_points_cantilever, n_points_braced)) in cases
            @testset "$quality" begin
                config = trefftz_aero.get_trefftz_config(quality; braced = true)
                @test trefftz_aero.n_surfaces(config) == 3
                @test config.strut_panels == trefftz_aero.SurfaceDiscretization(n_strut_panels, 0, 0)

                # Each surface block follows the previous one without a gap or an overlap
                @test trefftz_aero.i_first_tail(config) == trefftz_aero.i_last_wing(config) + 1
                @test trefftz_aero.i_first_strut(config) == trefftz_aero.i_last_tail(config) + 1
                @test trefftz_aero.i_last_strut(config) ==
                      trefftz_aero.i_first_strut(config) + n_strut_panels

                # The strut block ends the point list
                @test trefftz_aero.n_points_used(config) == n_points_braced
                @test trefftz_aero.i_last_strut(config) == n_points_braced
                @test trefftz_aero.i_last_tail(config) ==
                      trefftz_aero.n_points_used(trefftz_aero.get_trefftz_config(quality))
                @test trefftz_aero.i_last_tail(config) == n_points_cantilever

                # The strut and the tail have the same number of points
                @test trefftz_aero.n_strut_points(config.strut_panels) ==
                      trefftz_aero.i_last_tail(config) - trefftz_aero.i_first_tail(config) + 1
                @test trefftz_aero.i_last_strut(config) - trefftz_aero.i_first_strut(config) ==
                      trefftz_aero.i_last_tail(config) - trefftz_aero.i_first_tail(config)
            end
        end
    end

    @testset "Geometry array overflow includes the strut block" begin
        # FINE wing and tail use 330 of the points in the geometry arrays
        n_slots = length(trefftz_aero.TREFFTZ_GEOM.y)
        n_strut_panels_that_fit = n_slots - 330 - 1 # one more point is the dummy point after the strut
        @test n_slots >= 427 # FINE with a strut
        wing_panels = trefftz_aero.SurfaceDiscretization(160, 48, 24)
        tail_panels = trefftz_aero.SurfaceDiscretization(80, 0, 16)
        config_with_strut(n_strut_panels) = trefftz_aero.TrefftzPlaneConfig(wing_panels, tail_panels;
            strut_panels = trefftz_aero.SurfaceDiscretization(n_strut_panels, 0, 0))

        @test trefftz_aero.n_points_used(config_with_strut(n_strut_panels_that_fit)) == n_slots # just fits
        @test_throws ErrorException config_with_strut(n_strut_panels_that_fit + 1)
    end

    @testset "calculate_array_size includes the strut block" begin
        ac = load_aircraft_quietly()
        htail = ac.htail
        htail.layout.root_span = 1.524
        cantilever_config = trefftz_aero.get_trefftz_config("COARSE")
        braced_config = trefftz_aero.get_trefftz_config("COARSE"; braced = true)

        @test trefftz_aero.calculate_array_size(cantilever_config, htail) == 43
        @test trefftz_aero.calculate_array_size(braced_config, htail) == 56

        # With a T-tail the tail image panels (2 in COARSE) drop out, the strut block does not
        htail.layout.root_span = 0.0
        @test trefftz_aero.calculate_array_size(cantilever_config, htail) == 41
        @test trefftz_aero.calculate_array_size(braced_config, htail) == 54
    end

    @testset "Cantilever induced drag is unchanged" begin
        ac = load_aircraft_quietly()
        @test trefftz_aero.n_surfaces(ac.options.trefftz_config) == 2

        CDi, span_efficiency = trefftz_drag_for_test_case!(ac)

        # Reference values were computed with the two-surface implementation (before strut panels
        # existed) by running this same test case. The tolerance only allows for platform-dependent
        # rounding in the linear algebra.
        @test CDi ≈ 0.05468605653459833 rtol = 1e-12
        @test span_efficiency ≈ 0.8319732645252401 rtol = 1e-12
        @test length(ac.wake_system.points) == 43
        @test length(ac.wake_system.elements) == 42
    end

    @testset "Braced aircraft gets a strut surface in the wake system" begin
        ac_cantilever = load_aircraft_quietly()
        ac_braced = load_braced_aircraft_quietly()
        braced_config = ac_braced.options.trefftz_config
        @test trefftz_aero.n_surfaces(braced_config) == 3
        @test trefftz_aero.n_surfaces(ac_cantilever.options.trefftz_config) == 2

        # Both aircraft use COARSE, so the strut is the only difference between the two configurations
        @test trefftz_aero.n_points_used(braced_config) ==
              trefftz_aero.n_points_used(ac_cantilever.options.trefftz_config) + 13

        CDi_cantilever, span_efficiency_cantilever = trefftz_drag_for_test_case!(ac_cantilever)
        CDi_braced, span_efficiency_braced = trefftz_drag_for_test_case!(ac_braced)

        @test length(ac_braced.wake_system.points) == 56
        @test length(ac_braced.wake_system.elements) == 55
        @test isfinite(CDi_braced) && CDi_braced > 0.0
        @test isfinite(span_efficiency_braced) && 0.0 < span_efficiency_braced <= 1.0

        # The strut carries no lift, so its wake has zero circulation and cannot induce drag on the
        # wing or tail; only rounding differences remain (rtol = 1e-8 is far above them).
        @test CDi_braced ≈ CDi_cantilever rtol = 1e-8
        @test span_efficiency_braced ≈ span_efficiency_cantilever rtol = 1e-8

        i_strut = trefftz_aero.i_first_strut(braced_config):(trefftz_aero.i_last_strut(braced_config) - 1)
        @test all(==(0.0), trefftz_aero.TREFFTZ_GEOM.gc[i_strut])

        # Placeholder strut is a flat line: constant z
        strut_z = [ac_braced.wake_system.points[i][2] for i in i_strut]
        @test all(==(first(strut_z)), strut_z)
    end

    @testset "Switching between two and three surfaces on unchanged geometry" begin
        ac_cantilever = load_aircraft_quietly()
        ac_braced = load_braced_aircraft_quietly()

        # The wake geometry is cached on the geometry: the surface count must also trigger a rebuild
        CDi_cantilever, _ = trefftz_drag_for_test_case!(ac_cantilever)
        trefftz_drag_for_test_case!(ac_braced; reuse_cached_geometry = true)
        @test length(ac_braced.wake_system.points) == 56
        CDi_cantilever_again, _ = trefftz_drag_for_test_case!(ac_cantilever; reuse_cached_geometry = true)
        @test length(ac_cantilever.wake_system.points) == 43
        @test CDi_cantilever_again ≈ CDi_cantilever rtol = 1e-12
    end
end
