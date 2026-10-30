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

    @testset "generate_trefftz_points_inclined!" begin
        # Fill the arrays with NaN so that any entry the function must not touch is detected
        function nan_filled_geometry()
            geom = trefftz_aero.TrefftzGeometry{40}()
            for name in (:t, :y, :yp, :z, :zp, :yc, :ycp, :zc, :zcp, :gc)
                fill!(getfield(geom, name), NaN)
            end
            return geom
        end

        @testset "Rising segment starting at an offset index" begin
            geom = nan_filled_geometry()
            i_start, n_panels = 5, 8
            y_start, z_start, y_end, z_end = 0.5, -1.4, 21.3, 1.7
            i_end = trefftz_aero.generate_trefftz_points_inclined!(
                i_start, geom, y_start, z_start, y_end, z_end, n_panels)

            @test i_end == i_start + n_panels
            nodes = i_start:i_end
            controls = i_start:(i_end - 1)

            @test geom.y[i_start] == y_start
            @test geom.z[i_start] == z_start
            @test geom.y[i_end] ≈ y_end rtol = 1e-14
            @test geom.z[i_end] ≈ z_end rtol = 1e-14

            # Evenly spaced and collinear
            @test all(isapprox.(diff(geom.y[nodes]), (y_end - y_start) / n_panels; rtol = 1e-12))
            slope = (z_end - z_start) / (y_end - y_start)
            @test all(isapprox.(diff(geom.z[nodes]) ./ diff(geom.y[nodes]), slope; rtol = 1e-12))
            @test all(isapprox.(geom.z[nodes], z_start .+ slope .* (geom.y[nodes] .- y_start); atol = 1e-12))

            # No wake contraction
            @test geom.yp[nodes] == geom.y[nodes]
            @test geom.zp[nodes] == geom.z[nodes]

            # Control points are the midpoints of consecutive nodes, and the strut carries no circulation
            @test geom.yc[controls] ≈ 0.5 .* (geom.y[i_start:(i_end - 1)] .+ geom.y[(i_start + 1):i_end])
            @test geom.zc[controls] ≈ 0.5 .* (geom.z[i_start:(i_end - 1)] .+ geom.z[(i_start + 1):i_end])
            @test geom.ycp[controls] == geom.yc[controls]
            @test geom.zcp[controls] == geom.zc[controls]
            @test all(==(0.0), geom.gc[controls])

            # Nothing outside the strut nodes (and the control points between them) is written
            @test all(isnan, geom.y[1:(i_start - 1)])
            @test all(isnan, geom.y[(i_end + 1):end])
            @test all(isnan, geom.yc[1:(i_start - 1)])
            @test all(isnan, geom.yc[i_end:end])
            @test all(isnan, geom.gc[i_end:end])
        end

        @testset "Falling segment" begin
            geom = nan_filled_geometry()
            i_end = trefftz_aero.generate_trefftz_points_inclined!(1, geom, 2.0, 3.0, 10.0, -1.0, 4)
            @test i_end == 5
            @test geom.z[1:5] ≈ [3.0, 2.0, 1.0, 0.0, -1.0]
            @test geom.y[1:5] ≈ [2.0, 4.0, 6.0, 8.0, 10.0]
        end

        @testset "Single panel" begin
            geom = nan_filled_geometry()
            i_end = trefftz_aero.generate_trefftz_points_inclined!(7, geom, 1.0, 0.0, 5.0, 2.0, 1)
            @test i_end == 8
            @test (geom.y[7], geom.z[7]) == (1.0, 0.0)
            @test (geom.y[8], geom.z[8]) == (5.0, 2.0)
            @test (geom.yc[7], geom.zc[7]) == (3.0, 1.0)
            @test (geom.ycp[7], geom.zcp[7]) == (3.0, 1.0)
            @test geom.gc[7] == 0.0
            @test isnan(geom.yc[8]) && isnan(geom.gc[8])
        end
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

    end

    @testset "Strut is an inclined straight line from the fuselage to the wing" begin
        ac = load_braced_aircraft_quietly()
        config = ac.options.trefftz_config
        trefftz_drag_for_test_case!(ac)
        wing = ac.wing
        @test wing.strut.z > 0.0

        i_nodes = trefftz_aero.i_first_strut(config):trefftz_aero.i_last_strut(config)
        strut_y = [ac.wake_system.points[i][1] for i in i_nodes]
        strut_z = [ac.wake_system.points[i][2] for i in i_nodes]
        @test length(i_nodes) == trefftz_aero.n_total(config.strut_panels) + 1

        # Fuselage attachment is strut.z below the wing box; the wing attachment is at the wing height
        @test strut_y[1] == wing.strut.y_root
        @test strut_z[1] ≈ wing.layout.z - wing.strut.z rtol = 1e-14
        @test strut_y[end] ≈ 0.5 * wing.layout.break_span rtol = 1e-14
        @test strut_z[end] ≈ wing.layout.z rtol = 1e-14

        # The strut rises towards the wing along a line of constant slope
        @test all(diff(strut_y) .> 0.0)
        @test all(diff(strut_z) .> 0.0)
        slope = (strut_z[end] - strut_z[1]) / (strut_y[end] - strut_y[1])
        @test all(isapprox.(diff(strut_z) ./ diff(strut_y), slope; rtol = 1e-10))
    end

    @testset "Inclined strut elements have finite influence coefficients" begin
        ac = load_braced_aircraft_quietly()
        config = ac.options.trefftz_config
        trefftz_drag_for_test_case!(ac)
        influence_matrix = ac.wake_system.influence_matrix

        n_points = trefftz_aero.n_points_used(config)
        @test size(influence_matrix) == (n_points - 1, n_points)
        @test all(isfinite, influence_matrix)

        # Strut control points are rows of the strut block; the strut wake points are columns of it
        i_strut_elements = trefftz_aero.i_first_strut(config):(trefftz_aero.i_last_strut(config) - 1)
        i_strut_points = trefftz_aero.i_first_strut(config):trefftz_aero.i_last_strut(config)
        @test all(isfinite, influence_matrix[i_strut_elements, :])
        @test all(isfinite, influence_matrix[:, i_strut_points])
        @test any(!=(0.0), influence_matrix[i_strut_elements, :])
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

    @testset "Wake geometry is rebuilt only when the geometry changes" begin
        wake_points(ac, i_points) = [ac.wake_system.points[i] for i in i_points]
        all_at_height(points, z) = all(point -> isapprox(point[2], z; atol = 1e-12), points)

        @testset "Braced aircraft: strut and wing height" begin
            ac = load_braced_aircraft_quietly()
            config = ac.options.trefftz_config
            wing = ac.wing
            para = set_trefftz_test_case!(ac)
            i_wing = trefftz_aero.i_first_wing(config):trefftz_aero.i_last_wing(config)
            i_strut = trefftz_aero.i_first_strut(config):trefftz_aero.i_last_strut(config)

            trefftz_aero.TREFFTZ_GEOMETRY_HASH[] = UInt64(0)
            trefftz_aero.induced_drag!(para, ac, config)

            # Unchanged geometry keeps the wake system
            wake_system = ac.wake_system
            trefftz_aero.induced_drag!(para, ac, config)
            @test ac.wake_system === wake_system

            # Each change is made on top of the previous state, without resetting the cached geometry
            changes = (
                "strut.y_root" => () -> wing.strut.y_root += 0.2,
                "strut.z"      => () -> wing.strut.z += 0.1,
                "layout.z"     => () -> wing.layout.z += 0.3,
                "layout.ηs"    => () -> wing.layout.ηs += 0.05)
            for (name, change!) in changes
                @testset "$name" begin
                    wake_system = ac.wake_system
                    strut_before = wake_points(ac, i_strut)
                    wing_before = wake_points(ac, i_wing)

                    change!()
                    trefftz_aero.induced_drag!(para, ac, config)
                    strut_after = wake_points(ac, i_strut)
                    wing_after = wake_points(ac, i_wing)

                    @test ac.wake_system !== wake_system
                    @test strut_after[1][1] == wing.strut.y_root
                    @test strut_after[1][2] ≈ wing.layout.z - wing.strut.z rtol = 1e-14
                    @test strut_after[end][1] ≈ 0.5 * wing.layout.break_span rtol = 1e-14
                    @test strut_after[end][2] ≈ wing.layout.z rtol = 1e-14
                    @test all_at_height(wing_after, wing.layout.z)
                    @test strut_after != strut_before
                    name == "layout.z" && @test wing_after != wing_before
                end
            end
        end

        @testset "Cantilever aircraft: wing and tail height" begin
            for (name, surface_of) in ("wing" => ac -> ac.wing, "htail" => ac -> ac.htail)
                @testset "$name.layout.z" begin
                    ac = load_aircraft_quietly()
                    config = ac.options.trefftz_config
                    surface = surface_of(ac)
                    para = set_trefftz_test_case!(ac)
                    i_wing = trefftz_aero.i_first_wing(config):trefftz_aero.i_last_wing(config)
                    i_tail = trefftz_aero.i_first_tail(config):trefftz_aero.i_last_tail(config)

                    trefftz_aero.TREFFTZ_GEOMETRY_HASH[] = UInt64(0)
                    trefftz_aero.induced_drag!(para, ac, config)
                    wake_system = ac.wake_system
                    CDi_before = para[iaCDi]

                    surface.layout.z += 0.4
                    trefftz_aero.induced_drag!(para, ac, config)

                    @test ac.wake_system !== wake_system
                    @test all_at_height(wake_points(ac, i_wing), ac.wing.layout.z)
                    @test all_at_height(wake_points(ac, i_tail), ac.htail.layout.z)

                    # The height difference between wing and tail changes the induced drag. The result
                    # with the updated wake must be the one of a wake built from scratch.
                    CDi_updated, span_efficiency_updated = para[iaCDi], para[iaspaneff]
                    @test !isapprox(CDi_updated, CDi_before; rtol = 1e-4)
                    trefftz_aero.TREFFTZ_GEOMETRY_HASH[] = UInt64(0)
                    trefftz_aero.induced_drag!(para, ac, config)
                    @test CDi_updated ≈ para[iaCDi] rtol = 1e-12
                    @test span_efficiency_updated ≈ para[iaspaneff] rtol = 1e-12
                end
            end

            # Moving the wing and the tail by the same height leaves the induced drag unchanged
            ac = load_aircraft_quietly()
            CDi, span_efficiency = trefftz_drag_for_test_case!(ac)
            ac.wing.layout.z += 0.4
            ac.htail.layout.z += 0.4
            CDi_shifted, span_efficiency_shifted = trefftz_drag_for_test_case!(ac; reuse_cached_geometry = true)
            @test CDi_shifted ≈ CDi rtol = 1e-12
            @test span_efficiency_shifted ≈ span_efficiency rtol = 1e-12
        end
    end
end
