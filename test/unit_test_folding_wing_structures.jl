using Printf

@testset "Folding wing structures" verbose=true begin
    database = TASOPT.materials.MaterialProperties

    # Composite Simpson rule with n (even) intervals, used to integrate the reference loads and fractions
    function simpson(f, a, c, n = 200)
        h = (c - a) / n
        return h / 3 * (f(a) + f(c) + 4 * sum(f(a + i * h) for i in 1:2:n-1) +
                        2 * sum(f(a + i * h) for i in 2:2:n-2))
    end

    @testset "FoldingWingTip defaults" begin
        fold = TASOPT.structures.FoldingWingTip()

        # Folding is off and the counts and factors are zero, so that a missing input fails the input checks
        @test fold.has_fold == false
        @test fold.n_hinge == 0
        @test fold.n_latch == 0
        @test fold.fitting_factor == 0.0

        for name in fieldnames(typeof(fold))
            value = getfield(fold, name)
            if value isa Float64
                @test isfinite(value)
            end
        end

        @test fold.W_fold == 0.0
        @test fold.S_fold == 0.0
        @test fold.M_fold == 0.0
        @test fold.CD_fold == 0.0
        @test fold.span_margin == 0.0

        # Materials: 300M pins and lugs, titanium fittings; bearing strengths from the pin and lug alloys
        @test fold.material_pin.name == "300M"
        @test fold.material_lug.name == "300M"
        @test fold.material_fitting.name == "Ti-6Al-4V"
        @test fold.σ_br_pin == database["300M"]["bearing_strength"]
        @test fold.σ_br_lug == database["300M"]["bearing_strength"]
    end

    @testset "FoldingWingTip keyword override" begin
        fold = TASOPT.structures.FoldingWingTip(has_fold = true, n_hinge = 3,
                                                material_fitting = StructuralAlloy("15-5PH"))

        @test fold.has_fold == true
        @test fold.n_hinge == 3
        @test fold.material_fitting.name == "15-5PH"

        @test fold.n_latch == 0
        @test fold.fitting_factor == 0.0
        @test fold.material_pin.name == "300M"
        @test fold.W_fold == 0.0
    end

    @testset "Wing fold field" begin
        ac = load_default_model()
        @test ac.wing.fold isa TASOPT.structures.FoldingWingTip
        @test ac.wing.fold.has_fold == false

        # Each Wing gets its own fold object
        wing_a = TASOPT.structures.Wing()
        wing_b = TASOPT.structures.Wing()
        wing_a.fold.n_hinge = 2
        @test wing_b.fold.n_hinge == 0

        # Existing access through the getproperty override still works
        @test ac.wing.S === ac.wing.layout.S
        @test ac.wing.has_strut == false

        size_aircraft!(ac; printiter = false)
        @test ac.wing.fold.has_fold == false
        @test ac.wing.fold.W_fold == 0.0
    end

    @testset "FoldingTip input block" begin
        defaults_dir = joinpath(TASOPT.__TASOPTroot__, "../example/defaults")
        default_input = joinpath(defaults_dir, "default_input.toml")
        TOML = TASOPT.TOML
        Logging = Test.Logging

        # Read default_input.toml with the given [Wing.FoldingTip] keys overridden
        function read_with_fold_inputs(; overrides...)
            input = TOML.parsefile(default_input)
            block = get!(input["Wing"], "FoldingTip", Dict{String, Any}())
            for (key, value) in overrides
                block[String(key)] = value
            end
            mktempdir() do dir
                path = joinpath(dir, "modified_input.toml")
                open(io -> TOML.print(io, input), path, "w")
                Logging.with_logger(Logging.NullLogger()) do
                    read_aircraft_model(path)
                end
            end
        end

        # Read an input file without printing the @info about the fallback to the default block
        read_quietly(path) = Logging.with_logger(Logging.NullLogger()) do
            read_aircraft_model(path)
        end

        # Starting values of the block with folding off
        function test_starting_values(fold)
            @test fold.has_fold == false
            @test fold.η_fold == 0.7
            @test fold.n_hinge == 1
            @test fold.n_latch == 3
            @test fold.fitting_factor == 1.15
            @test fold.ground_wind_speed ≈ 65 * 0.514444 rtol = 1e-4
            @test fold.rib_min_gauge ≈ 1.5e-3
            @test fold.material_pin.name == "300M"
            @test fold.material_lug.name == "300M"
            @test fold.material_fitting.name == "Ti-6Al-4V"
            @test fold.σ_br_pin == database["300M"]["bearing_strength"]
            @test fold.σ_br_lug == database["300M"]["bearing_strength"]
        end

        @testset "Default model" begin
            test_starting_values(load_default_model().wing.fold)
        end

        @testset "Non-default values" begin
            fold = read_with_fold_inputs(has_fold = true, fold_location = 0.8, n_hinge = 3, n_latch = 2,
                                         fitting_factor = 1.3, fitting_material = "15-5PH",
                                         pin_material = "Ti-6Al-4V").wing.fold

            @test fold.has_fold == true
            @test fold.η_fold == 0.8
            @test fold.n_hinge == 3
            @test fold.n_latch == 2
            @test fold.fitting_factor == 1.3
            @test fold.material_fitting.name == "15-5PH"
            @test fold.material_pin.name == "Ti-6Al-4V"
            @test fold.material_lug.name == "300M"

            # Bearing strengths follow the chosen pin and lug alloys
            @test fold.σ_br_pin == database["Ti-6Al-4V"]["bearing_strength"]
            @test fold.σ_br_lug == database["300M"]["bearing_strength"]
        end

        @testset "Invalid inputs" begin
            # Rejected when folding is on
            @test_throws ErrorException read_with_fold_inputs(has_fold = true, fold_location = 0.2) # below the panel break
            @test_throws ErrorException read_with_fold_inputs(has_fold = true, fold_location = 1.0)
            @test_throws ErrorException read_with_fold_inputs(has_fold = true, n_hinge = 0)
            @test_throws ErrorException read_with_fold_inputs(has_fold = true, n_latch = 0)
            @test_throws ErrorException read_with_fold_inputs(has_fold = true, n_latch = 1)
            @test_throws ErrorException read_with_fold_inputs(has_fold = true, fitting_factor = 0.9)

            # Not checked when folding is off
            fold = read_with_fold_inputs(has_fold = false, n_hinge = 0, fitting_factor = 0.9).wing.fold
            @test fold.n_hinge == 0
            @test fold.fitting_factor == 0.9
        end

        @testset "Inputs without a FoldingTip block" begin
            for name in ("default_regional.toml", "default_wide.toml")
                input = TOML.parsefile(joinpath(defaults_dir, name))
                delete!(input["Wing"], "FoldingTip")
                mktempdir() do dir
                    path = joinpath(dir, "no_fold_block.toml")
                    open(io -> TOML.print(io, input), path, "w")
                    test_starting_values(read_quietly(path).wing.fold)
                end
            end
        end

        @testset "Shipped inputs have three latches and the actuator torque density" begin
            example_dir = joinpath(TASOPT.__TASOPTroot__, "../example")
            for path in (default_input,
                         joinpath(defaults_dir, "default_folding_wing.toml"),
                         joinpath(defaults_dir, "default_regional.toml"),
                         joinpath(defaults_dir, "default_wide.toml"),
                         joinpath(example_dir, "cryo_input.toml"))
                block = TOML.parsefile(path)["Wing"]["FoldingTip"]
                @test block["n_latch"] == 3
                @test block["actuator_torque_density"] == 0.01
                @test read_quietly(path).wing.fold.rib_min_gauge ≈ 1.5e-3
            end

            fold = read_quietly(joinpath(defaults_dir, "default_folding_wing.toml")).wing.fold
            @test fold.has_fold
            @test fold.n_latch == 3
            @test fold.actuator_torque_density == 0.01
        end

        @testset "Shipped inputs have a FoldingTip block with folding off" begin
            example_dir = joinpath(TASOPT.__TASOPTroot__, "../example")
            expected_keys = keys(TOML.parsefile(default_input)["Wing"]["FoldingTip"])
            for path in (joinpath(defaults_dir, "default_regional.toml"),
                         joinpath(defaults_dir, "default_wide.toml"),
                         joinpath(example_dir, "cryo_input.toml"))
                wing = TOML.parsefile(path)["Wing"]
                @test haskey(wing, "FoldingTip")
                block = get(wing, "FoldingTip", Dict{String, Any}())
                @test get(block, "has_fold", nothing) === false
                @test Set(keys(block)) == Set(expected_keys)
            end
        end
    end

    @testset "Fold inputs survive save and read" begin
        scalar_fold_fields = (:has_fold, :η_fold, :n_hinge, :n_latch, :fitting_factor,
            :lug_R_over_d, :lug_L_over_d, :fitting_length_factor, :rib_min_gauge,
            :actuator_torque_density, :ground_wind_speed, :systems_mass, :ΔCD_fold_counts)

        mktempdir() do dir
            #non-default values for every input are read back unchanged
            ac_fold = load_default_model()
            fold = ac_fold.wing.fold
            fold.has_fold, fold.η_fold, fold.n_hinge, fold.n_latch, fold.fitting_factor = true, 0.8, 3, 2, 1.3
            fold.material_pin = StructuralAlloy("Ti-6Al-4V")
            fold.material_lug = StructuralAlloy("15-5PH")
            fold.material_fitting = StructuralAlloy("300M")
            fold.lug_R_over_d, fold.lug_L_over_d, fold.fitting_length_factor = 1.8, 1.2, 2.5
            fold.rib_min_gauge, fold.actuator_torque_density = 0.002, 0.01
            fold.ground_wind_speed, fold.systems_mass, fold.ΔCD_fold_counts = 30.0, 12.0, 1.5

            filepath_fold = joinpath(dir, "iotest_fold.toml")
            save_aircraft_model(ac_fold, filepath_fold)
            fold_reread = read_aircraft_model(filepath_fold).wing.fold
            for field in scalar_fold_fields
                @test getfield(fold_reread, field) == getfield(fold, field)
            end
            @test fold_reread.material_pin.name == "Ti-6Al-4V"
            @test fold_reread.material_lug.name == "15-5PH"
            @test fold_reread.material_fitting.name == "300M"
            @test fold_reread.σ_br_pin == database["Ti-6Al-4V"]["bearing_strength"]
            @test fold_reread.σ_br_lug == database["15-5PH"]["bearing_strength"]

            #the default model keeps fold off and its starting values
            ac_def = load_default_model()
            fold_def = ac_def.wing.fold
            filepath_nofold = joinpath(dir, "iotest_nofold.toml")
            save_aircraft_model(ac_def, filepath_nofold)
            fold_def_reread = read_aircraft_model(filepath_nofold).wing.fold
            @test !fold_def_reread.has_fold
            for field in scalar_fold_fields
                @test getfield(fold_def_reread, field) == getfield(fold_def, field)
            end
        end
    end

    @testset "Fold station geometry" begin
        wing = TASOPT.structures.Wing()
        wing.layout.ηs = 0.3
        wing.layout.span = 40.0
        wing.layout.root_chord = 6.0
        wing.layout.sweep = 25.0
        wing.inboard.λ = 0.6
        wing.outboard.λ = 0.25
        wing.outboard.cross_section.thickness_to_chord = 0.12
        wing.outboard.cross_section.web_to_box_height = 0.75
        wing.outboard.cross_section.width_to_chord = 0.5

        # Hand calculation of the root-mean-square box height for a web height of 0.75 of the box height
        A = 1 - 0.75
        h_rms = 0.12 * sqrt(1 - 2A / 3 + A^2 / 5)

        # Fold at 70% semi-span: the chord ratio is 0.6 - 0.35 * (0.7 - 0.3) / (1 - 0.3) = 0.4
        geometry = TASOPT.structures.fold_station_geometry(wing, 0.7)
        c_perp = 6.0 * 0.4 * cosd(25.0)
        @test geometry.λ_f ≈ 0.4
        @test geometry.ℓ ≈ 20.0 * 0.3
        @test geometry.y_f ≈ 0.7 * 20.0
        @test geometry.c_f ≈ 2.4
        @test geometry.c_perp_f ≈ c_perp
        @test geometry.h_box_eff ≈ h_rms * c_perp
        @test geometry.w_box ≈ 0.5 * c_perp

        # At the panel break the chord ratio is the inboard one
        at_break = TASOPT.structures.fold_station_geometry(wing, 0.3)
        @test at_break.λ_f ≈ 0.6
        @test at_break.ℓ ≈ 20.0 * (1 - 0.3)
        @test at_break.y_f ≈ 0.3 * 20.0

        # At the tip the chord ratio is the tip one and nothing is left outboard
        at_tip = TASOPT.structures.fold_station_geometry(wing, 1.0)
        @test at_tip.λ_f ≈ 0.25
        @test at_tip.ℓ == 0.0
        @test at_tip.y_f ≈ 20.0
    end

    @testset "Fold station loads" begin
        wing = TASOPT.structures.Wing()
        wing.layout.ηs = 0.3
        wing.layout.span = 40.0
        wing.layout.root_chord = 6.0
        wing.layout.sweep = 25.0
        wing.inboard.λ = 0.6
        wing.outboard.λ = 0.25
        wing.tip_lift_loss = 0.1
        wing.fold.η_fold = 0.7

        po = 1e5
        γs, γt = 0.8, 0.3
        Nload = 2.5
        K_w = 4.0e4
        b, co, ηs, Λ = 40.0, 6.0, 0.3, 25.0
        λs, λt = 0.6, 0.25

        γ(η) = γs + (γt - γs) * (η - ηs) / (1 - ηs)
        λ(η) = λs + (λt - λs) * (η - ηs) / (1 - ηs)
        ΔL_t = 0.1 * po * co * γt * λt

        # Numerical shear and moment about η_f of the lift and of the structural weight outboard of η_f
        function outboard_integrals(η_f)
            lift_shear = simpson(η -> po * γ(η) * b / 2, η_f, 1.0)
            lift_moment = simpson(η -> po * γ(η) * b / 2 * (η - η_f) * b / 2, η_f, 1.0)
            weight = simpson(η -> K_w * co^2 * cosd(Λ) * λ(η)^2 * b / 2, η_f, 1.0)
            weight_moment = simpson(η -> K_w * co^2 * cosd(Λ) * λ(η)^2 * b / 2 * (η - η_f) * b / 2, η_f, 1.0)
            return lift_shear, lift_moment, weight, weight_moment
        end

        loads = TASOPT.structures.fold_station_loads(wing, po, γt, γs, Nload, K_w)
        lift_shear, lift_moment, weight, weight_moment = outboard_integrals(0.7)
        ℓ = b / 2 * (1 - 0.7)

        # Lift shear and moment, including the tip roll-off point load at the tip
        @test loads.S_a ≈ lift_shear + ΔL_t rtol = 1e-8
        @test loads.M_a ≈ lift_moment + ΔL_t * ℓ rtol = 1e-8

        # Structural weight and its moment about the fold station
        @test loads.W_tip ≈ weight rtol = 1e-8
        @test loads.dyW_tip ≈ weight_moment rtol = 1e-8

        # Loads on the fold are the lift loads less the inertial relief, with Nload ≠ 1
        @test loads.S_f ≈ loads.S_a - Nload * loads.W_tip
        @test loads.M_f ≈ loads.M_a - Nload * loads.dyW_tip

        # Fold at the panel break: the outer-panel shear and moment of wing_weights! with the structural weight only
        wing.fold.η_fold = ηs
        at_break = TASOPT.structures.fold_station_loads(wing, po, γt, γs, Nload, K_w)
        W_s = K_w * co^2 * (b / 2) * (1 - ηs) * (λs^2 + λs * λt + λt^2) / 3 * cosd(Λ)
        dyW_s = K_w * co^2 * (b^2 / 48) * (1 - ηs)^2 * (λs^2 + 2 * λs * λt + 3 * λt^2) * cosd(Λ)
        S_s = (po * b / 4) * (γs + γt) * (1 - ηs) + ΔL_t - Nload * W_s
        M_s = (po * b^2 / 24) * (γs + 2 * γt) * (1 - ηs)^2 + ΔL_t * (b / 2) * (1 - ηs) - Nload * dyW_s
        @test at_break.W_tip ≈ W_s
        @test at_break.dyW_tip ≈ dyW_s
        @test at_break.S_f ≈ S_s
        @test at_break.M_f ≈ M_s

        # Fold at the tip: no structure and no lever arm are left outboard, only the roll-off point load
        wing.fold.η_fold = 1.0
        at_tip = TASOPT.structures.fold_station_loads(wing, po, γt, γs, Nload, K_w)
        @test at_tip.W_tip == 0.0
        @test at_tip.dyW_tip == 0.0
        @test at_tip.M_a == 0.0
        @test at_tip.M_f == 0.0
        @test at_tip.S_f ≈ ΔL_t

        # Without the tip roll-off loss there is nothing outboard of a fold at the tip
        wing.tip_lift_loss = 0.0
        no_loss = TASOPT.structures.fold_station_loads(wing, po, γt, γs, Nload, K_w)
        @test all(iszero, values(no_loss))
    end

    @testset "Pinned joint sizing" begin
        gee = TASOPT.gee
        make_fold(; pin = "300M", lug = "300M", n_hinge = 2, n_latch = 2) = TASOPT.structures.FoldingWingTip(
            n_hinge = n_hinge, n_latch = n_latch,
            material_pin = StructuralAlloy(pin), material_lug = StructuralAlloy(lug),
            σ_br_pin = database[pin]["bearing_strength"], σ_br_lug = database[lug]["bearing_strength"],
            lug_R_over_d = 1.5, lug_L_over_d = 1.5)

        # Pin diameter, lug thickness and weight from the double shear and bearing formulas
        function hand_calculation(F, n, pin, lug, sigfac)
            τ_allow = StructuralAlloy(pin).τmax * sigfac
            σ_br_allow = min(database[pin]["bearing_strength"], database[lug]["bearing_strength"]) / 1.5 * sigfac
            d = sqrt(2F / (n * π * τ_allow))
            t = F / (n * d * σ_br_allow)
            R, L = 1.5d, 1.5d
            A_lug = π * R^2 / 2 + 2R * L - π * d^2 / 4
            V_pin = π * d^2 / 4 * (2t)
            V_lug = A_lug * (2t)
            W = gee * n * (database[pin]["density"] * V_pin + database[lug]["density"] * V_lug)
            return d, t, W
        end

        @testset "Hand calculation for each pin material" begin
            F, n = 1e6, 2
            for (pin, lug) in (("300M", "300M"), ("Ti-6Al-4V", "15-5PH"), ("15-5PH", "Ti-6Al-4V"))
                W, details = TASOPT.structures.size_pinned_joint(F, n, make_fold(pin = pin, lug = lug), 1.0)
                d, t, W_hand = hand_calculation(F, n, pin, lug, 1.0)
                @test details.d ≈ d rtol = 1e-10
                @test details.t ≈ t rtol = 1e-10
                @test W ≈ W_hand rtol = 1e-10
            end
        end

        @testset "Scaling with force and number of pins" begin
            fold = make_fold(pin = "Ti-6Al-4V", lug = "Ti-6Al-4V")
            joint(F, n) = TASOPT.structures.size_pinned_joint(F, n, fold, 1.0)
            F0 = 1e6
            W_ref, ref = joint(F0, 1)

            # d and t both scale with sqrt(F / n), so the weight scales as n (F / n)^1.5 = F^1.5 / sqrt(n)
            W_double_F, double_F = joint(2F0, 1)
            @test double_F.d / ref.d ≈ sqrt(2) rtol = 1e-10
            @test double_F.t / ref.t ≈ sqrt(2) rtol = 1e-10
            @test W_double_F / W_ref ≈ 2^1.5 rtol = 1e-10

            W_four_pins, four_pins = joint(F0, 4)
            @test four_pins.d / ref.d ≈ 1 / 2 rtol = 1e-10
            @test four_pins.t / ref.t ≈ 1 / 2 rtol = 1e-10
            @test W_four_pins / W_ref ≈ 1 / 2 rtol = 1e-10

            # Two pins instead of one: d, t and the weight all shrink by 1 / sqrt(2)
            W_two_pins, two_pins = joint(F0, 2)
            @test two_pins.d / ref.d ≈ 1 / sqrt(2) rtol = 1e-10
            @test two_pins.t / ref.t ≈ 1 / sqrt(2) rtol = 1e-10
            @test W_two_pins / W_ref ≈ 1 / sqrt(2) rtol = 1e-10
        end

        @testset "Allowable stresses scale with sigfac" begin
            fold = make_fold()
            W_1, one = TASOPT.structures.size_pinned_joint(1e6, 2, fold, 1.0)
            W_2, two = TASOPT.structures.size_pinned_joint(1e6, 2, fold, 2.0)
            d_hand, t_hand, W_hand = hand_calculation(1e6, 2, "300M", "300M", 2.0)

            # Twice the allowables: d shrinks by 1 / sqrt(2) and t = F / (n d σ_br) by (1 / 2) * sqrt(2)
            @test two.d / one.d ≈ 1 / sqrt(2) rtol = 1e-10
            @test two.t / one.t ≈ 1 / sqrt(2) rtol = 1e-10
            @test two.d ≈ d_hand rtol = 1e-10
            @test two.t ≈ t_hand rtol = 1e-10
            @test W_2 ≈ W_hand rtol = 1e-10
            @test W_2 < W_1
        end

        @testset "Zero force gives no joint" begin
            W, details = TASOPT.structures.size_pinned_joint(0.0, 2, make_fold(), 1.0)
            @test W == 0.0
            for name in (:d, :t, :R, :L)
                @test getfield(details, name) == 0.0
            end
        end

        @testset "Latches carry the cap force" begin
            fold = make_fold(n_hinge = 3, n_latch = 5)
            F_cap = 8e5
            W, details = TASOPT.structures.size_latches(F_cap, fold, 1.0)
            W_joint, joint = TASOPT.structures.size_pinned_joint(F_cap, 5, fold, 1.0)
            @test W ≈ W_joint rtol = 1e-12
            @test details.d ≈ joint.d rtol = 1e-12
            @test details.t ≈ joint.t rtol = 1e-12

            # The actuator mass is per latch pin and adds its weight to the pinned joint
            W_actuated, _ = TASOPT.structures.size_latches(F_cap, fold, 1.0; latch_actuator_mass = 0.7)
            @test W_actuated - W ≈ 0.7 * gee * 5 rtol = 1e-10
        end
    end

    @testset "Load introduction sizing" begin
        gee = TASOPT.gee
        h_box, w_box, ρ_rib, sigfac = 0.8, 1.5, 2700.0, 1.0

        # One fitting per cap (two per wing side) and two full-depth closing ribs
        function hand_calculation(F_cap, material, k, t_rib)
            σ_fit = StructuralAlloy(material).σmax * sigfac
            W_fittings = 2 * gee * database[material]["density"] * (F_cap / σ_fit) * k * h_box
            W_ribs = 2 * gee * ρ_rib * w_box * h_box * t_rib
            return W_fittings, W_ribs
        end

        make_fold(material, k, t_rib) = TASOPT.structures.FoldingWingTip(
            material_fitting = StructuralAlloy(material), fitting_length_factor = k, rib_min_gauge = t_rib)

        @testset "Hand calculation for each fitting material" begin
            F_cap = 5e5
            for (material, k, t_rib) in (("Ti-6Al-4V", 2.0, 0.001), ("300M", 3.0, 0.0), ("15-5PH", 1.5, 0.002))
                fold = make_fold(material, k, t_rib)
                W, details = TASOPT.structures.size_load_introduction(F_cap, h_box, w_box, fold, ρ_rib, sigfac)
                W_fittings, W_ribs = hand_calculation(F_cap, material, k, t_rib)
                @test details.W_fittings ≈ W_fittings rtol = 1e-10
                @test details.W_ribs ≈ W_ribs rtol = 1e-10
                @test W ≈ W_fittings + W_ribs rtol = 1e-10
            end
        end

        @testset "Zero rib gauge leaves only the fittings" begin
            fold = make_fold("300M", 3.0, 0.0)
            W, details = TASOPT.structures.size_load_introduction(5e5, h_box, w_box, fold, ρ_rib, sigfac)
            @test details.W_ribs == 0.0
            @test W ≈ details.W_fittings rtol = 1e-12
        end

        @testset "Fitting weight is linear in cap force" begin
            fold = make_fold("Ti-6Al-4V", 2.0, 0.001)
            W_1, one = TASOPT.structures.size_load_introduction(2e5, h_box, w_box, fold, ρ_rib, sigfac)
            W_3, three = TASOPT.structures.size_load_introduction(6e5, h_box, w_box, fold, ρ_rib, sigfac)
            @test three.W_fittings / one.W_fittings ≈ 3 rtol = 1e-10
            @test three.W_ribs ≈ one.W_ribs rtol = 1e-12
        end
    end

    @testset "Fold actuator sizing" begin
        gee = TASOPT.gee

        # Same wing and loads as the fold station loads
        wing = TASOPT.structures.Wing()
        wing.layout.ηs = 0.3
        wing.layout.span = 40.0
        wing.layout.root_chord = 6.0
        wing.layout.sweep = 25.0
        wing.inboard.λ = 0.6
        wing.outboard.λ = 0.25
        wing.tip_lift_loss = 0.1
        wing.fold.η_fold = 0.7
        wing.fold.ground_wind_speed = 33.4
        wing.fold.actuator_torque_density = 1e-3
        loads = TASOPT.structures.fold_station_loads(wing, 1e5, 0.3, 0.8, 2.5, 4.0e4)

        # Hand calculation: fold line at 70% semi-span, chord ratio 0.4 there, tip panel length 20 * 0.3
        ℓ = 20.0 * 0.3
        co, λ_f, λt = 6.0, 0.4, 0.25
        ρ_air = TASOPT.atmos(0.0).ρ
        q_wind = 0.5 * ρ_air * 33.4^2
        M_grav = loads.W_tip * (loads.dyW_tip / loads.W_tip)

        # Wind moment about the fold line: normal force on the tip panel, whose chord is linear from the fold to the tip,
        # integrated with a composite Simpson rule (n even intervals)
        chord(y) = co * (λ_f + (λt - λ_f) * y / ℓ)
        M_wind = simpson(y -> q_wind * 1.2 * chord(y) * y, 0.0, ℓ)

        @testset "Gravity and wind moments" begin
            W, details = TASOPT.structures.size_fold_actuator(wing, loads)
            @test details.M_grav ≈ M_grav rtol = 1e-10
            @test details.M_wind ≈ M_wind rtol = 1e-8
            @test details.M_wind ≈ q_wind * 1.2 * co * ℓ^2 * (λ_f + 2λt) / 6 rtol = 1e-10
            @test details.M_act ≈ M_grav + M_wind rtol = 1e-8
            @test W ≈ gee * 1e-3 * (M_grav + M_wind) rtol = 1e-8
        end

        @testset "Zero torque density gives no actuator weight" begin
            wing.fold.actuator_torque_density = 0.0
            W, details = TASOPT.structures.size_fold_actuator(wing, loads)
            @test W == 0.0
            @test details.M_act ≈ M_grav + M_wind rtol = 1e-8
            wing.fold.actuator_torque_density = 1e-3
        end

        @testset "No wind leaves the gravity moment" begin
            wing.fold.ground_wind_speed = 0.0
            W, details = TASOPT.structures.size_fold_actuator(wing, loads)
            @test details.M_wind == 0.0
            @test details.M_act ≈ M_grav rtol = 1e-10
            @test W ≈ gee * 1e-3 * M_grav rtol = 1e-10
        end

        @testset "No tip structure leaves only the wind moment" begin
            wing.fold.ground_wind_speed = 33.4
            _, details = TASOPT.structures.size_fold_actuator(wing, (W_tip = 0.0, dyW_tip = 0.0))
            @test details.M_grav == 0.0
            @test details.M_act ≈ M_wind rtol = 1e-8
        end
    end

    @testset "Fold mechanism sizing" begin
        gee = TASOPT.gee
        sigfac = 1.3
        b, Λ, ηs, ηo = 40.0, 25.0, 0.3, 0.1

        # Wing as in the fold station loads, with a lighter structure so that the moment at the fold is positive
        make_wing(; fitting_factor = 1.15, η_fold = 0.7) = begin
            wing = TASOPT.structures.Wing()
            wing.layout.ηs = ηs
            wing.layout.root_span = ηo * b
            wing.layout.span = b
            wing.layout.root_chord = 6.0
            wing.layout.sweep = Λ
            wing.inboard.λ = 0.6
            wing.outboard.λ = 0.25
            wing.outboard.cross_section.thickness_to_chord = 0.12
            wing.outboard.cross_section.web_to_box_height = 0.75
            wing.outboard.cross_section.width_to_chord = 0.5
            wing.tip_lift_loss = 0.1
            wing.weight = 1.0e5
            wing.dxW = 2.0e5
            wing.outboard.weight = 3.0e4

            fold = wing.fold
            fold.η_fold = η_fold
            fold.n_hinge = 2
            fold.n_latch = 3
            fold.fitting_factor = fitting_factor
            fold.lug_R_over_d = 1.5
            fold.lug_L_over_d = 1.5
            fold.fitting_length_factor = 3.0
            fold.rib_min_gauge = 0.001
            fold.actuator_torque_density = 1e-3
            fold.ground_wind_speed = 33.4
            fold.systems_mass = 12.0
            return wing
        end
        fold_loads(wing, K_w = 1.0e3) = TASOPT.structures.fold_station_loads(wing, 1e5, 0.3, 0.8, 2.5, K_w)

        wing = make_wing()
        loads = fold_loads(wing)
        fold = wing.fold
        geometry = TASOPT.structures.fold_station_geometry(wing, fold.η_fold)

        # Components sized independently from the loads
        F_cap = 1.15 * (loads.M_f / cosd(Λ)) / geometry.h_box_eff
        # The hinge pins carry the resultant of the cap force and the shear
        W_h, _ = TASOPT.structures.size_pinned_joint(hypot(F_cap, loads.S_f), fold.n_hinge, fold, sigfac)
        W_l, _ = TASOPT.structures.size_latches(F_cap, fold, sigfac)
        W_li, _ = TASOPT.structures.size_load_introduction(F_cap, geometry.h_box_eff, geometry.w_box, fold,
                                                           wing.outboard.caps.ρ, sigfac)
        W_act, _ = TASOPT.structures.size_fold_actuator(wing, loads)
        W_sys = gee * 12.0
        W_side = W_h + W_l + W_li + W_act + W_sys

        TASOPT.structures.size_fold_mechanism!(wing, loads, sigfac, 0.0)

        @testset "Outputs follow from the components" begin
            @test fold.S_fold == loads.S_f
            @test fold.M_fold == loads.M_f
            @test fold.F_cap ≈ F_cap rtol = 1e-12
            @test fold.W_hinge_lugs ≈ 2W_h rtol = 1e-12
            @test fold.W_latches ≈ 2W_l rtol = 1e-12
            @test fold.W_load_intro ≈ 2W_li rtol = 1e-12
            @test fold.W_actuator ≈ 2W_act rtol = 1e-12
            @test fold.W_fold ≈ 2W_side rtol = 1e-12
            @test fold.dyW_fold ≈ W_side * (0.7 - ηs) * b / 2 rtol = 1e-12
            @test fold.dxW_fold ≈ 2W_side * (geometry.y_f - ηo * b / 2) * tand(Λ) rtol = 1e-12
        end

        @testset "Total includes the systems allowance" begin
            components = fold.W_hinge_lugs + fold.W_latches + fold.W_load_intro + fold.W_actuator
            @test fold.W_fold ≈ components + 2 * gee * fold.systems_mass rtol = 1e-12
            @test fold.W_fold > components
        end

        @testset "Only the fold is written, and repeated calls do not accumulate" begin
            first_call = deepcopy(fold)
            TASOPT.structures.size_fold_mechanism!(wing, loads, sigfac, 0.0)
            for name in fieldnames(typeof(fold))
                after, before = getfield(fold, name), getfield(first_call, name)
                if after isa TASOPT.structures.StructuralAlloy
                    # Mutable struct, so compared field by field against the copy
                    @test all(getfield(after, f) == getfield(before, f) for f in fieldnames(typeof(after)))
                else
                    @test after == before
                end
            end

            untouched = make_wing()
            @test wing.weight == untouched.weight
            @test wing.dxW == untouched.dxW
            @test wing.outboard.weight == untouched.outboard.weight
            @test wing.layout.span == untouched.layout.span
            @test wing.outboard.caps.ρ == untouched.outboard.caps.ρ
        end

        @testset "Larger fitting factor increases the cap force and hinge weights" begin
            previous = nothing
            for fitting_factor in (1.0, 1.15, 1.3, 1.6)
                wing_ff = make_wing(fitting_factor = fitting_factor)
                TASOPT.structures.size_fold_mechanism!(wing_ff, fold_loads(wing_ff), sigfac, 0.0)
                current = wing_ff.fold
                if previous !== nothing
                    @test current.F_cap > previous.F_cap
                    @test current.W_hinge_lugs > previous.W_hinge_lugs
                    @test current.W_latches > previous.W_latches
                    @test current.W_load_intro > previous.W_load_intro
                    @test current.W_fold > previous.W_fold
                end
                previous = current
            end
        end

        @testset "Fold at the panel break has no bending relief" begin
            wing_break = make_wing(η_fold = ηs)
            TASOPT.structures.size_fold_mechanism!(wing_break, fold_loads(wing_break), sigfac, 0.0)
            @test wing_break.fold.W_fold > 0.0
            @test wing_break.fold.dyW_fold == 0.0
        end

        @testset "Negative fold moment gives a cap force of equal magnitude" begin
            # A heavy tip structure relieves more bending than the lift applies at the fold
            wing_heavy = make_wing()
            heavy_loads = fold_loads(wing_heavy, 4.0e4)
            @test heavy_loads.M_f < 0.0

            TASOPT.structures.size_fold_mechanism!(wing_heavy, heavy_loads, sigfac, 0.0)
            fold_heavy = wing_heavy.fold
            geometry_heavy = TASOPT.structures.fold_station_geometry(wing_heavy, fold_heavy.η_fold)
            @test fold_heavy.M_fold == heavy_loads.M_f

            # The caps still resist the moment, so the cap force is the magnitude of the one for the same positive moment
            F_cap_heavy = 1.15 * abs(heavy_loads.M_f / cosd(Λ)) / geometry_heavy.h_box_eff
            @test fold_heavy.F_cap > 0.0
            @test fold_heavy.F_cap ≈ F_cap_heavy rtol = 1e-12

            # The components are sized for that force
            W_h, _ = TASOPT.structures.size_pinned_joint(hypot(F_cap_heavy, heavy_loads.S_f), fold_heavy.n_hinge, fold_heavy, sigfac)
            W_l, _ = TASOPT.structures.size_latches(F_cap_heavy, fold_heavy, sigfac)
            W_i, _ = TASOPT.structures.size_load_introduction(F_cap_heavy, geometry_heavy.h_box_eff, geometry_heavy.w_box,
                                                              fold_heavy, wing_heavy.outboard.caps.ρ, sigfac)
            @test fold_heavy.W_hinge_lugs ≈ 2 * W_h rtol = 1e-12
            @test fold_heavy.W_latches ≈ 2 * W_l rtol = 1e-12
            @test fold_heavy.W_load_intro ≈ 2 * W_i rtol = 1e-12
            @test isfinite(fold_heavy.W_actuator)
            @test isfinite(fold_heavy.W_fold)
        end
        @testset "Strength-matched joint" begin
            t_cap = 0.01
            σ_cap = wing.outboard.caps.material.σmax
            w̄ = wing.outboard.cross_section.width_to_chord
            c_perp = 6.0 * 0.4 * cosd(Λ)    # chord ratio 0.4 at the fold, perpendicular to the sweep

            @testset "Cap force capacity" begin
                F_strength = TASOPT.structures.fold_cap_strength(wing, t_cap, sigfac)
                @test F_strength ≈ σ_cap * sigfac * t_cap * w̄ * c_perp^2 rtol = 1e-12
                @test TASOPT.structures.fold_cap_strength(wing, 2t_cap, sigfac) ≈ 2F_strength rtol = 1e-12
                @test TASOPT.structures.fold_cap_strength(wing, t_cap, 2sigfac) ≈ 2F_strength rtol = 1e-12
                @test TASOPT.structures.fold_cap_strength(wing, 0.0, sigfac) == 0.0
            end

            # Cap force and weights of the sizing with the given outboard cap thickness
            function sized_fold(t_cap)
                wing_t = make_wing()
                TASOPT.structures.size_fold_mechanism!(wing_t, fold_loads(wing_t), sigfac, t_cap)
                return wing_t.fold
            end
            F_load = abs(loads.M_f / cosd(Λ)) / geometry.h_box_eff
            F_strength = TASOPT.structures.fold_cap_strength(wing, t_cap, sigfac)
            @test F_strength > F_load

            @testset "Load governs without a cap thickness and for a thin cap" begin
                t_thin = 1e-4
                F_thin = TASOPT.structures.fold_cap_strength(wing, t_thin, sigfac)
                @test F_thin < F_load

                @test sized_fold(0.0).F_cap ≈ 1.15 * F_load rtol = 1e-12
                @test sized_fold(0.0).F_cap_strength == 0.0
                @test sized_fold(t_thin).F_cap ≈ 1.15 * F_load rtol = 1e-12
                @test sized_fold(t_thin).F_cap_strength ≈ F_thin rtol = 1e-12
            end

            @testset "Strength governs for a thick cap" begin
                fold_strength, fold_load = sized_fold(t_cap), sized_fold(0.0)
                @test fold_strength.F_cap_strength ≈ F_strength rtol = 1e-12
                @test fold_strength.F_cap ≈ 1.15 * F_strength rtol = 1e-12

                # The components are sized for the larger cap force
                F_cap = fold_strength.F_cap
                W_h, _ = TASOPT.structures.size_pinned_joint(hypot(F_cap, loads.S_f), fold.n_hinge, fold, sigfac)
                W_l, _ = TASOPT.structures.size_latches(F_cap, fold, sigfac)
                W_li, _ = TASOPT.structures.size_load_introduction(F_cap, geometry.h_box_eff, geometry.w_box, fold,
                                                                   wing.outboard.caps.ρ, sigfac)
                @test fold_strength.W_hinge_lugs ≈ 2W_h rtol = 1e-12
                @test fold_strength.W_latches ≈ 2W_l rtol = 1e-12
                @test fold_strength.W_load_intro ≈ 2W_li rtol = 1e-12

                @test fold_strength.F_cap > fold_load.F_cap
                @test fold_strength.W_hinge_lugs > fold_load.W_hinge_lugs
                @test fold_strength.W_latches > fold_load.W_latches
                @test fold_strength.W_load_intro > fold_load.W_load_intro
                @test fold_strength.W_fold > fold_load.W_fold
            end
        end
    end

    @testset "Fold tip fuel fractions" begin
        ηs, λs, λt = 0.3, 0.6, 0.25
        wing = TASOPT.structures.Wing()
        wing.layout.ηs = ηs
        wing.inboard.λ = λs
        wing.outboard.λ = λt
        wing.fold.has_fold = true

        # Chord ratio varies linearly from the inboard value at the panel break to the tip value at the tip
        λ(η) = λs + (λt - λs) * (η - ηs) / (1 - ηs)

        volume_fraction(η_f) = simpson(η -> λ(η)^2, ηs, η_f) / simpson(η -> λ(η)^2, ηs, 1.0)
        moment_fraction(η_f) = simpson(η -> λ(η)^2 * (η - ηs), ηs, η_f) / simpson(η -> λ(η)^2 * (η - ηs), ηs, 1.0)

        @testset "Match numerical integration" begin
            @test TASOPT.structures.fold_tip_fuel_fraction(wing, 0.7) ≈ volume_fraction(0.7) rtol = 1e-8
            @test TASOPT.structures.fold_tip_fuel_moment_fraction(wing, 0.7) ≈ moment_fraction(0.7) rtol = 1e-8
        end

        @testset "Fold at the panel break and at the tip" begin
            @test TASOPT.structures.fold_tip_fuel_fraction(wing, ηs) == 0.0
            @test TASOPT.structures.fold_tip_fuel_moment_fraction(wing, ηs) == 0.0
            @test TASOPT.structures.fold_tip_fuel_fraction(wing, 1.0) ≈ 1.0
            @test TASOPT.structures.fold_tip_fuel_moment_fraction(wing, 1.0) ≈ 1.0
        end

        @testset "No fold keeps all the fuel volume" begin
            wing.fold.has_fold = false
            @test TASOPT.structures.fold_tip_fuel_fraction(wing, 0.7) == 1.0
            @test TASOPT.structures.fold_tip_fuel_moment_fraction(wing, 0.7) == 1.0
            wing.fold.has_fold = true
        end

        @testset "Fractions grow with the fold station and the moment fraction is smaller" begin
            stations = (0.5, 0.7, 0.9)
            volume = [TASOPT.structures.fold_tip_fuel_fraction(wing, η_f) for η_f in stations]
            moment = [TASOPT.structures.fold_tip_fuel_moment_fraction(wing, η_f) for η_f in stations]
            @test all(diff(volume) .> 0)
            @test all(diff(moment) .> 0)
            @test all(0.0 .< moment .< volume .< 1.0)
        end
    end

    @testset "Ground span and height margins" begin
        b, Λ, ηs, ηo, η_fold = 40.0, 25.0, 0.3, 0.1, 0.7
        max_span, z_wing, fuselage_radius, fin_span = 30.0, -1.5, 3.0, 8.0
        margins! = TASOPT.structures.fold_ground_margins!

        function make_ground_case(; has_fold = true, max_span = max_span, fin_span = fin_span, Γ = 0.0)
            wing = TASOPT.structures.Wing()
            wing.layout.ηs = ηs
            wing.layout.root_span = ηo * b
            wing.layout.span = b
            wing.layout.max_span = max_span
            wing.layout.z = z_wing
            wing.layout.root_chord = 6.0
            wing.layout.sweep = Λ
            wing.inboard.λ = 0.6
            wing.outboard.λ = 0.25
            wing.outboard.cross_section.thickness_to_chord = 0.12
            wing.outboard.cross_section.web_to_box_height = 0.75
            wing.outboard.cross_section.width_to_chord = 0.5
            wing.weight = 1.0e5
            wing.fold.has_fold = has_fold
            wing.fold.η_fold = η_fold

            fuselage = TASOPT.structures.Fuselage{TASOPT.structures.SingleBubble}()
            fuselage.layout.cross_section.radius = fuselage_radius
            vtail = TASOPT.structures.Tail()
            vtail.layout.span = fin_span
            landing_gear = TASOPT.structures.LandingGear()
            landing_gear.wing_dihedral_angle = Γ
            return wing, fuselage, vtail, landing_gear
        end

        # Hand calculation: fold station at 70% of the 20 m semi-span, root at 10% of the span
        y_f = η_fold * b / 2
        y_o = ηo * b / 2
        ℓ = (b / 2) * (1 - η_fold)
        wing_0, _, _, _ = make_ground_case()
        h_box_eff = TASOPT.structures.fold_station_geometry(wing_0, η_fold).h_box_eff

        @testset "Zero dihedral" begin
            wing, fuselage, vtail, landing_gear = make_ground_case()
            @test margins!(wing, fuselage, vtail, landing_gear) === nothing

            z_tip = z_wing + h_box_eff / 2 + ℓ
            @test y_f ≈ 14.0 && ℓ ≈ 6.0
            @test wing.fold.ground_span ≈ 28.0 rtol = 1e-12
            @test wing.fold.span_margin ≈ max_span - 28.0 rtol = 1e-12
            @test wing.fold.z_tip ≈ z_tip rtol = 1e-12
            @test wing.fold.height_margin ≈ (fuselage_radius + fin_span) - z_tip rtol = 1e-12
        end

        @testset "Dihedral raises the hinge and tilts the tip panel" begin
            Γ = 0.1
            wing, fuselage, vtail, landing_gear = make_ground_case(Γ = Γ)
            margins!(wing, fuselage, vtail, landing_gear)

            z_hinge = z_wing + (y_f - y_o) * tan(Γ) + h_box_eff / 2
            z_tip = z_hinge + ℓ * cos(Γ)
            @test wing.fold.ground_span ≈ 28.0 rtol = 1e-12
            @test wing.fold.z_tip ≈ z_tip rtol = 1e-12
            @test wing.fold.height_margin ≈ (fuselage_radius + fin_span) - z_tip rtol = 1e-12

            # Compared with no dihedral, the hinge rises by the dihedral term and the tip panel loses height
            flat, _, _, flat_gear = make_ground_case()
            margins!(flat, fuselage, vtail, flat_gear)
            @test wing.fold.z_tip ≈ flat.fold.z_tip + (y_f - y_o) * tan(Γ) - ℓ * (1 - cos(Γ)) rtol = 1e-10
        end

        @testset "Margins change sign" begin
            # Fin shorter than the folded tip height
            wing, fuselage, vtail, landing_gear = make_ground_case(fin_span = 1.0)
            margins!(wing, fuselage, vtail, landing_gear)
            @test wing.fold.z_tip > fuselage_radius + 1.0
            @test wing.fold.height_margin < 0.0
            @test wing.fold.height_margin ≈ (fuselage_radius + 1.0) - wing.fold.z_tip rtol = 1e-12

            # Span limit below the folded span
            wing, fuselage, vtail, landing_gear = make_ground_case(max_span = 25.0)
            margins!(wing, fuselage, vtail, landing_gear)
            @test wing.fold.span_margin ≈ -3.0 rtol = 1e-12
            @test wing.fold.height_margin > 0.0
        end

        @testset "No fold" begin
            wing, fuselage, vtail, landing_gear = make_ground_case(has_fold = false, Γ = 0.1)
            margins!(wing, fuselage, vtail, landing_gear)
            @test wing.fold.ground_span == b
            @test wing.fold.span_margin == max_span - b
            @test wing.fold.z_tip == 0.0
            @test wing.fold.height_margin == 0.0
        end

        @testset "Only the fold is written, and repeated calls give the same result" begin
            wing, fuselage, vtail, landing_gear = make_ground_case(Γ = 0.1)
            untouched, fuselage_0, vtail_0, landing_gear_0 = make_ground_case(Γ = 0.1)
            margins!(wing, fuselage, vtail, landing_gear)
            first_call = deepcopy(wing.fold)
            margins!(wing, fuselage, vtail, landing_gear)
            for name in fieldnames(typeof(first_call))
                after, before = getfield(wing.fold, name), getfield(first_call, name)
                if after isa TASOPT.structures.StructuralAlloy
                    @test all(getfield(after, f) == getfield(before, f) for f in fieldnames(typeof(after)))
                else
                    @test after == before
                end
            end

            for name in (:root_span, :span, :max_span, :z, :ηs, :root_chord, :sweep)
                @test getfield(wing.layout, name) == getfield(untouched.layout, name)
            end
            @test wing.weight == untouched.weight
            @test wing.dxW == untouched.dxW
            @test wing.outboard.weight == untouched.outboard.weight
            @test vtail.layout.span == vtail_0.layout.span
            @test vtail.weight == vtail_0.weight
            @test fuselage.layout.cross_section.radius == fuselage_0.layout.cross_section.radius
            @test fuselage.layout.cross_section.radius == fuselage_radius
            @test landing_gear.wing_dihedral_angle == landing_gear_0.wing_dihedral_angle
        end
    end

    @testset "Ground margins after sizing" begin
        path_folding = joinpath(TASOPT.__TASOPTroot__, "../example/defaults/default_folding_wing.toml")
        sized_margins(ac) = (size_aircraft!(ac; printiter = false); ac.wing.fold)

        @testset "Without folding" begin
            ac = load_default_model()
            fold = sized_margins(ac)
            @test fold.ground_span == ac.wing.layout.span
            @test fold.span_margin == ac.wing.layout.max_span - ac.wing.layout.span
            @test fold.z_tip == 0.0
            @test fold.height_margin == 0.0
        end

        @testset "With folding" begin
            ac = read_aircraft_model(path_folding)
            fold = sized_margins(ac)
            @test fold.has_fold
            @test fold.ground_span ≈ fold.η_fold * ac.wing.layout.span rtol = 1e-12
            @test fold.span_margin ≈ ac.wing.layout.max_span - fold.ground_span rtol = 1e-12
            @test fold.z_tip > 0.0

            # Same margins as evaluating the sized aircraft again
            sized = deepcopy(fold)
            TASOPT.structures.fold_ground_margins!(ac.wing, ac.fuselage, ac.vtail, ac.landing_gear)
            @test fold.z_tip == sized.z_tip
            @test fold.height_margin == sized.height_margin
        end
    end

    @testset "Folding in wing_weights!" begin
        po, γt, γs, Nload, sigfac = 1.1e5, 0.225, 0.8666, 3.0, 1.0
        weights!(wing) = TASOPT.structures.wing_weights!(wing, po, γt, γs, Nload, 3e4, 0, 0.0, 0, 0.0, sigfac, 817.0)

        # Default wing with the geometry, loads and fold inputs filled in; the fold outputs carry the previous iteration
        function make_wing(; has_fold, has_strut = false, W_fold = 0.0)
            wing = deepcopy(TASOPT.load_default_model().wing)
            wing.layout.span, wing.layout.root_chord, wing.layout.root_span = 37.5, 6.2, 3.6
            wing.layout.ηs, wing.tip_lift_loss = 0.285, 0.05
            wing.inboard.λ, wing.outboard.λ = 0.7, 0.25
            wing.inboard.co, wing.outboard.co = 6.2, 6.2 * 0.7
            for section in (wing.inboard, wing.outboard)
                section.cross_section.width_to_chord, section.cross_section.web_to_box_height = 0.5, 0.75
            end
            wing.inboard.cross_section.thickness_to_chord, wing.outboard.cross_section.thickness_to_chord = 0.1268, 0.1266
            wing.inboard.weight, wing.outboard.weight, wing.inboard.dyW, wing.outboard.dyW = 5e4, 7e4, 8e4, 3.3e5
            wing.has_strut, wing.strut.z = has_strut, has_strut ? 3.9 : 0.0
            fold = wing.fold
            fold.has_fold, fold.η_fold, fold.n_hinge, fold.n_latch = has_fold, 0.7, 2, 3
            fold.actuator_torque_density, fold.systems_mass = 1e-3, 12.0
            fold.W_fold = W_fold
            fold.dyW_fold = W_fold / 2 * (fold.η_fold - wing.layout.ηs) * wing.layout.span / 2
            return wing
        end
        # Fixed wing carrying the loads of a sized folded wing: the fold relief is moved into the outboard weight inputs
        function relieved_fixed_wing(folded)
            fixed = make_wing(has_fold = false)
            fixed.outboard.weight += folded.fold.W_fold / 2
            fixed.outboard.dyW += folded.fold.dyW_fold
            return fixed
        end
        loads_of(wing) = (wing.outboard.max_shear_load, wing.outboard.moment,
                          wing.inboard.max_shear_load, wing.inboard.moment)

        @testset "Without folding the fold outputs are ignored and left untouched" begin
            fresh, junk = make_wing(has_fold = false), make_wing(has_fold = false, W_fold = 1e5)
            junk.fold.dyW_fold = 1e6
            @test weights!(fresh) == weights!(junk)
            @test loads_of(fresh) == loads_of(junk)
            @test (junk.fold.W_fold, junk.fold.dyW_fold, junk.fold.S_fold) == (1e5, 1e6, 0.0)
        end

        @testset "The returned fold weight relieves the outboard loads" begin
            folded, fixed = make_wing(has_fold = true), make_wing(has_fold = false)
            weights!(folded); weights!(fixed)
            @test folded.fold.W_fold > 0
            @test fixed.outboard.max_shear_load - folded.outboard.max_shear_load ≈ Nload * folded.fold.W_fold / 2 rtol = 1e-6
            @test fixed.outboard.moment - folded.outboard.moment ≈ Nload * folded.fold.dyW_fold rtol = 1e-6
            @test folded.inboard.max_shear_load < fixed.inboard.max_shear_load
            @test folded.inboard.moment < fixed.inboard.moment
        end

        @testset "Calling again with the same inputs leaves the fold weight unchanged" begin
            wing = make_wing(has_fold = true)
            weights!(wing)
            W_fold, dyW_fold, shear = wing.fold.W_fold, wing.fold.dyW_fold, wing.outboard.max_shear_load
            weights!(wing)
            @test wing.fold.W_fold ≈ W_fold rtol = 1e-6
            @test wing.fold.dyW_fold ≈ dyW_fold rtol = 1e-6
            @test wing.outboard.max_shear_load ≈ shear rtol = 1e-6
        end

        @testset "The returned fold weight relieves the strut" begin
            folded, fixed = make_wing(has_fold = true, has_strut = true), make_wing(has_fold = true, has_strut = true)
            fixed.fold.has_fold = false
            lsp = weights!(folded)[end]; weights!(fixed)
            @test lsp > 0
            @test fixed.strut.axial_force - folded.strut.axial_force ≈
                  Nload * folded.fold.W_fold / 2 * lsp / (folded.strut.z * folded.strut.material.σmax * sigfac) rtol = 1e-6
        end

        @testset "The fold mechanism is sized with the current outboard gauges" begin
            wing, expected = make_wing(has_fold = true, W_fold = 6e3), make_wing(has_fold = true, W_fold = 6e3)
            _, _, Wsout = weights!(wing)
            Vout = 6.2^2 * 37.5 * (1 - 0.285) * (0.7^2 + 0.7 * 0.25 + 0.25^2) / 6 * cosd(wing.layout.sweep)
            K_w = Wsout / Vout * (1 + TASOPT.structures.wing_additional_weight(wing))
            loads = TASOPT.structures.fold_station_loads(expected, po, γt, γs, Nload, K_w)
            TASOPT.structures.size_fold_mechanism!(expected, loads, sigfac, wing.outboard.caps.thickness)
            for name in (:S_fold, :M_fold, :F_cap, :W_fold, :dyW_fold)
                @test getfield(wing.fold, name) ≈ getfield(expected.fold, name) rtol = 1e-6
            end
        end

        @testset "Only the fuel of the outboard panel is reduced by the fold" begin
            folded = make_wing(has_fold = true)
            fold_out = weights!(folded)
            fixed = relieved_fixed_wing(folded)
            fixed_out = weights!(fixed)
            frac = TASOPT.structures.fold_tip_fuel_fraction(folded, folded.fold.η_fold)
            moment_frac = TASOPT.structures.fold_tip_fuel_moment_fraction(folded, folded.fold.η_fold)
            @test 0 < frac < 1
            @test fold_out[8] < fixed_out[8]
            @test fold_out[8] ≈ frac * fixed_out[8] rtol = 1e-6     # Wfout
            @test fold_out[10] ≈ frac * fixed_out[10] rtol = 1e-6    # dxWfout
            @test fold_out[12] ≈ moment_frac * fixed_out[12] rtol = 1e-6 # dyWfout
            # Wsinn, Wsout, dyWsinn, dyWsout, Wfcen, Wfinn, dxWfinn, dyWfinn keep the full panel
            for i in (2, 3, 4, 5, 6, 7, 9, 11)
                @test fold_out[i] ≈ fixed_out[i] rtol = 1e-6
            end
            @test folded.inboard.caps.weight.W ≈ fixed.inboard.caps.weight.W rtol = 1e-6
            @test folded.inboard.webs.weight.W ≈ fixed.inboard.webs.weight.W rtol = 1e-6
            @test folded.outboard.caps.thickness ≈ fixed.outboard.caps.thickness rtol = 1e-6
        end

        @testset "The fold mechanism weight is added to the wing weight outside the additional weight factor" begin
            folded = make_wing(has_fold = true)
            @test TASOPT.structures.wing_additional_weight(folded) > 0
            Wwing_fold = weights!(folded)[1]
            fixed = relieved_fixed_wing(folded)
            Wwing_fixed = weights!(fixed)[1]
            @test folded.fold.W_fold > 0
            @test Wwing_fold - Wwing_fixed ≈ folded.fold.W_fold rtol = 1e-6
            @test folded.dxW - fixed.dxW ≈ folded.fold.dxW_fold rtol = 1e-6
        end

        @testset "The fold mechanism moment about the root follows the sweep offset" begin
            dxW = Float64[]
            for sweep in (20.0, 30.0)
                wing = make_wing(has_fold = true)
                wing.layout.sweep = sweep
                weights!(wing)
                offset = (wing.fold.η_fold - wing.layout.ηo) * wing.layout.span / 2
                @test wing.fold.dxW_fold ≈ wing.fold.W_fold * offset * tand(sweep) rtol = 1e-12
                push!(dxW, wing.dxW)
            end
            @test dxW[1] != dxW[2]
        end

        @testset "A fold near the wing tip leaves the outboard fuel almost unchanged" begin
            folded = make_wing(has_fold = true)
            folded.fold.η_fold = 0.999
            @test TASOPT.structures.fold_tip_fuel_fraction(folded, 0.999) > 0.99
            folded_Wfout = weights!(folded)[8]
            @test folded_Wfout ≈ weights!(relieved_fixed_wing(folded))[8] rtol = 1e-2
        end
    end

    @testset "Fold text outputs" begin
        path_folding = joinpath(TASOPT.__TASOPTroot__, "../example/defaults/default_folding_wing.toml")
        text_of(print_function, ac) = (io = IOBuffer(); print_function(ac; io = io); String(take!(io)))
        line_of(text, label) = get(filter(line -> startswith(line, label), split(text, '\n')), 1, "")

        @testset "Without folding" begin
            ac = load_default_model()
            size_aircraft!(ac; printiter = false)
            weights_text = text_of(TASOPT.weight_buildup, ac)
            geometry_text = text_of(TASOPT.geometry, ac)
            @test !occursin("Wfold", weights_text)
            @test !occursin("Folding Wingtip", geometry_text)
        end

        @testset "With folding" begin
            ac = read_aircraft_model(path_folding)
            size_aircraft!(ac; printiter = false)
            fold = ac.wing.fold

            weights_text = text_of(TASOPT.weight_buildup, ac)
            for (label, weight) in (("Wfold", fold.W_fold), ("Whinge", fold.W_hinge_lugs),
                                    ("Wlatch", fold.W_latches), ("Wlintro", fold.W_load_intro),
                                    ("Wactua", fold.W_actuator))
                @test occursin(@sprintf("%10.1f", weight), line_of(weights_text, label))
            end

            geometry_text = text_of(TASOPT.geometry, ac)
            @test occursin("Folding Wingtip", geometry_text)
            @test occursin(@sprintf("%.1f", fold.ground_span), line_of(geometry_text, "ground span"))
            @test occursin(@sprintf("%.1f", fold.span_margin), line_of(geometry_text, "span margin"))
            @test occursin(@sprintf("%.1f", fold.z_tip), line_of(geometry_text, "z tip"))
            @test occursin(@sprintf("%.1f", fold.height_margin), line_of(geometry_text, "height margin"))
        end
    end

    @testset "size_aircraft! with folding on" begin
        path_folding = joinpath(TASOPT.__TASOPTroot__, "../example/defaults/default_folding_wing.toml")
        ac = read_aircraft_model(path_folding)
        size_aircraft!(ac; printiter = false)
        fold = ac.wing.fold

        @testset "Reference values" begin
            @test ac.parg[igWMTO] ≈ 929241.2424193665 rtol = 1e-4
            @test ac.parm[imPFEI] ≈ 0.9849599729832988 rtol = 1e-4

            reference = (S_fold = 219790.45755465538, M_fold = 554954.3095526628, F_cap = 5.035504501348088e6,
                         W_hinge_lugs = 348.73826955465154, W_latches = 201.05691781192618,
                         W_load_intro = 1606.0332953620766, W_actuator = 9129.033837019731, W_fold = 11284.862319748387,
                         dyW_fold = 45302.13179612856, dxW_fold = 64612.54440237243,
                         ground_span = 27.085216350825412, span_margin = 8.728783649174588,
                         z_tip = 4.27519812999302, height_margin = 5.976101123648716)
            for (name, value) in pairs(reference)
                @test getfield(fold, name) ≈ value rtol = 1e-4
            end
        end

        @testset "Relations between the results" begin
            @test fold.W_fold > 0.0
            components = fold.W_hinge_lugs + fold.W_latches + fold.W_load_intro + fold.W_actuator
            @test fold.W_fold ≈ components + 2 * TASOPT.gee * fold.systems_mass rtol = 1e-12

            # The folded tip panel holds no fuel, so the tank is smaller than for the default model with folding off
            ac_no_fold = load_default_model()
            size_aircraft!(ac_no_fold; printiter = false)
            @test ac.parg[igWfmax] < ac_no_fold.parg[igWfmax]
        end
    end

    @testset "Folding wingtip parameter sweeps" begin
        # The script only defines fold_sweep when included, its sweeps run when it is executed directly
        include(joinpath(TASOPT.__TASOPTroot__, "../example/folding_wingtip_sweeps.jl"))
        path_folding = joinpath(TASOPT.__TASOPTroot__, "../example/defaults/default_folding_wing.toml")
        sweep(ac, values, apply!) = Base.invokelatest(fold_sweep, ac, values, apply!)

        @testset "Fold location sweep" begin
            ac = read_aircraft_model(path_folding)
            WMTO_before = ac.parg[igWMTO]
            η_before = ac.wing.fold.η_fold

            rows = sweep(ac, [0.7, 0.85], (a, v) -> a.wing.fold.η_fold = v)
            @test length(rows) == 2
            @test [row.value for row in rows] == [0.7, 0.85]

            # The swept model is a copy, so the input model is not sized or changed
            @test ac.wing.fold.η_fold == η_before
            @test ac.parg[igWMTO] == WMTO_before

            ac_direct = read_aircraft_model(path_folding)
            ac_direct.wing.fold.η_fold = 0.7
            size_aircraft!(ac_direct; printiter = false)
            @test rows[1].W_fold ≈ ac_direct.wing.fold.W_fold rtol = 1e-10
            @test rows[1].WMTO ≈ ac_direct.parg[igWMTO] rtol = 1e-10

            for row in rows
                @test row.W_fold ≈ row.W_hinge_lugs + row.W_latches + row.W_load_intro + row.W_actuator +
                                   2 * TASOPT.gee * ac.wing.fold.systems_mass rtol = 1e-12
            end
        end

        @testset "Hinge count sweep" begin
            ac = read_aircraft_model(path_folding)
            rows = sweep(ac, [1, 2], (a, v) -> a.wing.fold.n_hinge = v)
            @test rows[2].W_hinge_lugs < rows[1].W_hinge_lugs
        end
    end
end
