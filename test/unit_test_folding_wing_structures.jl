@testset "Folding wing structures" verbose=true begin
    database = TASOPT.materials.MaterialProperties

    @testset "FoldingWingTip defaults" begin
        fold = TASOPT.structures.FoldingWingTip()

        # Folding is off and the counts and factors are zero, so that a missing input fails the input checks
        @test fold.has_fold == false
        @test fold.n_hinge == 0
        @test fold.n_latch == 0
        @test fold.fitting_factor == 0.0

        # No NaN in any numeric field
        for name in fieldnames(typeof(fold))
            value = getfield(fold, name)
            if value isa Float64
                @test isfinite(value)
            end
        end

        # Outputs start at zero and flags at false
        @test fold.W_fold == 0.0
        @test fold.S_fold == 0.0
        @test fold.M_fold == 0.0
        @test fold.CD_fold == 0.0
        @test fold.span_margin == 0.0
        @test fold.tail_flag == false

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

        # Everything not overridden keeps its default
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

        # Folding stays off after sizing the default aircraft
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
            @test fold.n_latch == 1
            @test fold.fitting_factor == 1.15
            @test fold.ground_wind_speed ≈ 65 * 0.514444 rtol = 1e-4
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
            @test_throws ErrorException read_with_fold_inputs(has_fold = true, fitting_factor = 0.9)

            # Not checked when folding is off
            fold = read_with_fold_inputs(has_fold = false, n_hinge = 0, fitting_factor = 0.9).wing.fold
            @test fold.n_hinge == 0
            @test fold.fitting_factor == 0.9
        end

        @testset "Inputs without a FoldingTip block" begin
            for name in ("default_regional.toml", "default_wide.toml")
                test_starting_values(read_quietly(joinpath(defaults_dir, name)).wing.fold)
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
end
