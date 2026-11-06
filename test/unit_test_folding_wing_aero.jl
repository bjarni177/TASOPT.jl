@testset "Folding wing aerodynamics" verbose=true begin
    TOML = TASOPT.TOML
    Logging = Test.Logging

    @testset "Hinge drag from input" begin
        # Read default_folding_wing.toml with delta_CD_fold_counts overridden
        function read_with_drag_counts(counts)
            path_folding = joinpath(TASOPT.__TASOPTroot__, "../example/defaults/default_folding_wing.toml")
            input = TOML.parsefile(path_folding)
            input["Wing"]["FoldingTip"]["delta_CD_fold_counts"] = counts
            mktempdir() do dir
                path = joinpath(dir, "modified_input.toml")
                open(io -> TOML.print(io, input), path, "w")
                Logging.with_logger(Logging.NullLogger()) do
                    read_aircraft_model(path)
                end
            end
        end

        # One drag count is 1e-4
        @test read_with_drag_counts(7.0).wing.fold.CD_fold ≈ 7e-4
        @test read_with_drag_counts(0.0).wing.fold.CD_fold == 0.0
        @test load_default_model().wing.fold.CD_fold == 0.0
    end

    @testset "Hinge drag in aircraft_drag!" begin
        ac = load_default_model()
        size_aircraft!(ac; printiter = false)
        fold = ac.wing.fold

        # Total CD at the first cruise point for the given hinge drag
        function total_CD(has_fold, CD_fold)
            fold.has_fold = has_fold
            fold.CD_fold = CD_fold
            TASOPT.aerodynamics.aircraft_drag!(ac, 1, ipcruise1, true)
            return ac.para[iaCD, ipcruise1, 1]
        end

        # With folding on, CD_fold is added to the total CD
        @test total_CD(true, 5e-4) - total_CD(true, 0.0) ≈ 5e-4 rtol = 1e-9

        # With folding off, CD_fold is ignored
        @test total_CD(false, 5e-4) == total_CD(false, 0.0)
    end

    @testset "Hinge drag in aeroperf_sweep" begin
        ac = load_default_model()
        size_aircraft!(ac; printiter = false)
        fold = ac.wing.fold
        fold.has_fold = true

        fold.CD_fold = 0.0
        without_drag = aeroperf_sweep(ac, [0.8])
        fold.CD_fold = 5e-4
        with_drag = aeroperf_sweep(ac, [0.8])

        # The hinge drag is reported in the total and in the other drag
        @test with_drag.CDs[1] - without_drag.CDs[1] ≈ 5e-4 rtol = 1e-9
        @test with_drag.CDothers[1] - without_drag.CDothers[1] ≈ 5e-4 rtol = 1e-9
    end
end
