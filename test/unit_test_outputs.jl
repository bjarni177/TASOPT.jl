@testset "outputs" verbose=true begin

    ac = load_default_model()
    size_aircraft!(ac; printiter=false);

    file_weights = joinpath(TASOPT.__TASOPTroot__, "../test/weights.txt")
    file_aero = joinpath(TASOPT.__TASOPTroot__, "../test/aero.txt")
    file_geom = joinpath(TASOPT.__TASOPTroot__, "../test/geom.txt")
    
    @testset "output summaries" begin
        f = open(io->TASOPT.weight_buildup(ac,io=io), "temp.txt", "w")
        content = read(file_weights, String)
        content = replace(content, "\r\n"=> "\n") # Convert \r\n to \n
        @test content == read("temp.txt", String)
        rm("temp.txt")

        f = open(io->TASOPT.aero(ac,io=io), "temp.txt", "w")
        content = read(file_aero, String)
        content = replace(content, "\r\n"=> "\n") # Convert \r\n to \n
        @test content == read("temp.txt", String)
        rm("temp.txt")

        f = open(io->TASOPT.geometry(ac,io=io), "temp.txt", "w")
        content = read(file_geom, String)
        content = replace(content, "\r\n"=> "\n") # Convert \r\n to \n
        @test content == read("temp.txt", String)
        rm("temp.txt")
    end

    @testset "output plots" begin
        
        p1 = TASOPT.stickfig(ac)
        @test p1 isa Plots.Plot
        p2 = TASOPT.plot_details(ac)
        @test p2 isa Plots.Plot
        p3 = TASOPT.plot737compare(ac)
        @test p3 isa Plots.Plot
        p4 = TASOPT.MomentShear(ac)
        @test p4 isa Plots.Plot
        # the title shows the governing case; the plot also works without evaluated cases
        function title_text(p)
            join([get(p.attr, :plot_title, ""); [sp[:title] for sp in p.subplots]], " ")
        end
        ac_cases = deepcopy(ac)
        sl = ac_cases.wing.sizing_loads
        sl.results[:pushover] = TASOPT.structures.WingCaseLoads(S_root = 5.0e5, M_root = 6.0e6, S_break = 7.0e4, M_break = 8.0e5)
        sl.results[:taxi] = TASOPT.structures.WingCaseLoads(S_root = 1.0e5, M_root = 2.0e6, S_break = 3.0e4, M_break = 4.0e5)
        sl.governing_root, sl.governing_break = :pushover, :taxi
        p4_cases = TASOPT.MomentShear(ac_cases)
        @test p4_cases isa Plots.Plot
        @test occursin("pushover", title_text(p4_cases))
        @test occursin("taxi", title_text(p4_cases))

        empty!(sl.results)
        @test TASOPT.MomentShear(ac_cases) isa Plots.Plot

        p5 = TASOPT.PayloadRange(ac)
        @test p5 isa Plots.Plot
        p6 = DragPolar(ac)
        @test p6 isa Plots.Plot
        p7 = plot_airf(ac)
        @test p7 isa Plots.Plot
        p8 = plot_drag_breakdown(ac, show_values=true)
        p9 = plot_drag_breakdown(ac, ip = :, show_fractions=false)
        @test p8 isa Plots.Plot
        @test p9 isa Plots.Plot
    end

end