using TASOPT, NLopt

isdefined(Main, :igAR) || include(TASOPT.__TASOPTindices__)

# ================================================================
# FUNCTION SIGNATURES PROVIDED BY THIS MODULE:
#
# optimize_aircraft!(ac; maxeval=150, ftol_rel=1e-5) 
#     -> (return_code::Symbol, pfei::Float64)
#
# evaluate_fuel_burn_envelope(ac; payload_fraction=0.8, step_km=100.0, itermax=35) 
#     -> NamedTuple with fields (range_km, fuel_burn_MJ, converged, payload_N, design_range_km)
# ================================================================

"""
    optimize_aircraft!(ac; maxeval=150, ftol_rel=1e-5) -> (return_code::Symbol, pfei::Float64)

Runs an NLopt derivative-free (LN_NELDERMEAD) design optimization on `ac`
IN PLACE, minimizing PFEI (`ac.parm[imPFEI,1]`) subject to penalty-based
constraints, following EXACTLY the pattern in `example/example_opt.jl`.

Design variables (14): AR, cruise CL, sweep [deg], cruise altitude [m], 
inboard taper λ, outboard taper λ, root t/c, spanbreak t/c, rcls, rclt, 
Tt4 [K], pihc, pif, BPR.

Initial values and bounds are read directly from `ac`'s current state,
using fixed relative/absolute bound offsets where applicable, ensuring
the function works across different aircraft architectures and fuel types.

Penalty constraints (6): max span, min climb gradient, max Tt3, max fuel 
volume, max fan diameter, max balanced field length.

Calls `size_aircraft!(ac; iter=50, printiter=false)` inside a try/catch 
in the objective, returning 1e6 penalty on exception.

Returns `(return_code, pfei)` where `return_code` is the Symbol from 
`NLopt.optimize` (e.g. `:SUCCESS`, `:FTOL_REACHED`, `:MAXEVAL_REACHED`, 
`:FAILURE`) and `pfei` is `ac.parm[imPFEI,1]` after optimization, with `ac` 
left in its best-found design state. If the final re-apply+re-size fails, 
returns `(:REOPT_FAILED, Inf)`.

Arguments:
  - `ac`: aircraft struct to optimize (modified in place)
  - `maxeval`: maximum function evaluations (default 150 for batch speed)
  - `ftol_rel`: relative objective function tolerance (default 1e-5)

Returns: Tuple `(return_code::Symbol, pfei::Float64)`

Example:
  ```
  ac = TASOPT.read_aircraft_model("default_wide.toml")
  size_aircraft!(ac)
  (ret, pfei) = optimize_aircraft!(ac; maxeval=200, ftol_rel=1e-4)
  println("Optimization returned \$ret with PFEI = \$pfei")
  ```
"""
function optimize_aircraft!(ac; maxeval=150, ftol_rel=1e-5)
    # Extract initial values from ac's current state
    ac_initial_AR = ac.wing.layout.AR
    ac_initial_CL = ac.para[iaCL, ipcruise1, 1]
    ac_initial_sweep = ac.wing.layout.sweep
    ac_initial_alt = ac.para[iaalt, ipcruise1, 1]
    ac_initial_lambda_in = ac.wing.inboard.λ
    ac_initial_lambda_out = ac.wing.outboard.λ
    ac_initial_tc_root = ac.wing.inboard.cross_section.thickness_to_chord
    ac_initial_tc_span = ac.wing.outboard.cross_section.thickness_to_chord
    ac_initial_rcls = ac.para[iarcls, ipcruise1, 1]
    ac_initial_rclt = ac.para[iarclt, ipcruise1, 1]
    ac_initial_Tt4 = ac.pare[ieTt4, ipcruise1, 1]
    ac_initial_pihc = ac.pare[iepihc, ipcruise1, 1]
    ac_initial_pif = ac.pare[iepif, ipcruise1, 1]
    ac_initial_BPR = ac.pare[ieBPR, ipcruise1, 1]

    initial = [
        ac_initial_AR,
        ac_initial_CL,
        ac_initial_sweep,
        ac_initial_alt,
        ac_initial_lambda_in,
        ac_initial_lambda_out,
        ac_initial_tc_root,
        ac_initial_tc_span,
        ac_initial_rcls,
        ac_initial_rclt,
        ac_initial_Tt4,
        ac_initial_pihc,
        ac_initial_pif,
        ac_initial_BPR,
    ]

    # Hardcoded reference bounds (from example/example_gradient_based_opt.jl,
    # tuned around the default widebody aircraft). These are NOT safe to
    # apply as absolute bounds to an arbitrary loaded aircraft: a different
    # aircraft's actual starting design point (e.g. a regional aircraft with
    # a lower Tt4/pihc) can fall outside them. Clipping `initial` into such
    # bounds can simultaneously move several engine/aero parameters into an
    # untested, physically infeasible corner (verified: this previously
    # caused `TFSIZE: Negative core plume velocity` errors on every single
    # NLopt objective evaluation for the regional aircraft, since Tt4 was
    # clipped from 1327 K up to 1400 K while pihc was simultaneously clipped
    # from 3.5 up to 10.0). Instead, each bound is WIDENED (never narrowed)
    # to guarantee it always brackets this aircraft's own actual initial
    # value, so `initial` never needs to be clipped and the optimizer always
    # starts from a point we already know is physically valid.
    ref_lower = [
        max(6.0, 0.5 * ac_initial_AR),       # AR
        0.45,                                 # CL
        25.0,                                 # sweep [deg]
        10000.0,                              # altitude [m]
        0.65,                                 # lambda_in
        0.1,                                  # lambda_out
        0.125,                                # tc_root
        0.125,                                # tc_span
        0.9,                                  # rcls
        0.7,                                  # rclt
        1400.0,                               # Tt4 [K]
        10.0,                                 # pihc
        1.25,                                 # pif
        1.0,                                  # BPR
    ]

    ref_upper = [
        min(18.0, 1.5 * ac_initial_AR),      # AR
        0.75,                                 # CL
        30.0,                                 # sweep [deg]
        20000.0,                              # altitude [m]
        0.85,                                 # lambda_in
        0.4,                                  # lambda_out
        0.15,                                 # tc_root
        0.15,                                 # tc_span
        1.3,                                  # rcls
        1.0,                                  # rclt
        1650.0,                               # Tt4 [K]
        15.0,                                 # pihc
        2.0,                                  # pif
        20.0,                                 # BPR
    ]

    lower = min.(ref_lower, initial)
    upper = max.(ref_upper, initial)

    # Initial step sizes
    initial_dx = [0.5, 0.05, 0.1, 200.0, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 100.0, 0.5, 0.05, 1.0]

    # Variables to track best result across objective calls
    best_x = copy(initial)
    best_f = Inf

    # Objective function closure
    function obj(x, grad)
        wing = ac.wing
        wing.layout.AR = x[1]
        wing.layout.sweep = x[3]
        wing.inboard.λ = x[5]
        wing.outboard.λ = x[6]
        wing.inboard.cross_section.thickness_to_chord = x[7]
        wing.outboard.cross_section.thickness_to_chord = x[8]

        ac.para[iaCL, ipcruise1:ipcruise2, 1] .= x[2]
        ac.para[iaalt, ipcruise1:ipcruise2, 1] .= x[4]
        ac.para[iarcls, ipcruise1:ipcruise2, 1] .= x[9]
        ac.para[iarclt, ipcruise1:ipcruise2, 1] .= x[10]

        ac.pare[ieTt4, ipcruise1:ipcruise2, 1] .= x[11]
        ac.pare[iepihc, ipcruise1, 1] = x[12]
        ac.pare[iepif, ipcruise1, 1] = x[13]
        ac.pare[ieBPR, ipcruise1, 1] = x[14]

        # Size aircraft, return large penalty on failure
        try
            TASOPT.size_aircraft!(ac, iter=50, printiter=false)
        catch
            return 1e6
        end

        # Extract objective (PFEI)
        f = ac.parm[imPFEI, 1]

        # Apply penalty constraints (same 6 as example_opt.jl)
        total_penalty = 0.0

        # 1. Maximum span constraint
        bmax = ac.wing.layout.max_span
        b = ac.wing.span
        if b > bmax
            constraint = b / bmax - 1.0
            penalty = 25.0 * ac.parg[igWpay] * constraint^2
            total_penalty += penalty
        end

        # 2. Minimum climb gradient constraint
        gtocmin = ac.parg[iggtocmin]
        gtoc = ac.para[iagamV, ipclimbn, 1]
        if gtoc < gtocmin
            constraint = 1.0 - gtoc / gtocmin
            penalty = 1.0 * ac.parg[igWpay] * constraint^2
            total_penalty += penalty
        end

        # 3. Maximum turbine temperature constraint
        Tt3max = 900.0
        Tt3 = maximum(ac.pare[ieTt3, :, 1])
        if Tt3 > Tt3max
            constraint = Tt3 / Tt3max - 1.0
            penalty = 5.0 * ac.parg[igWpay] * constraint^2
            total_penalty += penalty
        end

        # 4. Fuel volume constraint
        Wfmax = ac.parg[igWfmax]
        Wf = ac.parg[igWfuel]
        if Wf > Wfmax
            constraint = Wf / Wfmax - 1.0
            penalty = 10.0 * ac.parg[igWpay] * constraint^2
            total_penalty += penalty
        end

        # 5. Maximum fan diameter constraint
        dfanmax = 2.0
        dfan = ac.parg[igdfan]
        if dfan > dfanmax
            constraint = dfan / dfanmax - 1.0
            penalty = ac.parg[igWpay] * constraint^2
            total_penalty += penalty
        end

        # 6. Maximum balanced field length constraint
        lBF = ac.parm[imlBF]
        lBFmax = 2.4e3
        if lBF > lBFmax
            constraint = lBF / lBFmax - 1.0
            penalty = ac.parg[igWpay] * constraint^2
            total_penalty += penalty
        end

        f_total = f + total_penalty

        # Track best result
        if f_total < best_f
            best_f = f_total
            best_x = copy(x)
        end

        return f_total
    end

    # Set up NLopt optimizer
    opt = NLopt.Opt(:LN_NELDERMEAD, length(initial))
    opt.lower_bounds = lower
    opt.upper_bounds = upper
    opt.min_objective = obj
    opt.initial_step = initial_dx
    opt.ftol_rel = ftol_rel
    opt.maxeval = maxeval

    # Run optimization
    (optf, optx, ret) = NLopt.optimize(opt, initial)

    # Reapply best design and re-size one final time (outside try/catch-guarded objective)
    # to ensure ac's final state is consistent with optimum
    try
        wing = ac.wing
        wing.layout.AR = best_x[1]
        wing.layout.sweep = best_x[3]
        wing.inboard.λ = best_x[5]
        wing.outboard.λ = best_x[6]
        wing.inboard.cross_section.thickness_to_chord = best_x[7]
        wing.outboard.cross_section.thickness_to_chord = best_x[8]

        ac.para[iaCL, ipcruise1:ipcruise2, 1] .= best_x[2]
        ac.para[iaalt, ipcruise1:ipcruise2, 1] .= best_x[4]
        ac.para[iarcls, ipcruise1:ipcruise2, 1] .= best_x[9]
        ac.para[iarclt, ipcruise1:ipcruise2, 1] .= best_x[10]

        ac.pare[ieTt4, ipcruise1:ipcruise2, 1] .= best_x[11]
        ac.pare[iepihc, ipcruise1, 1] = best_x[12]
        ac.pare[iepif, ipcruise1, 1] = best_x[13]
        ac.pare[ieBPR, ipcruise1, 1] = best_x[14]

        TASOPT.size_aircraft!(ac, iter=50, printiter=false)
        pfei = ac.parm[imPFEI, 1]
        return (ret, pfei)
    catch
        return (:REOPT_FAILED, Inf)
    end
end

"""
    evaluate_fuel_burn_envelope(ac; payload_fraction=0.8, step_km=100.0, itermax=35) 
        -> NamedTuple

Evaluates the fuel-burn performance envelope for an already-sized aircraft `ac`
by sweeping a range of mission ranges and computing the fuel burn at each point.

`ac` MUST be sized (ac.is_sized[1] == true) before calling. Creates an off-design
mission-2 aircraft copy following the PayloadRange-derived pattern in the spec,
duplicating parm/para/pare slices and engine.heat_exchangers[i].HXgas_mission if present.

Sweeps range_km over 100:step_km:design_range_km, appending the exact design_range_km
as a final point if not already included (via floating-point-safe !isapprox check).
Payload is the FIXED absolute value payload_fraction * ac.parg[igWpaymax] (Newtons),
held constant across all points.

For each range point: sets ac_od.parm[imRange,2] and ac_od.parm[imWpay,2],
resets cruise altitude and CL to design values, calls fly_mission!(ac_od, 2; 
itermax=itermax, printTO=false) inside try/catch. On success, checks feasibility 
(WTO <= WMTO + 1.0 N, Wfuel <= Wfmax + 1.0 N, both weights >= 0) and computes 
fuel_burn_MJ via Wburn=Wfuel/(1+freserve), *LHVfuel/gee/1e6. On failure or 
infeasibility, sets fuel_burn_MJ[i] = NaN, converged[i] = false, continues 
without aborting.

Arguments:
  - `ac`: already-sized aircraft struct
  - `payload_fraction`: fraction of max payload capacity to use (default 0.8)
  - `step_km`: range step size in km (default 100.0)
  - `itermax`: max iterations for fly_mission! (default 35)

Returns: NamedTuple with fields
  - `range_km::Vector{Float64}`: range points (km)
  - `fuel_burn_MJ::Vector{Float64}`: fuel burn at each point (MJ, burned fuel excl. reserve)
  - `converged::Vector{Bool}`: convergence flag per point
  - `payload_N::Float64}`: absolute payload weight used (N)
  - `design_range_km::Float64}`: design range (km)

Example:
  ```
  ac = TASOPT.read_aircraft_model("default_wide.toml")
  size_aircraft!(ac)
  env = evaluate_fuel_burn_envelope(ac; payload_fraction=0.8, step_km=100.0)
  println("Envelope has \$(length(env.range_km)) points")
  println("Design range: \$(env.design_range_km) km")
  ```
"""
function evaluate_fuel_burn_envelope(ac; payload_fraction=0.8, step_km=100.0, itermax=35)
    # Design range in km
    design_range_m = ac.parg[igRange]
    design_range_km = design_range_m / 1e3

    # Build range sweep: 100:step_km:design_range_km, then append design_range_km if not already included
    range_points_km = collect(100.0:step_km:design_range_km)
    if !isapprox(range_points_km[end], design_range_km; rtol=1e-9, atol=1e-6)
        push!(range_points_km, design_range_km)
    end

    # Fixed payload (Newtons)
    payload_N = payload_fraction * ac.parg[igWpaymax]

    # Preallocate output arrays
    n_points = length(range_points_km)
    fuel_burn_MJ = Vector{Float64}(undef, n_points)
    converged = Vector{Bool}(undef, n_points)

    # Create off-design aircraft copy (mission 2)
    parm = cat(ac.parm[:, 1], ac.parm[:, 1], dims=2)
    pare = cat(ac.pare[:, :, 1], ac.pare[:, :, 1], dims=3)
    para = cat(ac.para[:, :, 1], ac.para[:, :, 1], dims=3)

    ac_od = TASOPT.aircraft(
        ac.name, ac.description, ac.options, ac.parg,
        parm, para, pare, [true], ac.fuselage, ac.fuse_tank, ac.wing,
        ac.htail, ac.vtail, ac.engine, ac.landing_gear
    )

    # Duplicate heat_exchangers if present
    for HX in ac_od.engine.heat_exchangers
        HX.HXgas_mission = cat(HX.HXgas_mission[:, 1], HX.HXgas_mission[:, 1], dims=2)
    end

    # Sweep range points
    for (i, range_km) in enumerate(range_points_km)
        # Set range and payload for mission 2
        ac_od.parm[imRange, 2] = range_km * 1e3  # Convert to meters
        ac_od.parm[imWpay, 2] = payload_N

        # Reset cruise altitude and CL to design values
        ac_od.para[iaalt, ipcruise1, 2] = ac_od.para[iaalt, ipcruise1, 1]
        ac_od.para[iaCL, ipcruise1, 2] = ac_od.para[iaCL, ipcruise1, 1]

        # Attempt to fly the mission
        try
            TASOPT.fly_mission!(ac_od, 2; itermax=itermax, printTO=false)

            # Check feasibility
            WTO_2 = ac_od.parm[imWTO, 2]
            WMTO_2 = ac_od.parg[igWMTO]
            Wfuel_2 = ac_od.parm[imWfuel, 2]
            Wfmax_2 = ac_od.parg[igWfmax]

            if WTO_2 <= WMTO_2 + 1.0 && Wfuel_2 <= Wfmax_2 + 1.0 && 
               WTO_2 >= 0.0 && Wfuel_2 >= 0.0

                # Compute fuel burn (excluding reserve)
                freserve = ac.parg[igfreserve]
                Wburn_2_N = Wfuel_2 / (1 + freserve)
                LHVfuel = ac.parg[igLHVfuel]
                fuel_burn_MJ[i] = (Wburn_2_N / TASOPT.gee) * LHVfuel / 1e6

                converged[i] = true
            else
                # Infeasible point
                fuel_burn_MJ[i] = NaN
                converged[i] = false
            end
        catch
            # Mission failed to converge
            fuel_burn_MJ[i] = NaN
            converged[i] = false
        end
    end

    return (
        range_km=range_points_km,
        fuel_burn_MJ=fuel_burn_MJ,
        converged=converged,
        payload_N=payload_N,
        design_range_km=design_range_km
    )
end
