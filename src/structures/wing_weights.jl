"""
    WingSectionDimensions{T<:Real}

Structural dimensions of a sized wing spar-box section.
Fields are ordered as: thicknesses first (web, cap), then areas (web, cap).

!!! details "Fields"
    - `tbweb::T`: Normalized Web thickness
    - `tbcap::T`: Normalized Cap thickness
    - `Abweb::T`: Normalized Web cross-sectional area (both webs combined)
    - `Abcap::T`: Normalized Cap cross-sectional area (both caps combined)
"""
struct WingSectionDimensions{T<:Real}
    tbweb::T
    tbcap::T
    Abweb::T
    Abcap::T
end

"""
    size_wing_section!(section, sweep, sigfac)

Calculates Loads and thicknesses for wing sections

!!! details "🔃 Inputs and Outputs"
    **Inputs:**
    - `section::TASOPT.structures.Wing.WingSection`: Wing Section to be sized.
    - `sweep::Float64`: Wing sweep.
    - `sigfac::Float64`: Stress factor
    - `t_cap_min::Float64`: (keyword, default 0.0) Minimum cap thickness
    - `t_web_min::Float64`: (keyword, default 0.0) Minimum web thickness

    **Outputs:** [`WingSectionDimensions`](@ref)
"""
function size_wing_section!(section, sweep, sigfac; t_cap_min = 0.0, t_web_min = 0.0)
    shear_load = section.max_shear_load
    moment = section.moment

    cross_section = section.cross_section

    Eweb = section.webs.material.E
    Ecap = section.caps.material.E
    Gcap = section.caps.material.G
    Gweb = section.webs.material.G
    sigcap = section.caps.material.σmax * sigfac
    tauweb = section.webs.material.τmax * sigfac

    cs = section.co
    cosL = cosd(sweep)
    cp = cs*cosL

    h_avg, h_rms = get_average_sparbox_heights(section.cross_section)

    web_height = cross_section.web_to_box_height * cross_section.thickness_to_chord

    tbweb, Abweb = size_web(tauweb, shear_load, cp, web_height; t_min = t_web_min)
    
    tbcap, Abcap = size_cap(sigcap, moment, cross_section.thickness_to_chord,
        cross_section.width_to_chord, h_rms, cs, cosL; t_min = t_cap_min)

    # EI_xx
    section.EI[1] = Ecap * cp^4 * (h_rms^3 - (h_rms - 2.0 * tbcap)^3) * cross_section.width_to_chord / 12.0 +
                                 Eweb * cp^4 * tbweb * web_height^3 / 6.0
    # EI_yy 
    section.EI[4] = Ecap * cp^4 * tbcap * cross_section.width_to_chord^3 / 6.0 +
                                Eweb * cp^4 * tbweb * web_height * 0.5 * cross_section.width_to_chord^2
    # println("GJ SURFW: cp = $cp, width_to_chord = $(cross_section.width_to_chord), tbweb = $tbweb, h_avg = $h_avg, tbcap = $tbcap, web_to_box_height = $(cross_section.web_to_box_height), thickness_to_chord = $(cross_section.thickness_to_chord), Gweb = $Gweb, Gcap = $Gcap")
    section.GJ = cp^4 * 2.0*((cross_section.width_to_chord-tbweb)*(h_avg-tbcap))^2 /
        (  (cross_section.web_to_box_height*section.cross_section.thickness_to_chord-tbcap)/(Gweb*tbweb) +
        (   cross_section.width_to_chord -tbweb)/(Gcap*tbcap) )

    return WingSectionDimensions(tbweb, tbcap, Abweb, Abcap)
end

"""
    Cantilever

Marker type for a wing without a strut. Its inboard loads follow from the outboard loads and the
inboard panel weights; see [`wing_section_loads`](@ref).
"""
struct Cantilever end

"""
    outboard_section_loads!(wing, po, gammat, gammas, Nload, We, neout, dyeout)

Sets the outboard section shear and bending moment at the wing break (strut attachment point), from the
outer-wing lift distribution, the tip roll-off lift loss, the outboard panel weight and any outboard engines.

!!! details "🔃 Inputs and Outputs"
    **Inputs:**
    - `wing::TASOPT.structures.Wing`: Wing structure.
    - `po::Float64`: Wing root loading magnitude.
    - `gammat::Float64`: Wing outer section taper ratio.
    - `gammas::Float64`: Wing inner section taper ratio.
    - `Nload::Int`: Max vertical load factor for wing bending loads.
    - `We::Float64`: Weight of one engine [N].
    - `neout::Int`: Number of engines per wing mounted outboard of the wing break.
    - `dyeout::Float64`: Average spanwise moment arm of the outboard engines about the wing break station ηs [m].

    **Outputs:**
    - `wing.outboard.max_shear_load` and `wing.outboard.moment` are updated in place.
"""
function outboard_section_loads!(wing, po, gammat, gammas, Nload, We, neout, dyeout)
    etas = wing.layout.ηs

    # Tip roll off Lift (modeled as a point load) and it's moment about ηs
    dLt = wing.tip_lift_loss * po * wing.layout.root_chord * gammat * wing.outboard.λ
    dMt = dLt * 0.5 * wing.layout.span * (1.0 - etas)

    # Outboard section:
    #---- strut-attach shear,moment from outer-wing loading. 
    #     Note added term to account for any outboard engines.
    #     If neout = 0 this simplifies to Drela's version which assumes engine
    #     fixed at ηs locations only.
    wing.outboard.max_shear_load = (po * wing.layout.span / 4.0) * (gammas + gammat) * (1.0 - etas) +
                                   dLt - Nload * wing.outboard.weight - Nload * neout * We
    wing.outboard.moment = (po * wing.layout.span^2 / 24.0) * (gammas + 2.0 * gammat) * (1.0 - etas)^2 +
                           dMt - Nload * wing.outboard.dyW - Nload * neout * We * dyeout
    return nothing
end # outboard_section_loads!

"""
    wing_section_loads(::Cantilever, wing, po, gammas, Nload, We, neinn, dyeinn)

Sets the inboard section shear and bending moment of a wing without a strut, from the outboard
section loads (`wing.outboard.max_shear_load`, `wing.outboard.moment`), the inboard lift distribution
and the inboard panel weight, including bending relief from inboard engines.

!!! details "🔃 Inputs and Outputs"
    **Inputs:**
    - `wing::TASOPT.structures.Wing`: Wing structure, with the outboard section loads already set.
    - `po::Float64`: Wing root loading magnitude.
    - `gammas::Float64`: Wing inner section taper ratio.
    - `Nload::Int`: Max vertical load factor for wing bending loads.
    - `We::Float64`: Weight of one engine [N].
    - `neinn::Int`: Number of engines per wing mounted at or inboard of the wing break.
    - `dyeinn::Float64`: Average spanwise moment arm of the inboard engines about the wing root station ηo [m].

    **Outputs:**
    - `wing.inboard.max_shear_load` and `wing.inboard.moment` are updated in place.
"""
function wing_section_loads(::Cantilever, wing, po, gammas, Nload, We, neinn, dyeinn)
    etao = wing.layout.ηo
    etas = wing.layout.ηs

    # Modifed to account for bending relief from multiple engines.
    # dyeinn allows engine to be in locations other than ηs
    So = wing.outboard.max_shear_load - Nload*neinn*We +
        0.25*po*wing.layout.span*(1.0+gammas)*(etas-etao) -
        Nload*wing.inboard.weight
    Mo = wing.outboard.moment + wing.outboard.max_shear_load*0.5*wing.layout.span*(etas-etao) +
        (1.0/24.0)*po*wing.layout.span^2*(1.0+2.0*gammas)*(etas-etao)^2 -
        Nload*wing.inboard.dyW - Nload*neinn*We*dyeinn

    #----- limit So,Mo to Ss,Ms, which might be needed with heavy outboard engine
    #-      (rules out negatively-tapered structure, deemed not feasible for downloads)
    wing.inboard.max_shear_load = max(So ,wing.outboard.max_shear_load)
    wing.inboard.moment = max(Mo ,wing.outboard.moment)
    return nothing
end # wing_section_loads

"""
    wing_case_loads(::Cantilever, wing, po, gammat, gammas, Nload, We, neout, dyeout, neinn, dyeinn)

Returns the root and break shear and bending moment of a wing without a strut for one load case, as a
[`WingCaseLoads`](@ref). The outboard loads follow from [`outboard_section_loads!`](@ref) and the inboard
loads from [`wing_section_loads`](@ref).

This function sizes nothing, but it mutates the section load fields of the `wing` passed in
(`wing.outboard.max_shear_load`, `wing.outboard.moment`, `wing.inboard.max_shear_load` and
`wing.inboard.moment`); pass a copy to keep the wing loads unchanged.

!!! details "🔃 Inputs and Outputs"
    **Inputs:**
    - `wing::TASOPT.structures.Wing`: Wing structure, with the panel weights of the load case.
    - `po::Float64`: Wing root loading magnitude.
    - `gammat::Float64`: Wing outer section taper ratio.
    - `gammas::Float64`: Wing inner section taper ratio.
    - `Nload::Int`: Max vertical load factor for wing bending loads.
    - `We::Float64`: Weight of one engine [N].
    - `neout::Int`: Number of engines per wing mounted outboard of the wing break.
    - `dyeout::Float64`: Average spanwise moment arm of the outboard engines about the wing break station ηs [m].
    - `neinn::Int`: Number of engines per wing mounted at or inboard of the wing break.
    - `dyeinn::Float64`: Average spanwise moment arm of the inboard engines about the wing root station ηo [m].

    **Outputs:**
    - `WingCaseLoads`: Shear and moment at the wing root and at the wing break.
"""
function wing_case_loads(::Cantilever, wing, po, gammat, gammas, Nload, We, neout, dyeout, neinn, dyeinn)
    outboard_section_loads!(wing, po, gammat, gammas, Nload, We, neout, dyeout)
    wing_section_loads(Cantilever(), wing, po, gammas, Nload, We, neinn, dyeinn)
    return WingCaseLoads(S_root = wing.inboard.max_shear_load, M_root = wing.inboard.moment,
                         S_break = wing.outboard.max_shear_load, M_break = wing.outboard.moment)
end # wing_case_loads

"""
    wing_case_root_loading(case, wing, rclt, rcls, W_lift, Lhtail, n_pullup)

Returns the load factor and the wing root loading magnitude of one wing sizing load case.

A `Maneuver` case uses the load factor of the case and a tail load of
`case.tail_load_scale * (load factor / n_pullup) * Lhtail`, and its root loading follows from
[`wing_loading`](@ref). A `Ground` case has the load factor of the case and no wing lift. `Gust` cases
are not supported yet and throw an `ArgumentError`.

!!! details "🔃 Inputs and Outputs"
    **Inputs:**
    - `case::WingLoadCase`: Wing sizing load case.
    - `wing::TASOPT.structures.Wing`: Wing structure.
    - `rclt::Float64`: Tip/root cl ratio (clt/clo).
    - `rcls::Float64`: Break/root cl ratio (cls/clo).
    - `W_lift::Float64`: Aircraft weight carried by the wing lift in the case [N].
    - `Lhtail::Float64`: Worst-case (most negative) tail lift at the pull-up load factor [N].
    - `n_pullup::Float64`: Pull-up load factor that `Lhtail` belongs to.

    **Outputs:**
    - `(Nload, po)`: Load factor for wing bending loads and wing root loading magnitude.
"""
function wing_case_root_loading(case, wing, rclt, rcls, W_lift, Lhtail, n_pullup)
    if case.kind == WingLoadKind.Maneuver
        n = case.load_factor
        Lh = case.tail_load_scale * (n / n_pullup) * Lhtail
        return (Nload = n, po = wing_loading(wing, rclt, rcls, n, W_lift, Lh))
    elseif case.kind == WingLoadKind.Ground
        return (Nload = case.load_factor, po = 0.0)
    else
        throw(ArgumentError("gust sizing cases are not supported yet"))
    end
end # wing_case_root_loading

"""
    wing_lift_curve_slope(AR, sweep, Mach)

Returns the lift curve slope of a finite wing from the Helmbold (DATCOM) formula
```math
C_{L_\\alpha} = \\frac{2\\pi AR}{2 + \\sqrt{4 + (AR\\,\\beta)^2 \\left(1 + \\tan^2\\Lambda / \\beta^2\\right)}}, \\qquad \\beta = \\sqrt{1 - M^2}
```
The formula is valid for subsonic Mach numbers. DATCOM uses the mid-chord sweep ``\\Lambda``; the
quarter-chord sweep of the wing layout is used as an approximation of it.

!!! details "🔃 Inputs and Outputs"
    **Inputs:**
    - `AR::Float64`: Wing aspect ratio.
    - `sweep::Float64`: Wing sweep angle [deg].
    - `Mach::Float64`: Flight Mach number.

    **Outputs:**
    - `CL_alpha::Float64`: Wing lift curve slope [1/rad].
"""
function wing_lift_curve_slope(AR, sweep, Mach) # standard range of the formulation is up to Mach <= 0.80
    β = sqrt(1 - Mach^2)
    return 2π * AR / (2 + sqrt(4 + (AR * β)^2 * (1 + tand(sweep)^2 / β^2))) # Polhamus formulation
end # wing_lift_curve_slope

"""
    gust_load_increment(W_over_S, c_mean, CL_alpha, U_de, V_EAS)

Returns the incremental load factor of an aircraft in a vertical gust from the Pratt–Walker gust alleviation
formula (https://ntrs.nasa.gov/api/citations/19930090988/downloads/19930090988.pdf)
```math
\\mu = \\frac{2\\,(W/S)}{\\rho_0\\, \\bar{c}\\, C_{L_\\alpha}\\, g}, \\qquad K_g = \\frac{0.88\\,\\mu}{5.3 + \\mu}, \\qquad
\\Delta n = \\frac{\\rho_0\\, U_{de}\\, V_{EAS}\\, C_{L_\\alpha}\\, K_g}{2\\,(W/S)}
```
where ``\\rho_0`` is the sea-level air density. The gust velocity and the airspeed are equivalent airspeeds (EAS).
The wing sizing cases do not use this function yet.

!!! details "🔃 Inputs and Outputs"
    **Inputs:**
    - `W_over_S::Float64`: Wing loading [N/m²].
    - `c_mean::Float64`: Mean geometric chord, wing area over span [m].
    - `CL_alpha::Float64`: Wing lift curve slope [1/rad].
    - `U_de::Float64`: Derived gust velocity [m/s EAS].
    - `V_EAS::Float64`: Equivalent airspeed [m/s].

    **Outputs:**
    - `Δn::Float64`: Magnitude of the incremental load factor.
"""
function gust_load_increment(W_over_S, c_mean, CL_alpha, U_de, V_EAS)
    ρ0 = atmos(0.0).ρ
    μ = 2 * W_over_S / (ρ0 * c_mean * CL_alpha * gee)
    K_g = 0.88 * μ / (5.3 + μ)
    return ρ0 * U_de * V_EAS * CL_alpha * K_g / (2 * W_over_S)
end # gust_load_increment

"""
    wing_sizing_masses(; W_cruise, W_MTO, W_pay, W_paymax, W_fuel, freserve, W_fuel_cap, has_wing_fuel)

Returns the weights that [`evaluate_wing_sizing_cases!`](@ref) passes to [`wing_mass_state`](@ref), from the
aircraft weights of a sizing iteration. The reserve, empty and fuel-at-MTO weights are simple estimates:
- `W_reserve = W_fuel * freserve / (1 + freserve)`: `freserve` is the reserve fuel over the burned fuel, and
  `W_fuel` includes the reserve.
- `W_empty = W_MTO - W_pay - W_fuel`: the operating empty weight, without payload and fuel.
- `W_fuel_at_MTO = W_pay - W_paymax + W_fuel`: the fuel that fits at the maximum takeoff weight with the
  maximum payload, capped at `W_fuel_cap` if the fuel is stored in the wing.
- `r_fuel_design`: the same fuel weight over `W_fuel_cap` (zero if the fuel is not stored in the wing). It is
  not capped at one.

!!! details "🔃 Inputs and Outputs"
    **Inputs:**
    - `W_cruise::Float64`: Aircraft weight at the design cruise point [N].
    - `W_MTO::Float64`: Maximum takeoff weight [N].
    - `W_pay::Float64`: Design payload weight [N].
    - `W_paymax::Float64`: Maximum payload weight [N].
    - `W_fuel::Float64`: Design mission fuel weight, including the reserve [N].
    - `freserve::Float64`: Reserve fuel as a fraction of the burned fuel.
    - `W_fuel_cap::Float64`: Fuel weight that fills the wing tanks [N].
    - `has_wing_fuel::Bool`: Whether fuel is stored in the wing.

    **Outputs:**
    - `masses::NamedTuple`: `W_cruise`, `W_MTO`, `W_fuel`, `W_reserve`, `W_empty`, `W_fuel_cap`, `W_fuel_at_MTO`
      and `r_fuel_design`, as defined above [N, except `r_fuel_design`].
"""
function wing_sizing_masses(; W_cruise, W_MTO, W_pay, W_paymax, W_fuel, freserve, W_fuel_cap, has_wing_fuel)
    W_fuel_design_MTO = W_pay - W_paymax + W_fuel
    return (W_cruise = W_cruise,
            W_MTO = W_MTO,
            W_fuel = W_fuel,
            W_reserve = W_fuel * freserve / (1 + freserve),
            W_empty = W_MTO - W_pay - W_fuel,
            W_fuel_cap = W_fuel_cap,
            W_fuel_at_MTO = has_wing_fuel ? min(W_fuel_cap, W_fuel_design_MTO) : W_fuel_design_MTO,
            r_fuel_design = (has_wing_fuel && W_fuel_cap > 0) ? W_fuel_design_MTO / W_fuel_cap : 0.0)
end # wing_sizing_masses

"""
    evaluate_wing_sizing_cases!(wing, masses, has_wing_fuel, rclt, rcls, Lhtail, gammat, gammas, We, dyeinn)

Computes the root and break loads of every active wing sizing load case of a wing without a strut and
returns their envelope. The loads of each case are stored in `wing.sizing_loads.results`, the load factor
of the case in its `n_computed`, and the cases with the largest root and break bending moment in
`wing.sizing_loads.governing_root` and `wing.sizing_loads.governing_break`.

Active gust cases are skipped. The wing has one engine per wing at the inboard side. The section loads
of the `wing` are not modified.

!!! details "🔃 Inputs and Outputs"
    **Inputs:**
    - `wing::TASOPT.structures.Wing`: Wing structure, with the panel weights of the design cruise mass state.
    - `masses::NamedTuple`: Weights for [`wing_mass_state`](@ref): `W_cruise`, `W_MTO`, `W_fuel`, `W_reserve`,
      `W_empty`, `W_fuel_cap`, `W_fuel_at_MTO` and `r_fuel_design`.
    - `has_wing_fuel::Bool`: Whether fuel is stored in the wing.
    - `rclt::Float64`: Tip/root cl ratio (clt/clo).
    - `rcls::Float64`: Break/root cl ratio (cls/clo).
    - `Lhtail::Float64`: Worst-case (most negative) tail lift at the pull-up load factor [N].
    - `gammat::Float64`: Wing outer section taper ratio.
    - `gammas::Float64`: Wing inner section taper ratio.
    - `We::Float64`: Weight of one engine [N].
    - `dyeinn::Float64`: Average spanwise moment arm of the inboard engine about the wing root station ηo [m].

    **Outputs:**
    - `(S_root, M_root, S_break, M_break)`: Largest magnitude of the shear and bending moment at the wing root
      and at the wing break, each from its governing case, or `nothing` if no case was evaluated.
"""
function evaluate_wing_sizing_cases!(wing, masses, has_wing_fuel, rclt, rcls, Lhtail, gammat, gammas, We, dyeinn)
    sl = wing.sizing_loads
    n_pullup = sl.pullup.load_factor
    empty!(sl.results)

    # Limitation: every case uses the panel weights and inertia stored in the wing (design cruise mass
    # state). Only the lift weight, the load factor and the tail load follow the mass state of the case.
    # Per-case panel inertia, from the structural and fuel weights of each mass state, is not implemented yet.
    for name in fieldnames(WingSizingLoads)
        case = getfield(sl, name)
        case isa WingLoadCase || continue
        (case.active && case.kind != WingLoadKind.Gust) || continue

        ms = wing_mass_state(case.mass_state, has_wing_fuel; masses...)
        (Nload, po) = wing_case_root_loading(case, wing, rclt, rcls, ms.W_lift, Lhtail, n_pullup)
        case.n_computed = Nload
        sl.results[name] = wing_case_loads(Cantilever(), deepcopy(wing), po, gammat, gammas, Nload, We, 0, 0.0, 1, dyeinn)
    end
    isempty(sl.results) && return nothing

    # first case in declaration order wins ties
    names = [name for name in fieldnames(WingSizingLoads) if haskey(sl.results, name)]
    sl.governing_root = names[argmax([abs(sl.results[name].M_root) for name in names])]
    sl.governing_break = names[argmax([abs(sl.results[name].M_break) for name in names])]

    root, brk = sl.results[sl.governing_root], sl.results[sl.governing_break]
    return (S_root = abs(root.S_root), M_root = abs(root.M_root),
            S_break = abs(brk.S_break), M_break = abs(brk.M_break))
end # evaluate_wing_sizing_cases!

"""
    wing_mass_state(mass_state, has_wing_fuel; W_cruise, W_MTO, W_fuel, W_reserve, W_empty,
        W_fuel_cap, W_fuel_at_MTO, r_fuel_design)

Returns the lift weight and the fuel split of a wing mass state: the weight the wing lifts, the fuel
carried in the wing as a fraction of the wing tank capacity, and the fuel carried in the fuselage.

!!! details "🔃 Inputs and Outputs"
    **Inputs:**
    - `mass_state::WingMassState.T`: Mass state (`DesignCruise`, `MTOW_MaxFuel`, `MZFW` or `Light`).
    - `has_wing_fuel::Bool`: Whether fuel is stored in the wing. If `false`, the wing fuel fraction is zero
      and `W_fuel_cap` is not used; if `true`, all fuel is in the wing and the fuselage fuel weight is zero.
    - `W_cruise::Float64`: Aircraft weight at the design cruise point [N].
    - `W_MTO::Float64`: Maximum takeoff weight [N].
    - `W_fuel::Float64`: Design mission fuel weight [N].
    - `W_reserve::Float64`: Reserve fuel weight [N].
    - `W_empty::Float64`: Operating empty weight, without fuel [N].
    - `W_fuel_cap::Float64`: Fuel weight that fills the wing tanks [N].
    - `W_fuel_at_MTO::Float64`: Fuel weight on board at maximum takeoff weight with the maximum payload [N].
    - `r_fuel_design::Float64`: Wing fuel fraction (fuel weight over `W_fuel_cap`) at the design cruise point.

    **Outputs:**
    - `W_lift::Float64`: Weight carried by the wing lift [N].
    - `r_fuel_wing::Float64`: Fuel weight in the wing as a fraction of `W_fuel_cap`; at most one at `MTOW_MaxFuel`.
    - `W_fuel_fuse::Float64`: Fuel weight in the fuselage [N].
"""
function wing_mass_state(mass_state, has_wing_fuel;
        W_cruise, W_MTO, W_fuel, W_reserve, W_empty, W_fuel_cap, W_fuel_at_MTO, r_fuel_design)

    if mass_state == WingMassState.DesignCruise
        W_lift, W_fuel_state = W_cruise, W_fuel
        r_fuel_wing = has_wing_fuel ? r_fuel_design : 0.0
    elseif mass_state == WingMassState.MTOW_MaxFuel
        W_lift, W_fuel_state = W_MTO, W_fuel_at_MTO
        r_fuel_wing = has_wing_fuel ? min(1.0, W_fuel_at_MTO / W_fuel_cap) : 0.0
    elseif mass_state == WingMassState.MZFW
        W_lift, W_fuel_state = W_MTO - W_fuel + W_reserve, W_reserve
        r_fuel_wing = has_wing_fuel ? W_reserve / W_fuel_cap : 0.0
    else # WingMassState.Light
        W_lift, W_fuel_state = W_empty + W_reserve, W_reserve
        r_fuel_wing = has_wing_fuel ? W_reserve / W_fuel_cap : 0.0
    end

    W_fuel_fuse = has_wing_fuel ? 0.0 : W_fuel_state
    return (W_lift = W_lift, r_fuel_wing = r_fuel_wing, W_fuel_fuse = W_fuel_fuse)
end # wing_mass_state

"""
    wing_weights!(wing, po, gammat, gammas, 
       Nload, We, neout, dyeout, neinn, dyeinn, sigfac, rhofuel; n_wings=2.0, has_strut=false,
       design_wing_fuel=nothing, has_wing_fuel=false, has_centerbox_fuel=false, section_loads_envelope=nothing)

Calculates Wing or Tail loads, stresses, weights of individual wing sections.
Also returns the material gauges, torsional and bending stiffness. Formerly, `get_wing_weights!()`.

!!! details "🔃 Inputs and Outputs"
    **Inputs:**
    - `wing::TASOPT.structures.Wing`: Wing structure.
    - `po::Float64`: Wing root loading magnitude.
    - `gammat::Float64`: Wing outer section taper ratio.
    - `gammas::Float64`: Wing inner section taper ratio.
    - `Nload::Int`: Max vertical load factor for wing bending loads.
    - `We::Float64`: Weight of one engine [N].
    - `neout::Int`:  Number of engines per wing mounted outboard of the wing break (strut attachment point).
    - `dyeout::Float64`: Average spanwise moment arm of the outboard engines about the wing break station ηs, i.e. ``y_e - y_s`` [m].
    - `neinn::Int`: Number of engines per wing mounted at or inboard of the wing break (strut attachment point).
    - `dyeinn::Float64`: Average spanwise moment arm of the inboard engines about the wing root station ηo, i.e. ``y_e - y_o`` [m]. For an engine at the break, ``(b/2)(η_s - η_o)``.
    - `sigfac::Float64`: Stress Factor.
    - `rhofuel::Float64`: Density of the fuel.
    - `n_wings::Int64`: Number of total wings (1 for Vtail).
    - `has_strut::Bool`: Whether the wing is braced by a strut (`false` for a cantilever wing).
    - `design_wing_fuel::Union{Nothing,Float64}`: Fuel weight carried at the design point [N]. If given (not `nothing`), the panel
        weights `wing.inboard.weight`, `wing.outboard.weight` and moments `wing.inboard.dyW`, `wing.outboard.dyW` are updated with the
        panel structure weight (including added weight) plus the wing fuel weight, scaled by the fuel fraction `design_wing_fuel / Wfmax`.
        If `nothing` (default), these panel fields are not modified.
    - `has_wing_fuel::Bool`: Whether fuel is stored in the wing (the fuel fraction is zero otherwise). Only used with `design_wing_fuel`.
    - `has_centerbox_fuel::Bool`: Whether fuel is stored in the wing center box. Only used with `design_wing_fuel`.
    - `section_loads_envelope::Union{Nothing,NamedTuple}`: Envelope of the section loads `(S_root, M_root, S_break, M_break)` over the wing sizing cases.
        If given (not `nothing`), it replaces the computed shear and moment of the inboard (root) and outboard (break) sections before they are sized.
        It applies to cantilever wings only and must not be used with `has_strut`. If `nothing` (default), the computed loads are used.

See [Geometry](@ref geometry),  [Wing/Tail Structures](@ref wingtail), and Section 2.7  of the [TASOPT Technical Description](@ref dreladocs). 
"""
function wing_weights!(wing, po, gammat, gammas,
       Nload, We, neout, dyeout, neinn, dyeinn, sigfac, rhofuel; n_wings=2.0, has_strut=false,
       design_wing_fuel=nothing, has_wing_fuel=false, has_centerbox_fuel=false, section_loads_envelope=nothing)

    tauweb = wing.inboard.webs.material.τmax * sigfac

    cosL = cosd(wing.layout.sweep)
    sinL = sind(wing.layout.sweep)

    # Calculate non-dim span coordinate at span break and root (ηs and ηo resp.)
    etao = wing.layout.ηo
    etas = wing.layout.ηs

    h_avgo, h_rmso = get_average_sparbox_heights(wing.inboard.cross_section)
    h_avgs, h_rmss = get_average_sparbox_heights(wing.outboard.cross_section)

    outboard_section_loads!(wing, po, gammat, gammas, Nload, We, neout, dyeout)
    if !has_strut && !isnothing(section_loads_envelope)
        wing.outboard.max_shear_load = section_loads_envelope.S_break
        wing.outboard.moment = section_loads_envelope.M_break
    end

    #---- size strut-attach station at etas
    cs = wing.layout.root_chord*wing.inboard.λ

    outboard_dims = size_wing_section!(wing.outboard, wing.sweep, sigfac)
    # Inboard Section:
    if !has_strut
        #----- no strut, with or without engine at etas
        ls = 0.
        Tstrut = 0.
        Rstrut = 0.
        Pstrut = 0.

        wing_section_loads(Cantilever(), wing, po, gammas, Nload, We, neinn, dyeinn)
        if !isnothing(section_loads_envelope)
            wing.inboard.max_shear_load = section_loads_envelope.S_root
            wing.inboard.moment = section_loads_envelope.M_root
        end

        inboard_dims = size_wing_section!(wing.inboard, wing.sweep, sigfac)

        lsp = 0.
        Tstrutp = 0.
        if wing isa Wing # Tail has no strut
            wing.strut.cos_lambda = 1.0
        end

    else
        #----- strut present
        ls = sqrt(wing.strut.z^2 + (0.5*wing.layout.span*(etas-etao))^2)
        Rstrut = (po*wing.layout.span/12.0)*(etas-etao)*(1.0+2.0*gammas) + wing.outboard.max_shear_load
        Tstrut = Rstrut*ls/wing.strut.z
        #c     Pstrut = Rstrut*0.5*wing.layout.span*(etas-etao)/zs

        #----- inboard shear,moment used for sparbox sizing
        wing.inboard.max_shear_load = wing.outboard.max_shear_load
        wing.inboard.moment = wing.outboard.moment
        #
        #----- size inboard station at etao
        inboard_dims = size_wing_section!(wing.inboard, wing.sweep, sigfac)

        #----- total strut length, tension
        lsp = sqrt(wing.strut.z^2 + (0.5*wing.layout.span*(etas-etao)/cosL)^2)
        Tstrutp = Tstrut*lsp/ls
        wing.strut.cos_lambda = ls/lsp
        wing.strut.axial_force = Tstrutp
        wing.strut.area = Tstrutp/(wing.strut.material.σmax * sigfac)
        wing.strut.length = lsp
        wing.strut.weight   = 2.0*wing.strut.material.ρ*gee*wing.strut.area*lsp
        wing.strut.dxW = wing.strut.weight * 0.25*wing.layout.span*(etas-etao) * sinL/cosL
    end

    Abfuels = calc_sparbox_internal_area(wing.inboard.cross_section.width_to_chord,
                                    h_avgs, outboard_dims.tbcap, outboard_dims.tbweb)
    Abfuelo = calc_sparbox_internal_area(wing.inboard.cross_section.width_to_chord,
                                    h_avgo, inboard_dims.tbcap, inboard_dims.tbweb)
    
    

    Vcen = wing.layout.root_chord^2*wing.layout.span*  etao / 2.0

    Vinn = wing.layout.root_chord^2*wing.layout.span* (etas-etao) *
        (1.0 + wing.inboard.λ + wing.inboard.λ^2)/6.0 *
        cosL
    Vout = wing.layout.root_chord^2*wing.layout.span* (1.0 -etas) *
        (wing.inboard.λ^2 + wing.inboard.λ*wing.outboard.λ + wing.outboard.λ^2)/6.0 *
        cosL

    dxVinn = wing.layout.root_chord^2*wing.layout.span^2 * (etas-etao)^2 *
        (1.0 + 2.0*wing.inboard.λ + 3.0*wing.inboard.λ^2)/48.0 *
        sinL
    dxVout = wing.layout.root_chord^2*wing.layout.span^2 * (1.0 -etas)^2 *
        (wing.inboard.λ^2 + 2.0*wing.inboard.λ*wing.outboard.λ + 3.0*wing.outboard.λ^2)/48.0 *
        sinL +
        wing.layout.root_chord^2*wing.layout.span^2 * (etas-etao)*(1.0 -etas) *
        (wing.inboard.λ^2 + wing.inboard.λ*wing.outboard.λ + wing.outboard.λ^2)/12.0 *
        sinL

    dyVinn = wing.layout.root_chord^2*wing.layout.span^2 * (etas-etao)^2 *
        (1.0 + 2.0*wing.inboard.λ + 3.0*wing.inboard.λ^2)/48.0 *
        cosL
    dyVout = wing.layout.root_chord^2*wing.layout.span^2 * (1.0 -etas)^2 *
        (wing.inboard.λ^2 + 2.0*wing.inboard.λ*wing.outboard.λ + 3.0*wing.outboard.λ^2)/48.0 *
        cosL

    #---- set chord^2 weighted average areas for inner panel
    Abcapi = (inboard_dims.Abcap + outboard_dims.Abcap*wing.inboard.λ^2)/(1.0+wing.inboard.λ^2)
    Abwebi = (inboard_dims.Abweb + outboard_dims.Abweb*wing.inboard.λ^2)/(1.0+wing.inboard.λ^2)

    Wscen   = (wing.inboard.caps.ρ*inboard_dims.Abcap  + wing.inboard.webs.ρ*inboard_dims.Abweb )*gee*Vcen
    Wsinn   = (wing.inboard.caps.ρ*Abcapi              + wing.inboard.webs.ρ*Abwebi             )*gee*Vinn
    Wsout   = (wing.inboard.caps.ρ*outboard_dims.Abcap + wing.inboard.webs.ρ*outboard_dims.Abweb)*gee*Vout

    dxWsinn = (wing.inboard.caps.ρ*Abcapi              + wing.inboard.webs.ρ*Abwebi             )*gee*dxVinn
    dxWsout = (wing.inboard.caps.ρ*outboard_dims.Abcap + wing.inboard.webs.ρ*outboard_dims.Abweb)*gee*dxVout

    dyWsinn = (wing.inboard.caps.ρ*Abcapi              + wing.inboard.webs.ρ*Abwebi             )*gee*dyVinn
    dyWsout = (wing.inboard.caps.ρ*outboard_dims.Abcap + wing.inboard.webs.ρ*outboard_dims.Abweb)*gee*dyVout


    Abfueli= (Abfuelo + Abfuels*wing.inboard.λ^2)/(1.0+wing.inboard.λ^2)

    Wfcen   = rhofuel*Abfuelo *gee*Vcen
    Wfinn   = rhofuel*Abfueli *gee*Vinn
    Wfout   = rhofuel*Abfuels *gee*Vout

    dxWfinn = rhofuel*Abfueli *gee*dxVinn
    dxWfout = rhofuel*Abfuels *gee*dxVout

    dyWfinn = rhofuel*Abfueli *gee*dyVinn
    dyWfout = rhofuel*Abfuels *gee*dyVout

    wing.inboard.caps.weight.W   = 2.0*wing.inboard.caps.ρ*gee*(inboard_dims.Abcap*Vcen + Abcapi*Vinn + outboard_dims.Abcap*Vout)
    wing.inboard.webs.weight.W   = 2.0*wing.inboard.webs.ρ*gee*(inboard_dims.Abweb*Vcen + Abwebi*Vinn + outboard_dims.Abweb*Vout)

    dxWcap = 2.0*wing.inboard.caps.ρ*gee*( Abcapi*dxVinn + outboard_dims.Abcap*dxVout )
    dxWweb = 2.0*wing.inboard.webs.ρ*gee*( Abwebi*dxVinn + outboard_dims.Abweb*dxVout )

    Vout = wing.layout.root_chord^2*wing.layout.span* (1.0 -etas) *
    (wing.inboard.λ^2 + wing.inboard.λ*wing.outboard.λ + wing.outboard.λ^2)/6.0 *
    cosL

    wing.inboard.caps.thickness = inboard_dims.tbcap
    wing.inboard.webs.thickness = inboard_dims.tbweb
    wing.outboard.caps.thickness = outboard_dims.tbcap
    wing.outboard.webs.thickness = outboard_dims.tbweb

    fwadd = wing_additional_weight(wing)
    Wwing = n_wings * (Wscen + Wsinn + Wsout) * (1.0 + fwadd)
    wing.dxW = n_wings * (dxWsinn + dxWsout) * (1.0 + fwadd)

    if !isnothing(design_wing_fuel)
        Wfmax = has_wing_fuel ? 2.0 * ((has_centerbox_fuel ? Wfcen : 0.0) + Wfinn + Wfout) : 0.0
        rfmax = has_wing_fuel ? design_wing_fuel / Wfmax : 0.0

        wing.inboard.weight = Wsinn * (1.0 + fwadd) + rfmax * Wfinn
        wing.outboard.weight = Wsout * (1.0 + fwadd) + rfmax * Wfout
        wing.inboard.dyW = dyWsinn * (1.0 + fwadd) + rfmax * dyWfinn
        wing.outboard.dyW = dyWsout * (1.0 + fwadd) + rfmax * dyWfout
    end

    return Wwing, Wsinn, Wsout, dyWsinn, dyWsout, Wfcen, Wfinn, Wfout,
    dxWfinn, dxWfout, dyWfinn, dyWfout, lsp

end # wing_weights


"""
    size_cap(σmax, moment, h̄, w̄, h_rms, c, cosΛ; t_min = 0.0)

Calculates area and thickness of wing caps based on maximum stress and geometric parameters.

!!! details "🔃 Inputs and Outputs"
    **Inputs:**
    - `σmax::Float64`: Maximum allowable stress
    - `moment::Float64`: Bending moment
    - `h̄::Float64`: Height-to-chord ratio
    - `w̄::Float64`: Width-to-chord ratio
    - `h_rms::Float64`: RMS height of sparbox
    - `c::Float64`: Chord length
    - `cosΛ::Float64`: Cosine of sweep angle
    - `t_min::Float64`: (keyword, default 0.0) Minimum cap thickness; `t_cap = max(required, t_min)`
    
    **Outputs:**
    - `t_cap::Float64`: Thickness of cap
    - `Ab_cap::Float64`: Cross-sectional area of cap
"""
function size_cap(σmax, moment, h̄, w̄, h_rms, c, cosΛ; t_min = 0.0)
    t_cap = max(calc_cap_thickness(σmax, moment, h̄, w̄, h_rms, c, cosΛ), t_min)
    Ab_cap = 2 * t_cap * w̄
    return t_cap, Ab_cap
end  # function size_cap

"""
    size_web(τmax, shear, c_perp, web_height; t_min = 0.0)

Calculates area and thickness of wing webs based on maximum shear stress and geometric parameters.

!!! details "🔃 Inputs and Outputs"
    **Inputs:**
    - `τmax::Float64`: Maximum allowable shear stress
    - `shear::Float64`: Shear load
    - `c_perp::Float64`: Perpendicular chord length
    - `web_height::Float64`: Height of web
    - `t_min::Float64`: (keyword, default 0.0) Minimum web thickness; `t_web = max(required, t_min)`
    
    **Outputs:**
    - `t_web::Float64`: Thickness of web
    - `Ab_web::Float64`: Cross-sectional area of web
"""
function size_web(τmax, shear, c_perp, web_height; t_min = 0.0)
    t_web = max(calc_web_thickness(τmax, shear, c_perp, web_height), t_min)
    Ab_web = 2 * t_web * web_height
    return t_web, Ab_web
end  # function size_web

"""
    calc_web_thickness(τmax, shear, c_perp, web_height)

Calculates the required web thickness based on maximum shear stress and loading conditions.

!!! details "🔃 Inputs and Outputs"
    **Inputs:**
    - `τmax::Float64`: Maximum allowable shear stress
    - `shear::Float64`: Shear load
    - `c_perp::Float64`: Perpendicular chord length
    - `web_height::Float64`: Height of web
    
    **Outputs:**
    - `t_web::Float64`: Required thickness of web
"""
function calc_web_thickness(τmax, shear, c_perp, web_height)
    t_web = shear / (c_perp^2 * 2 * web_height * τmax)
    return t_web
end  # function calc_web_thickness

"""
    calc_cap_thickness(σmax, moment, h̄, w̄, h_rms, c, cosΛ)

Calculates the required cap thickness based on maximum stress and geometric parameters.

!!! details "🔃 Inputs and Outputs"
    **Inputs:**
    - `σmax::Float64`: Maximum allowable stress
    - `moment::Float64`: Bending moment
    - `h̄::Float64`: Height-to-chord ratio
    - `w̄::Float64`: Width-to-chord ratio
    - `h_rms::Float64`: RMS height of sparbox
    - `c::Float64`: Chord length
    - `cosΛ::Float64`: Cosine of sweep angle
    
    **Outputs:**
    - `t_cap::Float64`: Required thickness of cap

Throws an `ErrorException` if the cap sizing is infeasible (`con >= h_rms^3`).
"""
function calc_cap_thickness(σmax, moment, h̄, w̄, h_rms, c, cosΛ)
    con = moment * 6h̄ / w̄ * 1 / (c^3 * σmax * cosΛ^4)
    if con >= h_rms^3
        error("Wing cap sizing infeasible: bending load parameter con = $con >= h_rms^3 = $(h_rms^3). " *
              "The cap cannot carry the moment within the sparbox height.")
    end
    t_cap = 0.5 * (h_rms - ∛(h_rms^3 - con))
    return t_cap
end  # function calc_cap_thickness

"""
    calc_sparbox_internal_area(width, height, t_cap, t_web)

Calculates the internal aera of the sparbox, accounting for the thickness of
the webs and caps.
A = (w - 2tweb)×(h - 2tcap)
"""
function calc_sparbox_internal_area(width, height, t_cap, t_web)
    return (width - 2*t_web)*(height - 2*t_cap)
end  # function calc_internal_area