# Structural loads on a folding wing tip: the loads acting at the fold station of the outer wing panel.

"""
    fold_station_loads(wing::Wing, po, γt, γs, Nload, K_w) -> NamedTuple

Calculates the shear and bending moment at the fold station `wing.fold.η_fold`, per wing side, from the lift and the
structural weight of the wing outboard of the fold (the folding wing tip). The lift is the linear outer-panel
loading used in [`wing_weights!`](@ref), including the tip roll-off point load at the tip, and the weight is the
wing structure only, integrated with the same outer-panel expressions as in `wing_weights!`. The weight does not
include fuel, engines or the fold mechanism. The fold station geometry is given by [`fold_station_geometry`](@ref).

The tip weight is `K_w c_o² cos(Λ) ∫ λ² dy` over the tip, where `λ = c / c_o` is the local chord ratio.

!!! details "🔃 Inputs and Outputs"
    **Inputs:**
    - `wing::TASOPT.structures.Wing`: Wing structure.
    - `po::Float64`: Wing root loading, i.e. the lift per unit span at the root [N/m].
    - `γt::Float64`: Wing tip load ratio, relative to the root loading `po`.
    - `γs::Float64`: Wing break (`ηs`) load ratio, relative to the root loading `po`.
    - `Nload::Float64`: Load factor.
    - `K_w::Float64`: Structural weight per unit chord-squared volume, including the added-weight fraction [N/m³].

    **Outputs:** `NamedTuple` with fields
    - `S_a::Float64`: Shear at the fold station from the lift alone [N].
    - `M_a::Float64`: Bending moment at the fold station from the lift alone [N⋅m].
    - `W_tip::Float64`: Structural weight outboard of the fold station [N].
    - `dyW_tip::Float64`: Moment of the structural weight outboard of the fold station about the fold station [N⋅m].
    - `S_f::Float64`: Shear at the fold station, `S_a - Nload W_tip` [N].
    - `M_f::Float64`: Bending moment at the fold station, `M_a - Nload dyW_tip` [N⋅m].
"""
function fold_station_loads(wing::Wing, po, γt, γs, Nload, K_w)
    layout = wing.layout
    b = layout.span
    co = layout.root_chord
    ηs = layout.ηs
    λt = wing.outboard.λ
    cosL = cosd(layout.sweep)

    η_f = wing.fold.η_fold
    λ_f, ℓ = fold_station_geometry(wing, η_f)

    γ_f = γs + (γt - γs) * (η_f - ηs) / (1 - ηs)

    # Tip roll-off lift, modeled as a point load at the tip
    ΔL_t = wing.tip_lift_loss * po * co * γt * λt

    S_a = (po * b / 4) * (γ_f + γt) * (1 - η_f) + ΔL_t
    M_a = (po * b^2 / 24) * (γ_f + 2 * γt) * (1 - η_f)^2 + ΔL_t * ℓ

    W_tip = K_w * co^2 * (b / 2) * (1 - η_f) * (λ_f^2 + λ_f * λt + λt^2) / 3 * cosL
    dyW_tip = K_w * co^2 * (b^2 / 48) * (1 - η_f)^2 * (λ_f^2 + 2 * λ_f * λt + 3 * λt^2) * cosL

    S_f = S_a - Nload * W_tip
    M_f = M_a - Nload * dyW_tip

    return (; S_a, M_a, W_tip, dyW_tip, S_f, M_f)
end

"""
    size_pinned_joint(F, n, fold::FoldingWingTip, sigfac) -> (W, details)

Sizes `n` identical pins and their lugs that together carry the force `F`, and returns the weight of the joint
per wing side. The pins are in double shear and the lug thickness is set by the bearing stress, with the allowable
stresses scaled by `sigfac` as in the wing box. The lug is a semicircular end of radius `R` with a base of length `L`
and a pin hole of diameter `d`. This is the single function that [`size_latches`](@ref) and the hinge sizing in
[`size_fold_mechanism!`](@ref) use, so an alternative joint sizing method replaces only this function.

The pin diameter is `d = √(2F / (n π τ_allow))` and the lug thickness is `t = F / (n d σ_br_allow)`. The lug radius and
length are `R = lug_R_over_d d` and `L = lug_L_over_d d`. A zero force gives a joint of zero weight and zero dimensions.

!!! details "🔃 Inputs and Outputs"
    **Inputs:**
    - `F::Float64`: Total force carried by the joint [N].
    - `n::Int`: Number of pins sharing the force.
    - `fold::TASOPT.structures.FoldingWingTip`: Folding wing tip, providing the pin and lug materials, bearing strengths and lug proportions.
    - `sigfac::Float64`: Stress factor applied to the allowable stresses (as in the wing box).

    **Outputs:**
    - `W::Float64`: Weight of the pins and lugs per wing side [N].
    - `details::NamedTuple`: Pin diameter `d`, lug thickness `t`, lug radius `R` and lug length `L` [m].
"""
function size_pinned_joint(F, n, fold::FoldingWingTip, sigfac)
    # No force needs no joint; the pin diameter formula below would otherwise divide by zero
    F == 0 && return 0.0, (; d = 0.0, t = 0.0, R = 0.0, L = 0.0)

    ultimate_factor = 1.5 # Ultimate to limit load factor, as in the wing box safety factor
    # PLACEHOLDER: the lug stack is a middle lug of thickness t between two clevis ears of t/2 each,
    # and the pin is as long as the stack
    k_stack = 2.0

    τ_allow = fold.material_pin.τmax * sigfac
    σ_br_allow = min(fold.σ_br_pin, fold.σ_br_lug) / ultimate_factor * sigfac

    d = sqrt(2F / (n * π * τ_allow)) # Pin in double shear
    t = F / (n * d * σ_br_allow)

    R = fold.lug_R_over_d * d
    L = fold.lug_L_over_d * d
    A_lug = π * R^2 / 2 + 2 * R * L - π * d^2 / 4

    V_pin = (π * d^2 / 4) * (k_stack * t)
    V_lug = A_lug * (k_stack * t)
    W = gee * n * (fold.material_pin.ρ * V_pin + fold.material_lug.ρ * V_lug)

    return W, (; d, t, R, L)
end

"""
    size_latches(F_cap, fold::FoldingWingTip, sigfac; latch_actuator_mass = 0.0) -> (W, details)

Sizes the `fold.n_latch` latch pins and lugs, which carry the wing-box cap force, with [`size_pinned_joint`](@ref),
and adds the actuator mass of each latch.

!!! details "🔃 Inputs and Outputs"
    **Inputs:**
    - `F_cap::Float64`: Wing-box cap force couple at the fold [N].
    - `fold::TASOPT.structures.FoldingWingTip`: Folding wing tip.
    - `sigfac::Float64`: Stress factor applied to the allowable stresses (as in the wing box).
    - `latch_actuator_mass::Float64`: PLACEHOLDER: actuator mass per latch [kg].

    **Outputs:**
    - `W::Float64`: Weight of the latch pins, lugs and actuators per wing side [N].
    - `details::NamedTuple`: Sizing details from [`size_pinned_joint`](@ref).
"""
function size_latches(F_cap, fold::FoldingWingTip, sigfac; latch_actuator_mass = 0.0)
    W_joint, details = size_pinned_joint(F_cap, fold.n_latch, fold, sigfac)
    W = W_joint + latch_actuator_mass * gee * fold.n_latch
    return W, details
end

"""
    size_load_introduction(F_cap, h_box, w_box, fold::FoldingWingTip, ρ_rib, sigfac) -> (W, details)

Sizes the structure that introduces the wing-box cap forces into the fold joint, and returns its weight per wing side.
It is made of one fitting per cap (two per wing side) and two full-depth closing ribs, one on each side of the fold.

Each fitting carries the cap force `F_cap` at the allowable stress of the fitting material, scaled by `sigfac` as in
the wing box, over a length of `fold.fitting_length_factor` times the box depth. The fitting weight is then
`2 g ρ_fitting (F_cap / σ_fit) fitting_length_factor h_box`. Each rib has the box cross-section and the
minimum gauge `fold.rib_min_gauge`.

!!! details "🔃 Inputs and Outputs"
    **Inputs:**
    - `F_cap::Float64`: Wing-box cap force couple at the fold [N].
    - `h_box::Float64`: Wing-box depth at the fold [m].
    - `w_box::Float64`: Wing-box width at the fold [m].
    - `fold::TASOPT.structures.FoldingWingTip`: Folding wing tip, providing the fitting material, the fitting length factor and the rib gauge.
    - `ρ_rib::Float64`: Density of the rib material [kg/m³].
    - `sigfac::Float64`: Stress factor applied to the allowable stresses (as in the wing box).

    **Outputs:**
    - `W::Float64`: Weight of the fittings and ribs per wing side [N].
    - `details::NamedTuple`: Weight of the fittings `W_fittings` and of the ribs `W_ribs` [N].
"""
function size_load_introduction(F_cap, h_box, w_box, fold::FoldingWingTip, ρ_rib, sigfac)
    σ_fit = fold.material_fitting.σmax * sigfac

    W_fittings = 2 * gee * fold.material_fitting.ρ * (F_cap / σ_fit) * fold.fitting_length_factor * h_box
    # PLACEHOLDER: the closing ribs are sized at the minimum gauge, not by the loads they carry
    W_ribs = 2 * gee * ρ_rib * w_box * h_box * fold.rib_min_gauge

    return W_fittings + W_ribs, (; W_fittings, W_ribs)
end

"""
    size_fold_actuator(wing::Wing, loads) -> (W, details)

Sizes the actuator that folds the wing tip, and returns its weight per wing side. The actuator moment about the fold line
is the sum of the gravity moment of the folding wing tip and the moment of the wind on the tip panel on the ground. The
actuator mass is the moment times `fold.actuator_torque_density`.

The gravity moment is `W_tip ℓ_cg`, where `ℓ_cg = dyW_tip / W_tip` is the spanwise distance from the fold station to the
center of gravity of the tip structure. The wind is a uniform pressure `q = ½ ρ V²` at the ground wind speed
`fold.ground_wind_speed` and sea-level density, normal to the tip panel. Its moment about the fold line integrates the
linearly tapering chord `c(y) = co (λ_f + (λt - λ_f) y / ℓ)` over the panel span `ℓ`, which gives
`q C_N ∫ c y dy = q C_N co ℓ² (λ_f + 2 λt) / 6`. The tip panel geometry is given by [`fold_station_geometry`](@ref).

!!! details "🔃 Inputs and Outputs"
    **Inputs:**
    - `wing::TASOPT.structures.Wing`: Wing structure, with the folding wing tip `wing.fold`.
    - `loads::NamedTuple`: Fold station loads from [`fold_station_loads`](@ref), of which `W_tip` and `dyW_tip` are used.

    **Outputs:**
    - `W::Float64`: Weight of the fold actuator per wing side [N].
    - `details::NamedTuple`: Actuator moment `M_act`, gravity moment `M_grav` and wind moment `M_wind` [N⋅m].
"""
function size_fold_actuator(wing::Wing, loads)
    fold = wing.fold
    co = wing.layout.root_chord
    λt = wing.outboard.λ
    λ_f, ℓ = fold_station_geometry(wing, fold.η_fold)

    # PLACEHOLDER: normal-force coefficient of the tip panel, taken as that of a flat plate
    C_N = 1.2

    ℓ_cg = loads.W_tip == 0 ? 0.0 : loads.dyW_tip / loads.W_tip
    M_grav = loads.W_tip * ℓ_cg

    q = 0.5 * atmos(0.0).ρ * fold.ground_wind_speed^2
    M_wind = q * C_N * co * ℓ^2 * (λ_f + 2 * λt) / 6

    M_act = M_grav + M_wind
    W = gee * M_act * fold.actuator_torque_density

    return W, (; M_act, M_grav, M_wind)
end

"""
    fold_cap_strength(wing::Wing, t_cap, sigfac) -> Float64

Force capacity of one wing-box cap at the fold, `σ_cap t_cap w̄ c_perp_f²`, before the fitting factor. The allowable
cap stress is `σ_cap = wing.outboard.caps.material.σmax sigfac`, `t_cap` is the outboard cap thickness normalised by the
chord perpendicular to the sweep (`tbcap` from [`size_wing_section!`](@ref)), `w̄ = wing.outboard.cross_section.width_to_chord`
is the box width over the chord and `c_perp_f` is the chord at the fold station perpendicular to the sweep from
[`fold_station_geometry`](@ref).

The outboard gauges are sized at the panel break and kept to the tip, so the cap at the fold is stronger than the fold
loads require. A joint matched to the strength of the box it joins is therefore an upper bound on the joint size, while
a joint sized for the fold loads alone is a lower bound. [`size_fold_mechanism!`](@ref) uses the larger of the two.

!!! details "🔃 Inputs and Outputs"
    **Inputs:**
    - `wing::TASOPT.structures.Wing`: Wing structure, with the folding wing tip `wing.fold`.
    - `t_cap::Float64`: Outboard cap thickness normalised by the chord perpendicular to the sweep [-].
    - `sigfac::Float64`: Stress factor applied to the allowable stresses (as in the wing box).

    **Outputs:**
    - `Float64`: Force capacity of one wing-box cap at the fold [N].
"""
function fold_cap_strength(wing::Wing, t_cap, sigfac)
    σ_cap = wing.outboard.caps.material.σmax * sigfac
    w̄ = wing.outboard.cross_section.width_to_chord
    c_perp_f = fold_station_geometry(wing, wing.fold.η_fold).c_perp_f

    return σ_cap * t_cap * w̄ * c_perp_f^2
end

"""
    size_fold_mechanism!(wing::Wing, loads, sigfac, t_cap)

Sizes the fold mechanism of the folding wing tip from the loads at the fold station, and stores the loads and weights
in the outputs of `wing.fold`. It writes only to `wing.fold`.

The hinge is streamwise, as on the 777X, which is consistent with the spanwise fold station and the ground span `2 y_f`.
The wing-box cap force couple has two candidates. The load-based force is `F_cap_load = abs(M_hinge / h_box_eff)`, where
`M_hinge = M_f / cos(Λ)` is the bending moment in the plane perpendicular to the sweep, as in the wing box, which is
conservative for the streamwise hinge, and `h_box_eff` is the effective box depth at the fold station from
[`fold_station_geometry`](@ref). A negative `M_f`, where the tip weight relief exceeds the aerodynamic moment, still
loads the caps, which must resist it, so `F_cap_load` uses the magnitude. The strength-based force `F_cap_strength` is the
force capacity of the outboard cap at the fold from [`fold_cap_strength`](@ref). The more restrictive of the two governs,
`F_cap = fitting_factor max(F_cap_load, F_cap_strength)`. The weight per wing side is the sum of the hinge pins and lugs
(which carry the resultant of `F_cap` and the shear `S_f`, sized with [`size_pinned_joint`](@ref)), the latches
([`size_latches`](@ref)), the load introduction structure ([`size_load_introduction`](@ref), with ribs of the wing-box
cap material), the actuator ([`size_fold_actuator`](@ref)) and the fold systems, `fold.systems_mass` per wing side.

The weight outputs `W_fold`, `W_hinge_lugs`, `W_latches`, `W_load_intro` and `W_actuator` are totals for both wing sides.
The bending relief `dyW_fold` is per wing side. The sweep offset `dxW_fold` is for both wing sides.

!!! details "🔃 Inputs and Outputs"
    **Inputs:**
    - `wing::TASOPT.structures.Wing`: Wing structure, with the folding wing tip `wing.fold`.
    - `loads::NamedTuple`: Fold station loads from [`fold_station_loads`](@ref), of which `S_f`, `M_f`, `W_tip` and `dyW_tip` are used.
    - `sigfac::Float64`: Stress factor applied to the allowable stresses (as in the wing box).
    - `t_cap::Float64`: Outboard cap thickness normalised by the chord perpendicular to the sweep, `tbcap` from [`size_wing_section!`](@ref) [-].

    **Outputs:** (stored in `wing.fold`)
    - `S_fold::Float64`: Shear at the fold station, per wing side [N].
    - `M_fold::Float64`: Bending moment at the fold station, per wing side [N⋅m].
    - `F_cap::Float64`: Wing-box cap force couple at the fold, including the fitting factor [N].
    - `F_cap_strength::Float64`: Force capacity of one wing-box cap at the fold, before the fitting factor [N].
    - `W_hinge_lugs::Float64`, `W_latches::Float64`, `W_load_intro::Float64`, `W_actuator::Float64`: Component weights, both wing sides [N].
    - `W_fold::Float64`: Total fold mechanism weight, both wing sides [N].
    - `dyW_fold::Float64`: Bending relief of the fold weight about the break station, per wing side [N⋅m].
    - `dxW_fold::Float64`: Moment of the total fold weight about the wing root from the sweep offset, both wing sides [N⋅m].
"""
function size_fold_mechanism!(wing::Wing, loads, sigfac, t_cap)
    fold = wing.fold
    layout = wing.layout
    b = layout.span
    Λ = layout.sweep
    geometry = fold_station_geometry(wing, fold.η_fold)

    M_hinge = loads.M_f / cosd(Λ)
    F_cap_load = abs(M_hinge / geometry.h_box_eff)
    F_cap_strength = fold_cap_strength(wing, t_cap, sigfac)
    F_cap = fold.fitting_factor * max(F_cap_load, F_cap_strength)

    # The hinge pins carry the resultant of the cap force and the shear
    W_hinge_lugs, _ = size_pinned_joint(hypot(F_cap, loads.S_f), fold.n_hinge, fold, sigfac)
    W_latches, _ = size_latches(F_cap, fold, sigfac)
    W_load_intro, _ = size_load_introduction(F_cap, geometry.h_box_eff, geometry.w_box, fold, wing.outboard.caps.ρ, sigfac)
    W_actuator, _ = size_fold_actuator(wing, loads)
    W_systems = gee * fold.systems_mass
    W_side = W_hinge_lugs + W_latches + W_load_intro + W_actuator + W_systems

    fold.S_fold = loads.S_f
    fold.M_fold = loads.M_f
    fold.F_cap = F_cap
    fold.F_cap_strength = F_cap_strength
    fold.W_hinge_lugs = 2 * W_hinge_lugs
    fold.W_latches = 2 * W_latches
    fold.W_load_intro = 2 * W_load_intro
    fold.W_actuator = 2 * W_actuator
    # Includes the fold systems, which have no output field of their own
    fold.W_fold = 2 * W_side
    fold.dyW_fold = W_side * (fold.η_fold - layout.ηs) * b / 2
    fold.dxW_fold = fold.W_fold * (geometry.y_f - layout.ηo * b / 2) * tand(Λ)

    return nothing
end

"""
    fold_tip_fuel_fraction(wing::Wing, η_f) -> Float64

    # This is a placeholder function. The final implementation will have the wings broken up into sections such that the folding wingtip does not need to estimate its fuel volume fraction and deduce it from the outboard wing panel...

Fraction of the fuel volume of the outer wing panel that lies inboard of the fold station `η_f`, for the outer-panel
fuel volume `Vout` of the fuel tank, which only fills the wing box inboard of the fold. It can also be used as an
approximation for the sweep term `dxVout`. The fuel volume is proportional to the chord squared, `λ²`, which varies linearly from the
inboard value `λs` at the panel break `ηs` to the tip value `λt` at the tip, so with `s = (η - ηs) / (1 - ηs)` the fraction is
`∫₀ˢᶠ λ² ds / ∫₀¹ λ² ds`, evaluated in closed form. It is `0` at the panel break and `1` at the tip.

Returns `1.0` when `wing.fold.has_fold` is `false`, so a caller can multiply by it unconditionally. It is used by
[`wing_weights!`](@ref) for the outer-panel fuel terms.

!!! details "🔃 Inputs and Outputs"
    **Inputs:**
    - `wing::TASOPT.structures.Wing`: Wing structure.
    - `η_f::Float64`: Spanwise station of the fold, as a fraction of the half-span.

    **Outputs:**
    - `Float64`: Fraction of the outer-panel fuel volume inboard of the fold.
"""
function fold_tip_fuel_fraction(wing::Wing, η_f)
    wing.fold.has_fold || return 1.0

    λs, Δλ, s_f = fold_tip_fuel_chord_geometry(wing, η_f)
    volume(s) = λs^2 * s + λs * Δλ * s^2 + Δλ^2 * s^3 / 3 # ∫₀ˢ λ² ds

    return volume(s_f) / volume(1.0)
end

"""
    fold_tip_fuel_moment_fraction(wing::Wing, η_f) -> Float64

Fraction of the spanwise moment of the outer-panel fuel volume about the panel break that comes from the part inboard
of the fold station `η_f`, for the fuel volume moment `dyVout` of the fuel tank. As in [`fold_tip_fuel_fraction`](@ref),
the fuel volume is proportional to `λ²`, so with `s = (η - ηs) / (1 - ηs)` the fraction is `∫₀ˢᶠ λ² s ds / ∫₀¹ λ² s ds`,
evaluated in closed form. It is `0` at the panel break and `1` at the tip.

Returns `1.0` when `wing.fold.has_fold` is `false`, so a caller can multiply by it unconditionally. It is used by
[`wing_weights!`](@ref) for the outer-panel fuel terms.

!!! details "🔃 Inputs and Outputs"
    **Inputs:**
    - `wing::TASOPT.structures.Wing`: Wing structure.
    - `η_f::Float64`: Spanwise station of the fold, as a fraction of the half-span.

    **Outputs:**
    - `Float64`: Fraction of the outer-panel fuel volume moment inboard of the fold.
"""
function fold_tip_fuel_moment_fraction(wing::Wing, η_f)
    wing.fold.has_fold || return 1.0

    λs, Δλ, s_f = fold_tip_fuel_chord_geometry(wing, η_f)
    moment(s) = λs^2 * s^2 / 2 + 2 * λs * Δλ * s^3 / 3 + Δλ^2 * s^4 / 4 # ∫₀ˢ λ² s ds

    return moment(s_f) / moment(1.0)
end

# Inboard chord ratio `λs`, its change `Δλ` to the tip, and the fold station `s_f` as a fraction of the outer panel
function fold_tip_fuel_chord_geometry(wing::Wing, η_f)
    ηs = wing.layout.ηs
    λs = wing.inboard.λ
    return λs, wing.outboard.λ - λs, (η_f - ηs) / (1 - ηs)
end

"""
    fold_ground_margins!(wing::Wing, fuselage::Fuselage, vtail::Tail, landing_gear)

Calculates the span and height of the aircraft with the wing tips folded on the ground, and the margins to the span limit
`wing.layout.max_span` and to the top of the vertical tail, and stores them in `wing.fold`. It writes only to `wing.fold`.
The margins are soft: they are reported but not enforced, so a negative margin does not change the sizing.

With the tips folded the span is `2 y_f`, where `y_f` is the fold station from [`fold_station_geometry`](@ref), and the span
margin is `max_span - 2 y_f`. The height is that of the highest point of the folded tip, taken as the hinge line at the top
of the wing box with the tip panel rotated up to vertical. With the wing dihedral `Γ` the hinge is raised by
`(y_f - y_o) tan(Γ)` relative to the wing root, where `y_o = ηo b/2` is the wing root station, and the tip panel of spanwise
length `ℓ` rises by `ℓ cos(Γ)`, so
`z_tip = z_wing + (y_f - y_o) tan(Γ) + h_box_eff / 2 + ℓ cos(Γ)`. The height margin is the fin top height
`R_fuse + b_vtail` minus `z_tip`.

The heights are in the frame of `wing.layout.z` with the aircraft level on the ground, so the landing gear height cancels.
The span `b` is already the span projected on the horizontal plane, so no `cos(Γ)` factor is needed. The fin root is assumed
to be at the fuselage crown, and the fin top is the height reference because it is the highest point of the aircraft today;
a T-tail would change this reference. Aircraft with more than one vertical tail (`ntails > 1`) are not handled. The dihedral
is read from `landing_gear.wing_dihedral_angle` and is not modified.

Without a fold (`wing.fold.has_fold` is `false`) the ground span is the wing span, the span margin is `max_span - b`, and
`z_tip` and `height_margin` are `0`.

!!! details "🔃 Inputs and Outputs"
    **Inputs:**
    - `wing::TASOPT.structures.Wing`: Wing structure, with the folding wing tip `wing.fold`.
    - `fuselage::TASOPT.structures.Fuselage`: Fuselage, of which the radius is used.
    - `vtail::TASOPT.structures.Tail`: Vertical tail, of which the span (fin height) is used.
    - `landing_gear::TASOPT.structures.LandingGear`: Landing gear, of which the wing dihedral angle [rad] is used.

    **Outputs:** (stored in `wing.fold`)
    - `ground_span::Float64`: Span with the wing tips folded [m].
    - `span_margin::Float64`: Span limit minus the ground span [m].
    - `z_tip::Float64`: Height of the folded wing tip in the frame of `wing.layout.z` [m].
    - `height_margin::Float64`: Fin top height minus `z_tip` [m].
"""
function fold_ground_margins!(wing::Wing, fuselage::Fuselage, vtail::Tail, landing_gear)
    fold = wing.fold
    layout = wing.layout
    b = layout.span

    if fold.has_fold
        geometry = fold_station_geometry(wing, fold.η_fold)
        y_o = layout.ηo * b / 2
        Γ = landing_gear.wing_dihedral_angle

        z_hinge = layout.z + (geometry.y_f - y_o) * tan(Γ) + geometry.h_box_eff / 2
        z_fin = fuselage.layout.radius + vtail.layout.span

        fold.ground_span = 2 * geometry.y_f
        fold.z_tip = z_hinge + geometry.ℓ * cos(Γ)
        fold.height_margin = z_fin - fold.z_tip
    else
        fold.ground_span = b
        fold.z_tip = 0.0
        fold.height_margin = 0.0
    end
    fold.span_margin = layout.max_span - fold.ground_span

    return nothing
end
