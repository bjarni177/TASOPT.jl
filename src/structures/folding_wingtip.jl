"""
    fold_station_loads(wing::Wing, po, γt, γs, Nload, K_w) -> NamedTuple

Calculates the shear and bending moment at the fold station `wing.fold.η_fold`, per wing side, from the lift and the
structural weight of the wing outboard of the fold. The lift is the outer-panel loading of [`wing_weights!`](@ref),
including the tip roll-off point load. The weight excludes fuel, engines and the fold mechanism.

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

Sizes `n` identical pins and lugs that together carry the force `F`, and returns the joint weight per wing side.
The pins are in double shear and the lug thickness is set by the bearing stress, with the allowable stresses scaled
by `sigfac` as in the wing box. The pin diameter is `d = √(2F / (n π τ_allow))` and the lug thickness is
`t = F / (n d σ_br_allow)`. A zero force gives a zero weight joint. Used by [`size_latches`](@ref) and
[`size_fold_mechanism!`](@ref).

!!! details "🔃 Inputs and Outputs"
    **Inputs:**
    - `F::Float64`: Total force carried by the joint [N].
    - `n::Int`: Number of pins sharing the force.
    - `fold::TASOPT.structures.FoldingWingTip`: Folding wing tip.
    - `sigfac::Float64`: Stress factor.

    **Outputs:**
    - `W::Float64`: Weight of the pins and lugs per wing side [N].
    - `details::NamedTuple`: Pin diameter `d`, lug thickness `t`, lug radius `R` and lug length `L` [m].
"""
function size_pinned_joint(F, n, fold::FoldingWingTip, sigfac)
    # Avoids dividing by zero in the pin diameter
    F == 0 && return 0.0, (; d = 0.0, t = 0.0, R = 0.0, L = 0.0)

    ultimate_factor = 1.5 # Ultimate to limit load factor, as in the wing box safety factor
    # PLACEHOLDER: lug stack of a middle lug (t) between two clevis ears (t/2 each), pin as long as the stack
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

Sizes the `fold.n_latch` latches that carry the wing-box cap force with [`size_pinned_joint`](@ref), and adds the
actuator mass of each latch.

!!! details "🔃 Inputs and Outputs"
    **Inputs:**
    - `F_cap::Float64`: Wing-box cap force couple at the fold [N].
    - `fold::TASOPT.structures.FoldingWingTip`: Folding wing tip.
    - `sigfac::Float64`: Stress factor.
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
It consists of one fitting per cap (two per wing side) and two full-depth closing ribs, one on each side of the fold.
Each fitting carries `F_cap` at the allowable stress of the fitting material, scaled by `sigfac`, over a length of
`fold.fitting_length_factor` times the box depth. Each rib has the box cross-section and the gauge `fold.rib_min_gauge`.

!!! details "🔃 Inputs and Outputs"
    **Inputs:**
    - `F_cap::Float64`: Wing-box cap force couple at the fold [N].
    - `h_box::Float64`: Wing-box depth at the fold [m].
    - `w_box::Float64`: Wing-box width at the fold [m].
    - `fold::TASOPT.structures.FoldingWingTip`: Folding wing tip.
    - `ρ_rib::Float64`: Density of the rib material [kg/m³].
    - `sigfac::Float64`: Stress factor.

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
is the gravity moment of the tip structure plus the moment of the ground wind load on the tip panel, and the actuator
mass is this moment times `fold.actuator_torque_density`. The wind is a uniform pressure `q = ½ ρ V²` at
`fold.ground_wind_speed` and sea-level density, normal to the tip panel.

!!! details "🔃 Inputs and Outputs"
    **Inputs:**
    - `wing::TASOPT.structures.Wing`: Wing structure, with the folding wing tip `wing.fold`.
    - `loads::NamedTuple`: Fold station loads from [`fold_station_loads`](@ref).

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

Force capacity of one wing-box cap at the fold, before the fitting factor: `σ_cap t_cap w̄ c_perp_f²`, with the
allowable cap stress `σ_cap`, the cap thickness `t_cap`, the box width over chord `w̄` and the chord `c_perp_f`
at the fold perpendicular to the sweep.

The outboard gauges are sized at the panel break and kept to the tip, so the cap at the fold is stronger than the fold
loads require. [`size_fold_mechanism!`](@ref) sizes the joint for the larger of the two.

!!! details "🔃 Inputs and Outputs"
    **Inputs:**
    - `wing::TASOPT.structures.Wing`: Wing structure, with the folding wing tip `wing.fold`.
    - `t_cap::Float64`: Outboard cap thickness normalised by the chord perpendicular to the sweep [-].
    - `sigfac::Float64`: Stress factor.

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

Sizes the fold mechanism from the loads at the fold station, and stores the loads and weights in `wing.fold`.

The hinge is streamwise, as on the 777X. The wing-box cap force is
`F_cap = fitting_factor max(F_cap_load, F_cap_strength)`, with `F_cap_load = abs(M_hinge / h_box_eff)` and
`M_hinge = M_f / cos(Λ)` (conservative for a streamwise hinge), and `F_cap_strength` from [`fold_cap_strength`](@ref).
The magnitude of `M_f` is used because the caps must also resist a negative moment. The weight per wing side is the sum of
the hinge pins and lugs (carrying the resultant of `F_cap` and `S_f`), the latches, the load introduction structure, the
actuator and the fold systems, `fold.systems_mass`.

The weights `W_fold`, `W_hinge_lugs`, `W_latches`, `W_load_intro` and `W_actuator` are totals for both wing sides.
The bending relief `dyW_fold` is per wing side and the sweep moment `dxW_fold` is for both wing sides.

!!! details "🔃 Inputs and Outputs"
    **Inputs:**
    - `wing::TASOPT.structures.Wing`: Wing structure, with the folding wing tip `wing.fold`.
    - `loads::NamedTuple`: Fold station loads from [`fold_station_loads`](@ref).
    - `sigfac::Float64`: Stress factor.
    - `t_cap::Float64`: Outboard cap thickness normalised by the chord perpendicular to the sweep (`tbcap`) [-].

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

    # Hinge pins carry the resultant of the cap force and the shear
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
    # Includes the fold systems mass
    fold.W_fold = 2 * W_side
    fold.dyW_fold = W_side * (fold.η_fold - layout.ηs) * b / 2
    fold.dxW_fold = fold.W_fold * (geometry.y_f - layout.ηo * b / 2) * tand(Λ)

    return nothing
end

"""
    fold_tip_fuel_fraction(wing::Wing, η_f) -> Float64

Fraction of the outer-panel fuel volume `Vout` that lies inboard of the fold station `η_f`, since the fuel tank only
fills the wing box inboard of the fold. The fuel volume is taken proportional to `λ²`, which gives
`∫₀ˢᶠ λ² ds / ∫₀¹ λ² ds`, with `s = (η - ηs) / (1 - ηs)`. Returns `1.0` when `wing.fold.has_fold` is `false`.
Used by [`wing_weights!`](@ref).

!!! details "🔃 Inputs and Outputs"
    **Inputs:**
    - `wing::TASOPT.structures.Wing`: Wing structure.
    - `η_f::Float64`: Spanwise station of the fold, as a fraction of the half-span.

    **Outputs:**
    - `Float64`: Fraction of the outer-panel fuel volume inboard of the fold.
"""
function fold_tip_fuel_fraction(wing::Wing, η_f)
    # PLACEHOLDER: estimates the fuel volume from the outboard panel instead of a separately sectioned wing
    wing.fold.has_fold || return 1.0

    λs, Δλ, s_f = fold_tip_fuel_chord_geometry(wing, η_f)
    volume(s) = λs^2 * s + λs * Δλ * s^2 + Δλ^2 * s^3 / 3 # ∫₀ˢ λ² ds

    return volume(s_f) / volume(1.0)
end

"""
    fold_tip_fuel_moment_fraction(wing::Wing, η_f) -> Float64

Fraction of the spanwise moment `dyVout` of the outer-panel fuel volume about the panel break that comes from the part
inboard of the fold station `η_f`. As in [`fold_tip_fuel_fraction`](@ref), the fraction is
`∫₀ˢᶠ λ² s ds / ∫₀¹ λ² s ds`. Returns `1.0` when `wing.fold.has_fold` is `false`. Used by [`wing_weights!`](@ref).

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

Calculates the ground span with the wing tips folded and the height of the folded tip, and the margins to the span limit
`wing.layout.max_span` and to the top of the vertical tail, and stores them in `wing.fold`. The margins are reported but
not enforced.

The folded span is `2 y_f`. The tip height is taken at the hinge line at the top of the wing box plus the tip panel
rotated up to vertical, `z_tip = z_wing + (y_f - y_o) tan(Γ) + h_box_eff / 2 + ℓ cos(Γ)`, in the frame of
`wing.layout.z`. The height margin is the fin top height `R_fuse + b_vtail` minus `z_tip`, assuming the fin root is at the
fuselage crown. Aircraft with more than one vertical tail (`ntails > 1`) are not handled.

Without a fold the ground span is the wing span, and `z_tip` and `height_margin` are `0`.

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
