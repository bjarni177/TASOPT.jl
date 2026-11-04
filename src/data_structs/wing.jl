using ..aerodynamics
abstract type AbstractWing end

"""
$TYPEDEF

Wing Strut

$TYPEDFIELDS
"""
@kwdef mutable struct Strut
    """Strut Material """
    material::StructuralAlloy = StructuralAlloy("TASOPT-Al")
    """Strut Area [m^2] """
    S::Float64 = 0
    """Strut Length [m] """
    length::Float64 = 0
    """Strut z position [m]"""
    z::Float64 = 0
    """Strut thickness to chord"""
    thickness_to_chord::Float64 = 0
    """Strut local velocity ratio"""
    local_velocity_ratio::Float64 = 0
    """Cosine of strut sweep angle"""
    cos_lambda::Float64 = 0 
    """Strut weight [N]"""
    weight::Float64 = 0
    """Strut axial force [N]"""
    axial_force::Float64 = 0
    """Strut chord [m]"""
    chord::Float64 = 0
    """Aircraft pitching moment contribution from the weight distribution of the strut [N m]"""
    dxW::Float64 = 0
end

"""
$TYPEDEF

The Wing structure is composed of 7 sub-structures as follow and are visualized [here](../assets/wing_struct.png).

1. General Properties
2. Wing Layout
3. Material
4. Wing Sections
5. Strut
6. Folding Wingtip
7. Weight Fractions

See [`structures.WingSection`](@ref), [`structures.WingLayout`](@ref) or [`structures.WingCrossSection`](@ref) for more detail.

$TYPEDFIELDS
"""
@kwdef mutable struct Wing{air<:aerodynamics.airfoil} <: AbstractWing
    """Wing Weight [N] """
    weight::Float64 = 0.0
    """Aircraft pitching moment contribution from the weight distribution of the wing [Nm]"""
    dxW::Float64 = 0.0
    """Wing Layout """
    layout::WingLayout = WingLayout()
    """Wing Material """
    material::StructuralAlloy = StructuralAlloy("TASOPT-Al")

    """Airfoil data"""
    airsection::air = aerodynamics.airtable(joinpath(__TASOPTroot__,"airfoil_data/C.air"))

    """Inboard Wing Section (at wing root)"""
    inboard::WingSection = WingSection() # at wing root 
    """Outboard Wing Section (at strut attachment point)"""
    outboard::WingSection = WingSection() # at strut attachment point
    """Span fraction of inner wing break ("snag")"""
    # ηs::Float64 = 0 

    """Fuselage lift carryover factor"""
    fuse_lift_carryover::Float64 = 0.0
    """Tip lift roll-off factor"""
    tip_lift_loss::Float64 = 0.0

    """Mean Aerodynamic Chord"""
    mean_aero_chord::Float64 = 0.0

    """Wing Strut"""
    has_strut::Bool = false
    strut::Strut = Strut()
    
    """Folding wingtip mechanism"""
    fold::FoldingWingTip = FoldingWingTip()

    """Wing flap weight fraction"""
    weight_frac_flap::Float64 = 0.0
    """Wing slats weight fraction"""
    weight_frac_slat::Float64 = 0.0
    """Wing ailerons weight fraction"""
    weight_frac_ailerons::Float64 = 0.0
    """Wing leading_trailing_edge weight fraction"""
    weight_frac_leading_trailing_edge::Float64 = 0.0
    """Wing ribs weight fraction"""
    weight_frac_ribs::Float64 = 0.0
    """Wing spoilers weight fraction"""
    weight_frac_spoilers::Float64 = 0.0
    """Wing attachments weight fraction"""
    weight_frac_attachments::Float64 = 0.0

end

"""
$TYPEDEF

Tail

$TYPEDFIELDS
"""
@kwdef mutable struct Tail <: AbstractWing
    """Tail Layout """
    layout::WingLayout = WingLayout()
    """Tail Sections """
    outboard::WingSection= WingSection()
    inboard::WingSection = WingSection()
    """Tail Strut"""
    has_strut::Bool = false
    strut::Strut = Strut()
    """Tip lift roll-off factor"""
    tip_lift_loss::Float64 = 0.0
    """Aircraft pitching moment contribution from the weight distribution of the strut [N m]"""
    dxW::Float64 = 0
    """Tail Weight [N] """
    weight::Float64 = 0
    """Tail Added Weight Fraction"""
    weight_fraction_added::Float64 = 0
    """Tail Max CL """
    CL_max::Float64 = 0
    """Tail Volume [m^3] """
    volume::Float64 = 0
    """Tail Sizing assumption selection - different for HTail vs VTail """
    opt_sizing::TailSizing.T = TailSizing.FixedVh
    """Tail Downwash factor dε/dα """
    downwash_factor::Float64 = 0
    """Tail max fwd CG (only used if opt_sizing == "CLmax_fwdCG" for HTail) """
    CL_max_fwd_CG::Float64 = 0
    """Tail Minimum static margin"""
    SM_min::Float64 = 0
    """Max Tail down load. Tail download param at max load case"""
    CL_CLmax::Float64 = 0
    """Number of Tails"""
    ntails::Float64 = 0
end

function wing_additional_weight(wing::AbstractWing)
    if wing isa Wing
        return wing.weight_frac_flap + wing.weight_frac_slat + wing.weight_frac_ailerons + 
        wing.weight_frac_leading_trailing_edge + wing.weight_frac_ribs +
        wing.weight_frac_spoilers + wing.weight_frac_attachments
    elseif wing isa Tail
        return wing.weight_fraction_added
    end
    
end

"""
    fold_station_geometry(wing::Wing, η_f) -> NamedTuple

Calculates the planform and wing-box geometry at the spanwise fold station `η_f`, which lies on the outboard
panel (`ηs ≤ η_f ≤ 1`). The chord varies linearly between the break chord ratio `λs` and the tip chord ratio `λt`.
The box height and width are measured perpendicular to the sweep, using the normalized gauges of the outboard cross-section,
since the outer panel keeps the same normalized gauges as at the panel break `ηs`.

!!! details "🔃 Inputs and Outputs"
    **Inputs:**
    - `wing::TASOPT.structures.Wing`: Wing structure.
    - `η_f::Float64`: Fold station as a fraction of the semi-span.

    **Outputs:** `NamedTuple` with fields
    - `λ_f::Float64`: Chord ratio `c_f / c_root` at the fold station.
    - `ℓ::Float64`: Spanwise length of the wing outboard of the fold station, `(b/2)(1 - η_f)` [m].
    - `y_f::Float64`: Spanwise position of the fold station from the centerline, `η_f b/2` [m].
    - `c_f::Float64`: Streamwise chord at the fold station [m].
    - `c_perp_f::Float64`: Chord at the fold station perpendicular to the sweep, `c_f cos(Λ)` [m].
    - `h_box_eff::Float64`: Effective (root-mean-square) wing-box height at the fold station [m].
    - `w_box::Float64`: Wing-box width at the fold station [m].
"""
function fold_station_geometry(wing::Wing, η_f)
    layout = wing.layout
    cross_section = wing.outboard.cross_section

    λs = wing.inboard.λ
    λt = wing.outboard.λ

    λ_f = λs + (λt - λs) * (η_f - layout.ηs) / (1 - layout.ηs)
    ℓ = (layout.span / 2) * (1 - η_f)
    y_f = η_f * layout.span / 2
    c_f = layout.root_chord * λ_f
    c_perp_f = c_f * cosd(layout.sweep)

    _, h_rms = get_average_sparbox_heights(cross_section)
    h_box_eff = h_rms * c_perp_f
    w_box = cross_section.width_to_chord * c_perp_f

    return (; λ_f, ℓ, y_f, c_f, c_perp_f, h_box_eff, w_box)
end

"""
"""
function Base.getproperty(obj::AbstractWing, sym::Symbol)
    concrete_type = typeof(obj)  # Gets the actual type (Wing or Tail)
    if hasfield(concrete_type, sym)
        return getfield(obj, sym)
    elseif hasfield(WingLayout, sym)
        return getfield(obj.layout, sym)
    else
        throw(KeyError("Property $sym not found in $(concrete_type) or WingLayout"))
    end
end