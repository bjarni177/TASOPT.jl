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

    # Geometry inputs
    """Strut Area [m^2] """
    S::Float64 = 0
    """Strut z position [m]"""
    z::Float64 = 0
    """Strut y position at the fuselage attachment [m]"""
    y_root::Float64 = 0
    """Strut thickness to chord"""
    thickness_to_chord::Float64 = 0
    """Strut box wall thickness to depth ratio"""
    wall_to_depth::Float64 = 0.05
    """Strut box width to chord ratio"""
    box_width_to_chord::Float64 = 0.5
    """Strut box web height to box height ratio"""
    web_to_box_height::Float64 = 0.75
    """Euler effective-length factor of the strut end fixity"""
    end_fixity_K::Float64 = 1.0
    """Strut local velocity ratio"""
    local_velocity_ratio::Float64 = 0

    # Computed geometry
    """Strut Length [m] """
    length::Float64 = 0
    """Strut chord [m]"""
    chord::Float64 = 0
    """Strut cross-sectional area [m^2]"""
    area::Float64 = 0
    """Cosine of strut sweep angle"""
    cos_lambda::Float64 = 0 
    """Strut dihedral angle [rad]"""
    dihedral::Float64 = 0

    # Loads
    """Maximum strut tension force [N]"""
    axial_force::Float64 = 0
    """Strut axial force in the most compressive case [N]"""
    axial_force_compression::Float64 = 0
    """Strut Euler buckling load P_cr [N]"""
    buckling_load::Float64 = 0
    """Strut minor-axis bending stiffness [N m^2]"""
    EI_min::Float64 = 0
    """Governing strut sizing criterion: `:tension`, `:buckling` or `:gauge`"""
    sizing_criterion::Symbol = :none

    # Aerodynamics
    """Strut lift coefficient"""
    CL::Float64 = 0
    """Strut lift divided by total lift"""
    lift_fraction::Float64 = 0

    # Weights
    """Strut weight [N]"""
    weight::Float64 = 0
    """Added weight fraction for fairings and fittings"""
    weight_frac_added::Float64 = 0.60 # Placeholder - derrived from Boeing SUGAR Phase IV, NASA CR, NTRS 20220016017, Tables 12/16 ... may also contain the weight of the jury strut too
    """Aircraft pitching moment contribution from the weight distribution of the strut [N m]"""
    dxW::Float64 = 0
end

"""
$TYPEDEF

Jury strut connecting the wing to the main strut

$TYPEDFIELDS
"""
@kwdef mutable struct JuryStrut
    """Jury strut Material """
    material::StructuralAlloy = StructuralAlloy("TASOPT-Al")
    """Wing-side attachment, as a fraction of the wing semispan"""
    eta_wing::Float64 = 0
    """Strut-side attachment, as a fraction of the strut length"""
    eta_strut::Float64 = 0
    """Jury strut length [m]"""
    length::Float64 = 0
    """Jury strut cross-sectional area [m^2]"""
    area::Float64 = 0
    """Jury strut axial force [N]"""
    axial_force::Float64 = 0
    """Jury strut Euler buckling load [N]"""
    buckling_load::Float64 = 0
    """Jury strut weight [N]"""
    weight::Float64 = 0
    """Added weight fraction for fairings and fittings"""
    weight_frac_added::Float64 = 0.60 # Placeholder - Assumed to be the same value as for the main strut
    """Aircraft pitching moment contribution from the weight distribution of the jury strut [N m]"""
    dxW::Float64 = 0
end

"""
$TYPEDEF

The Wing structure is composed of 6 sub-structures as follow and are visualized [here](../assets/wing_struct.png).

1. General Properties
2. Wing Layout
3. Material
4. Wing Sections
5. Strut
6. Weight Fractions

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