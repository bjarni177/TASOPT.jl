"""
$TYPEDEF

Folding wingtip: inputs and sizing outputs. Numeric inputs default to zero so that a missing value fails
the input checks when `has_fold = true`.

$TYPEDFIELDS
"""
@kwdef mutable struct FoldingWingTip
    # Inputs
    """Wingtip folds (true) or is fixed (false)"""
    has_fold::Bool = false
    """Span fraction of the fold hinge line"""
    η_fold::Float64 = 0.0
    """Number of hinges"""
    n_hinge::Int = 0
    """Number of latches, at least 2 so that no single failure unlocks the tip in flight"""
    n_latch::Int = 0
    """Fitting factor on fittings (14 CFR 25.625) [-]"""
    fitting_factor::Float64 = 0.0
    """Pin material"""
    material_pin::StructuralAlloy = StructuralAlloy("300M")
    """Lug material"""
    material_lug::StructuralAlloy = StructuralAlloy("300M")
    """Fitting material"""
    material_fitting::StructuralAlloy = StructuralAlloy("Ti-6Al-4V")
    """Pin bearing strength [Pa]"""
    σ_br_pin::Float64 = materials.MaterialProperties["300M"]["bearing_strength"]
    """Lug bearing strength [Pa]"""
    σ_br_lug::Float64 = materials.MaterialProperties["300M"]["bearing_strength"]
    """Lug outer radius over pin diameter"""
    lug_R_over_d::Float64 = 0.0
    """Lug length from the pin centre to the lug base over pin diameter"""
    lug_L_over_d::Float64 = 0.0
    """Fitting length over local wing box height"""
    fitting_length_factor::Float64 = 0.0
    """PLACEHOLDER input: minimum rib gauge thickness [m]"""
    rib_min_gauge::Float64 = 0.0
    """PLACEHOLDER input: actuator mass per unit hinge moment [kg/(N m)]. The shipped inputs use 0.01 (100 N m/kg) from industrial steel/iron helical hydraulic rotary actuators (Eckart SM4: 69-100 N m/kg; Moveco RR.58: 12.7 kN m, 185 kg), an aerospace actuator is expected to be lighter"""
    actuator_torque_density::Float64 = 0.0
    """Ground wind speed for the hinge load case [m/s]"""
    ground_wind_speed::Float64 = 0.0
    """Fold systems mass (controls, sensors, wiring) [kg]"""
    systems_mass::Float64 = 0.0
    """Fold drag increment [drag counts]"""
    ΔCD_fold_counts::Float64 = 0.0

    # Outputs
    """Shear force at the fold station [N]"""
    S_fold::Float64 = 0.0
    """Bending moment at the fold station [N m]"""
    M_fold::Float64 = 0.0
    """Wing-box cap force couple at the fold, including the fitting factor [N]"""
    F_cap::Float64 = 0.0
    """Force capacity of one wing-box cap at the fold, before the fitting factor [N]"""
    F_cap_strength::Float64 = 0.0
    """Hinge and lug weight [N]"""
    W_hinge_lugs::Float64 = 0.0
    """Latch weight [N]"""
    W_latches::Float64 = 0.0
    """Load introduction (fittings and ribs) weight [N]"""
    W_load_intro::Float64 = 0.0
    """Actuator weight [N]"""
    W_actuator::Float64 = 0.0
    """Total fold mechanism weight [N]"""
    W_fold::Float64 = 0.0
    """Fold weight times its spanwise arm (bending relief) [N m]"""
    dyW_fold::Float64 = 0.0
    """Aircraft pitching moment contribution from the fold weight [N m]"""
    dxW_fold::Float64 = 0.0
    """Fold drag coefficient increment"""
    CD_fold::Float64 = 0.0
    """Height of the folded wingtip in the wing z frame (as `wing.layout.z`) [m]"""
    z_tip::Float64 = 0.0
    """Span with the wingtips folded, 2 y_fold [m]"""
    ground_span::Float64 = 0.0
    """Span limit minus the folded span (`max_span` - ground span) [m]"""
    span_margin::Float64 = 0.0
    """Fin top height minus folded wingtip height [m]"""
    height_margin::Float64 = 0.0
end
