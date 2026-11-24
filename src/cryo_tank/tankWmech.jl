"""
Length of the head-morphing window of the inner vessel, as a fraction of the (maximum) inner vessel radius. Over this window of
added cylinder length the heads change continuously from hemispherical (aspect ratio 1) to the design aspect ratio `ARtank`.
"""
const HEAD_MORPH_LEN_FRAC = 1.0

"""Smallest allowed size of the spherical inner vessel, as a fraction of its maximum radius (avoids a degenerate zero-volume tank)."""
const SPHERE_MIN_FRAC = 1e-3

"""
Wall build-up and governing sizing mode of the cylinder and head of a tank wall.
"""
struct WallDiagnostics
      """Number of plies of the cylinder wall (0 for a metal wall)"""
      n_cyl::Int
      """Number of plies of the head wall (0 for a metal wall)"""
      n_head::Int
      """Cylinder wall thickness (m)"""
      t_cyl::Float64
      """Head wall thickness (m)"""
      t_head::Float64
      """Governing mode of the cylinder: inner `:hoop`, `:axial` or `:min_stack`; outer `:buckling` or `:min_stack`; `:alloy` for a metal wall"""
      mode_cyl::Symbol
      """Governing mode of the head: inner `:strength` or `:min_stack`; outer `:buckling` or `:min_stack`; `:alloy` for a metal wall"""
      mode_head::Symbol
end

"""
Geometric and weight outputs of the inner cryogenic tank sizing.
"""
struct InnerTankGeometry{T<:Real}
      """Total tank weight (structural + insulation) (N)"""
      Wtank::T
      """Total insulation weight (N)"""
      Winsul_sum::T
      """Volume of fuel (m^3)"""
      Vfuel::T
      """Insulated surface area of each ellipsoidal-cap interface, length N+1 (m^2)"""
      Shead_insul::Vector{T}
      """Outer radius of the inner vessel (m)"""
      Rtank_outer::T
      """Total longitudinal length of the tank (m)"""
      l_tank::T
      """Length of the cylindrical portion (m)"""
      l_cyl::T
      """Cylinder wall thickness of the inner vessel (m)"""
      t_cyl::T
      """Effective aspect ratio of the heads (-), between 1 (hemispherical) and the design `ARtank`"""
      AR_head::T
      """Wall build-up and governing modes of the inner vessel (see [`WallDiagnostics`](@ref))"""
      wall_diag::WallDiagnostics
end

#Outer constructor that promotes mixed argument types (e.g., ForwardDiff duals and Float64)
function InnerTankGeometry(Wtank, Winsul_sum, Vfuel, Shead_insul::AbstractVector, Rtank_outer, l_tank, l_cyl, t_cyl, AR_head,
                           wall_diag::WallDiagnostics)
      T = promote_type(typeof(Wtank), typeof(Winsul_sum), typeof(Vfuel), eltype(Shead_insul), 
                       typeof(Rtank_outer), typeof(l_tank), typeof(l_cyl), typeof(t_cyl), typeof(AR_head))
      return InnerTankGeometry{T}(T(Wtank), T(Winsul_sum), T(Vfuel), convert(Vector{T}, Shead_insul), 
                                  T(Rtank_outer), T(l_tank), T(l_cyl), T(t_cyl), T(AR_head), wall_diag)
end

"""
      size_inner_tank(fuse::Fuselage, fuse_tank::fuselage_tank, t_cond::Vector{Float64})

`size_inner_tank` calculates the weight of the cryogenic fuel tank for a LH-fueled aircraft.

!!! details "🔃 Inputs and Outputs"
      **Inputs:**
      - `fuse::Fuselage`: fuselage object.
      - `fuse_tank::fuselage_tank`: fuselage tank object.
      - `t_cond::Vector{Float64}`: Vector with tank insulation layer thickness. Provided separately from fuse_tank as it changes during
      non-linear solve process.

      **Outputs:**
      - `::InnerTankGeometry`: inner tank geometry and weights.

The wall is selected from the type of `fuse_tank.inner_material`: a `StructuralAlloy` is sized with the
Barron (1985) pressure-vessel formulas and its stiffener rings use the wall alloy, a `CompositeLaminate` is sized with
the ply-based membrane model of `composite_inner_wall` and its rings use `fuse_tank.stiffener_material`. The
permeation barrier and joint masses (`has_permeation_barrier`, `t_barrier`, `f_joint_add`) apply to composite walls only.

NOTE: Al alloy 2219 has been recommended as tank material (from H2 tank paper in OneNote)

See [here](@ref fueltanks).
"""
function size_inner_tank(fuse::Fuselage, fuse_tank::fuselage_tank, t_cond::AbstractVector{<:Real})
      #---------------------------------
      # Unpack parameters in fuse_tank
      #---------------------------------
      Rfuse = fuse.layout.radius
      fuse_cs = fuse.layout.cross_section #Fuselage cross section

      Wfuel = fuse_tank.Wfuelintank

      ρfuel = fuse_tank.rhofuel
      ρfgas = fuse_tank.rhofuelgas
      ftankadd = fuse_tank.ftankadd
      material_insul = fuse_tank.material_insul
      wall_material = fuse_tank.inner_material
      rhoskin = wall_material.ρ
      ring_material = stiffener_ring_material(wall_material, fuse_tank)
      s_a = ring_material.UTS / 4 #Maximum allowable stress is 1/4 Ultimate tensile strength (Barron 1985, p. 359)
      rhoring = ring_material.ρ
      clearance_fuse = fuse_tank.clearance_fuse
      AR = fuse_tank.ARtank
      ullage_frac = fuse_tank.ullage_frac
      θ = fuse_tank.theta_inner

      #---------------------------------
      # Wall thickness sizing
      #---------------------------------
      thickness_insul = sum(t_cond) #Total insulation thickness
      Rmax_outer = Rfuse - thickness_insul - clearance_fuse #Largest inner vessel outer radius that fits in the fuselage
      Rmax_outer > 0 || throw(DomainError(Rmax_outer, "size_inner_tank: insulation and clearance leave no room for the inner vessel"))

      #The wall thickness is not linear in the radius for a composite wall (whole-stack rounding), so size the wall of the largest vessel
      wall_max = inner_wall(wall_material, fuse_tank, Rmax_outer)
      Rmax_inner = Rmax_outer - wall_max.t_cyl #Largest inner vessel inner radius

      #---------------------------------
      # Required volume
      #---------------------------------
      Wfuel_tot = Wfuel #Wfuel already includes the amount that boils off
      ρfmix = (1 - ullage_frac) * ρfuel + ullage_frac * ρfgas #Density of saturated mixture in tank
      Vfuel = Wfuel_tot / (gee * ρfmix) #Total tank volume taken by saturated mixture
      Vinternal = Vfuel  # required internal volume

      #---------------------------------
      # Continuous tank shape as a function of volume
      #---------------------------------
      #The volume of two end caps of aspect ratio AR on a cross-section of area A and radius R is 2*(2/3)*A*R/AR 
      #(for a standard ellipsoid V = 1/2×(4π/3 ×(abc)) where a,b,c are the semi-axes length. If base is circular,
      #a = R/AR, b=c=R. Since Abase = πR^2, this can also be written as V = 2 * (Abase * R/AR)/3. 
      #
      #The shape grows without discontinuities as the fuel volume increases:
      #  1. V ≤ V_sph: a sphere (two AR = 1 caps, no cylinder), geometrically scaled from zero up to the maximum 
      #     radius that fits in the fuselage (insulation and clearance included).
      #  2. V_sph < V ≤ V_morph: radius is the maximum radius, a cylinder begins to grow, and the aspect ratio of the heads 
      #     moves smoothly (cubic smoothstep) from 1 (hemispherical) to the design value ARtank.
      #  3. V > V_morph: design heads and a cylinder with length l_cyl = (V - V_caps)/A.
      _, Amax = scaled_cross_section(fuse_cs, Rmax_inner)
      AR_design = max(AR, one(AR)) #Heads are never more slender than hemispherical
      V_sph = 4 * Amax * Rmax_inner / 3 #Volume of the largest sphere (two AR = 1 caps)
      V_morph = V_sph + HEAD_MORPH_LEN_FRAC * Rmax_inner * Amax #End of the morphing window

      if Vinternal <= V_sph #Regime 1: sphere
            scale = max(cbrt(Vinternal / V_sph), SPHERE_MIN_FRAC) #Geometric scaling of the largest sphere
            Rtank_outer = Rmax_outer * scale
            AR_head = one(AR_design)
            l_cyl = zero(Rtank_outer)
      else #Regimes 2 and 3
            Rtank_outer = Rmax_outer
            ξ = clamp((Vinternal - V_sph) / (V_morph - V_sph), 0.0, 1.0)
            AR_head = 1 + (AR_design - 1) * ξ^2 * (3 - 2 * ξ) #Smoothstep: zero slope at both ends of the window
            Vcaps = 4 * Amax * Rmax_inner / (3 * AR_head)
            l_cyl = max((Vinternal - Vcaps) / Amax, zero(Vinternal / Amax)) #Required length of cylindrical portion
      end

      wall = inner_wall(wall_material, fuse_tank, Rtank_outer; AR_head) #Cylinder and head wall thicknesses
      tskin = wall.t_cyl
      t_head = wall.t_head
      Rtank = Rtank_outer - tskin #Inner vessel inner radius

      Lhead = Rtank / AR_head       # eg. for a 2:1 ellipsoid majorax/minorax = 2/1 ⟹ R/Lhead = 2/1 

      #---------------------------------
      # Vessel geometry
      #---------------------------------
      perim_tank, Atank = scaled_cross_section(fuse_cs, Rtank) #Tank perimeter and cross-sectional area

      # areas
      Scyl = perim_tank*l_cyl # Surface area of cylindrical part

      Shead = 2*Atank * ( 0.333 + 0.667*(Lhead/Rtank)^1.6 )^0.625 # Surface area of ellipsoidal caps
            #This form is better for insul thickness
            # but just as a note to reader this comes from  semi- oblate spheroid surf area is ≈ 2π×R²[1/3 + 2/3×(1/AR)^1.6]^(1/1.6)
      
            # component volumes    
      Vcyl  = Scyl*tskin    # volume of the metal in the cylindrical part

      Vhead = Shead * t_head # volume of head

      Sinternal = Scyl + 2 * Shead

      #---------------------------------
      # Weight calculation
      #---------------------------------
      Whead = rhoskin*gee*Vhead #Head weight
      Wcyl  = rhoskin*gee*Vcyl #Cylinder weight
      Winnertank = Wcyl + 2*Whead + Wfuel #Weight of inner vessel without stiffeners and supports, inc. fuel

      #stiffeners
      Nmain = 2 #Tanks typically have two main support rings
      Wmainstiff = stiffener_weight("inner", Winnertank / Nmain, Rtank, perim_tank, 
                                s_a, rhoring, θ) #Weight of one main stiffener

      Wstiff = Nmain * Wmainstiff

      #Composite walls only: permeation barrier (mass only; its plies are part of the laminate thickness) and joints
      Wextra = 0.0
      if wall_material isa CompositeLaminate
            W_barrier = fuse_tank.has_permeation_barrier ? rhoskin * fuse_tank.t_barrier * Sinternal * gee : 0.0
            W_joint = fuse_tank.f_joint_add * (Wcyl + 2*Whead)
            Wextra = W_barrier + W_joint
      end

      Wtank = (Wcyl + 2*Whead + Wstiff + Wextra) * (1 + ftankadd)

      # Insulation weight
      N = length(t_cond) #Number of insulation layers
      #Initialize storage vectors
      Winsul_sum = zero(eltype(t_cond)) #Element type follows t_cond so that ForwardDiff duals propagate
      Shead_insul = zeros(eltype(t_cond), N + 1) #add one for first (tank wall) surface 
      L = Lhead + tskin #Length of ellipsoid semi-minor axis

      Ro = Ri = Rtank_outer # Start calculating insulation from the outer wall of the metal tank ∴Ri of insul = outer R of tank
      _, Ao = scaled_cross_section(fuse_cs, Ro) #Cross-sectional area of double bubble
      Shead_insul[1] = 2*Ao * ( 0.333 + 0.667*(L/Ro)^1.6 )^0.625 #Surface area of vessel head
      
      for n in 1:N
            #Update radius and semi-minor axis
            Ro = Ro + t_cond[n]
            L  = L  + t_cond[n]

            _, Ao = scaled_cross_section(fuse_cs, Ro)
            _, Ai = scaled_cross_section(fuse_cs, Ri)
            rho_insul = material_insul[n].ρ
            Vcyl_insul  = l_cyl * (Ao - Ai) #Volume of cylindrical layer
            Shead_insul[n+1] = 2*Ao * ( 0.333 + 0.667*(L/Ro)^1.6 )^0.625 #Surface area of ellipsodal cap

            Area_coeff = Shead_insul[n+1] / Ro^2 #coefficient that relates area and radius squared
            Vhead_insul = ((Shead_insul[n] + Shead_insul[n+1])/2 - Area_coeff/(6) * t_cond[n]^2) * t_cond[n] #Closed-form solution
            
            Winsul_sum += (Vcyl_insul + 2*Vhead_insul) * rho_insul * gee #Weight of insulation layer

            Ri = Ro #Update inner radius for next point
      end

      Wtank = (Wtank + Winsul_sum)
      l_tank = l_cyl + 2*Lhead + 2*thickness_insul + 2*t_head #Total longitudinal length of the tank

      wall_diag = inner_wall_diagnostics(wall_material, wall)
      @debug "Inner tank wall" wall_diag

      return InnerTankGeometry(Wtank, Winsul_sum, Vfuel, Shead_insul, Rtank_outer, l_tank, l_cyl, tskin, AR_head, wall_diag)
end


"""
Geometric and weight outputs of the outer cryogenic tank (vacuum-jacket vessel) sizing.
"""
struct OuterTankGeometry
      """Total weight of outer vessel (N)"""
      Wtank::Float64
      """Weight of cylindrical portion (N)"""
      Wcyl::Float64
      """Weight of one elliptical head (N)"""
      Whead::Float64
      """Total stiffener weight (N)"""
      Wstiff::Float64
      """Total surface area of outer vessel (m^2)"""
      Souter::Float64
      """Surface area of one outer-vessel head (m^2)"""
      Shead::Float64
      """Surface area of cylindrical portion (m^2)"""
      Scyl::Float64
      """Wall thickness of cylindrical portion (m)"""
      t_cyl::Float64
      """Wall thickness of tank head (m)"""
      t_head::Float64
      """Total length of the outer vessel (m)"""
      l_outer::Float64
      """Weight of the low-emissivity facing on the vacuum-facing area, 0 for an alloy wall (N)"""
      Wfacing::Float64
      """Wall build-up and governing modes of the outer vessel (see [`WallDiagnostics`](@ref))"""
      wall_diag::WallDiagnostics
end

"""
    size_outer_tank(fuse::Fuselage, fuse_tank::fuselage_tank, Winnertank::Float64, l_cyl::Float64, Ninterm::Float64)
This function sizes the outer vessel and calculates the weights of its components.
For a `CompositeLaminate` outer wall the buckling uses `E_eq * buckling_knockdown` and `ν_eq`, the thicknesses are rounded up to whole stacks, the rings use `stiffener_material`, and the facing mass is added.

!!! details "🔃 Inputs and Outputs"
    **Inputs:**
    - `fuse::Fuselage`: fuselage object.
    - `fuse_tank::fuselage_tank`: fuselage tank object.
    - `Winnertank::Float64`: weight of inner vessel and contents (N).
    - `l_cyl::Float64`: length of cylindrical portion of outer vessel (m).
    - `Ninterm::Float64`: optimum number of intermediate stiffener rings.
    - `Router::Float64`: radius of the outer vessel (m). 
    - `AR_head::Float64`: aspect ratio of the outer vessel heads.

    **Outputs:**
    - `::OuterTankGeometry`: outer tank geometry and weights.
"""
function size_outer_tank(fuse::Fuselage, fuse_tank::fuselage_tank, Winnertank::Float64, l_cyl::Float64, Ninterm::Float64;
      Router::Float64 = fuse.layout.radius - fuse_tank.clearance_fuse, AR_head::Float64 = fuse_tank.ARtank)
      #---------------------------------
      # Unpack parameters in fuse_tank
      #---------------------------------
      Rfuse = fuse.layout.radius
      fuse_cs = fuse.layout.cross_section #Fuselage cross section

      ftankadd = fuse_tank.ftankadd
      ARtank = AR_head
      θ_outer = fuse_tank.theta_outer

      #---------------------------------
      # Size outer vessel
      #---------------------------------
      θ1 = θ_outer[1]
      θ2 = θ_outer[2]

      Nmain = 2 #Tanks typically have two main support rings
      # This is per ASME code in 1985. The required factor of safety may be substantially higher than aerospace requirements.
      pc = 4 * pref #4*p_atm; Collapsing pressure, Eq. (7.11) in Barron (1985)
      ring_mat = stiffener_ring_material(fuse_tank.outer_material, fuse_tank) #Material of the stiffener rings
      s_a = ring_mat.UTS / 4 #Maximum allowable stress is approx. 1/4 Ultimate tensile strength (Barron 1985, p. 359)

      #Calculate outer vessel geometry
      Rtank_outer = Router
      Do = 2 * Rtank_outer #outside diameter

      Nstiff = Nmain + Ninterm #Total number of stiffeners
      L = l_cyl / (Nstiff - 1) #There are two stiffeners at the ends, so effective number of sections in skin is N - 1
      L_Do = L / Do

      #---------------------------------
      # Wall thicknesses
      #---------------------------------

      E_wall, ν_wall = outer_buckling_properties(fuse_tank.outer_material, fuse_tank) #Modulus and Poisson ratio for buckling

      #Find cylinder wall thickness. This applies to a short cylinder.
      #If the vessel has no cylindrical portion (spherical inner vessel) there is no cylinder to size; this is also the limit of
      #the buckling relation as L/D → 0, so the weight is continuous.
      t_Do = l_cyl > 0 ? outer_cylinder_thickness_ratio(E_wall, ν_wall, L_Do, pc) : 0.0 #Bracketed root of the buckling equation
      t_cyl_req = t_Do * Do

      #Find head wall thickness
      K1 = find_K1_head(ARtank) #Find coefficient K1
      t_head_req = K1 * Do * sqrt(pc * sqrt(3*(1 - ν_wall^2))/ (0.5*E_wall))

      wall_diag = outer_wall_diagnostics(fuse_tank.outer_material, fuse_tank, t_cyl_req, t_head_req)
      @debug "Outer tank wall" wall_diag
      t_cyl = wall_diag.t_cyl
      t_head = wall_diag.t_head

      #---------------------------------
      # Vessel geometry
      #---------------------------------
      perim_vessel, Avessel = scaled_cross_section(fuse_cs, Rtank_outer) #Tank perimeter and cross-sectional area

      Shead = 2*Avessel*(0.333 + 0.667*(1/ARtank)^1.6 )^0.625 #ellipsoid surface area
      Scyl  = perim_vessel*l_cyl  # surface area

      Souter = Scyl + 2*Shead

      ## Volume and Weight
      Vcyl  = Scyl*t_cyl
      Vhead = Shead*t_head

      Wcyl  = Vcyl*fuse_tank.outer_material.ρ*gee
      Whead =  Vhead*fuse_tank.outer_material.ρ*gee

      Wtank_no_stiff = Wcyl + 2 * Whead

      #---------------------------------
      # Size stiffeners
      #---------------------------------
      tanktype = "outer"

      Wmainstiff = stiffener_weight(tanktype, Winnertank / Nmain, Rtank_outer, perim_vessel, #Weight of one main stiffener, each one 
                                    s_a, ring_mat.ρ, θ1, θ2, Nstiff, l_cyl, ring_mat.E)  #carries half the inner vessel load
                                                                              
      Wintermstiff = stiffener_weight(tanktype, 0.0, Rtank_outer, perim_vessel, 
                                    s_a, ring_mat.ρ, θ1, θ2, Nstiff, l_cyl, ring_mat.E) #Weight of one intermediate stiffener, which carries no load

      #---------------------------------
      # Weight estimation
      #---------------------------------
      Wstiff = Nmain * Wmainstiff + Ninterm * Wintermstiff #Total stiffener weight

      Wfacing = outer_facing_weight(fuse_tank.outer_material, Souter) #Low-emissivity facing on the vacuum-facing area

      Wtank = (Wtank_no_stiff + Wstiff) * (1 + ftankadd) + Wfacing #Find total tank weight, including additional mass factor and facing

      l_outer = l_cyl + Do / ARtank + 2*t_head #Total length of outer vessel

      return OuterTankGeometry(Wtank, Wcyl, Whead, Wstiff, Souter, Shead, Scyl, t_cyl, t_head, l_outer, Wfacing, wall_diag)
end

"""
      outer_buckling_properties(mat, fuse_tank::fuselage_tank)

Returns the Young's modulus and Poisson ratio used in the outer-vessel buckling formulas. For a `StructuralAlloy` these are
its own values. For a `CompositeLaminate` they are the equivalent-isotropic `E_eq * fuse_tank.buckling_knockdown` and `ν_eq`.
"""
outer_buckling_properties(mat::StructuralAlloy, fuse_tank::fuselage_tank) = (mat.E, mat.ν)
outer_buckling_properties(mat::CompositeLaminate, fuse_tank::fuselage_tank) = (mat.E_eq * fuse_tank.buckling_knockdown, mat.ν_eq)

"""
      round_outer_wall(mat, fuse_tank::fuselage_tank, t_req::Float64)

Rounds a required outer-wall thickness to the wall build-up. A `StructuralAlloy` is returned unchanged. A `CompositeLaminate`
is rounded up to whole stacks of `mat.plies_per_stack` plies, with a minimum of `max(1, fuse_tank.composite_min_stacks)` stacks.
"""
round_outer_wall(mat::StructuralAlloy, fuse_tank::fuselage_tank, t_req::Float64) = t_req

function round_outer_wall(mat::CompositeLaminate, fuse_tank::fuselage_tank, t_req::Float64)
      n_stack = mat.plies_per_stack
      stacks_floor = max(1, fuse_tank.composite_min_stacks)
      n_stacks = max(stacks_floor, required_stacks(mat, t_req))
      return n_stacks * n_stack * mat.t_ply
end

"""
      required_stacks(mat::CompositeLaminate, t_req::Real)

Number of whole stacks of `mat.plies_per_stack` plies needed for a strength requirement of thickness `t_req` (m),
before the minimum-stack floor is applied.
"""
required_stacks(mat::CompositeLaminate, t_req::Real) =
      ceil(Int, ForwardDiff.value(t_req) / (mat.plies_per_stack * mat.t_ply) - 1e-9)

"""
      outer_wall_diagnostics(mat, fuse_tank::fuselage_tank, t_cyl_req::Float64, t_head_req::Float64)

Rounds the required outer-wall thicknesses with `round_outer_wall` and reports the ply counts and governing modes.
A `StructuralAlloy` is returned unchanged with modes `:alloy`. A `CompositeLaminate` wall is `:min_stack` where the
minimum-stack floor sets its ply count, and `:buckling` otherwise.

!!! details "🔃 Inputs and Outputs"
      **Inputs:**
      - `mat`: outer-vessel wall material.
      - `fuse_tank::fuselage_tank`: fuselage tank object.
      - `t_cyl_req::Float64`: required cylinder wall thickness (m).
      - `t_head_req::Float64`: required head wall thickness (m).

      **Outputs:**
      - `::WallDiagnostics`: rounded wall thicknesses, ply counts and modes.
"""
function outer_wall_diagnostics(mat::StructuralAlloy, fuse_tank::fuselage_tank, t_cyl_req::Float64, t_head_req::Float64)
      t_cyl = round_outer_wall(mat, fuse_tank, t_cyl_req)
      t_head = round_outer_wall(mat, fuse_tank, t_head_req)
      return WallDiagnostics(0, 0, t_cyl, t_head, :alloy, :alloy)
end

function outer_wall_diagnostics(mat::CompositeLaminate, fuse_tank::fuselage_tank, t_cyl_req::Float64, t_head_req::Float64)
      stacks_floor = max(1, fuse_tank.composite_min_stacks)
      t_cyl = round_outer_wall(mat, fuse_tank, t_cyl_req)
      t_head = round_outer_wall(mat, fuse_tank, t_head_req)
      mode_cyl = stacks_floor >= required_stacks(mat, t_cyl_req) ? :min_stack : :buckling
      mode_head = stacks_floor >= required_stacks(mat, t_head_req) ? :min_stack : :buckling
      return WallDiagnostics(round(Int, t_cyl / mat.t_ply), round(Int, t_head / mat.t_ply), t_cyl, t_head, mode_cyl, mode_head)
end

"""
      outer_facing_weight(mat, Souter::Float64)

Weight (N) of the low-emissivity facing on the vacuum-facing outer-vessel area: zero for a `StructuralAlloy`, and
`facing_areal_mass * Souter * gee` for a `CompositeLaminate`.
"""
outer_facing_weight(mat::StructuralAlloy, Souter::Float64) = 0.0
outer_facing_weight(mat::CompositeLaminate, Souter::Float64) = mat.facing_areal_mass * Souter * gee

"""
      check_outer_wall_emissivity(mat)

Warns if a bare composite outer wall (emissivity above 0.1) faces a vacuum gap, since it then radiates far more heat than
a low-emissivity facing. Use a `-VDA` laminate. Does nothing for a `StructuralAlloy`.
"""
function check_outer_wall_emissivity(mat::CompositeLaminate)
      if mat.ε_emis > 0.1
            @warn "Outer composite wall '$(mat.name)' has emissivity $(mat.ε_emis) and faces a vacuum gap: a bare CFRP wall radiates far more heat than a low-emissivity facing. Use a '-VDA' laminate."
      end
      return nothing
end
check_outer_wall_emissivity(mat::StructuralAlloy) = nothing

"""
      stiffener_weight(tanktype::String, W::Float64, Rtank::Float64, perim::Float64, s_a::Float64, 
      ρstiff::Float64, θ1::Float64, θ2::Float64 = 0.0, Nstiff::Float64 = 2.0, l_cyl::F
This function calculates the weight of a single stiffener in an inner or outer vessel for a given inner vessel weight.

!!! details "🔃 Inputs and Outputs"
    **Inputs:**
    - `tanktype::String`: type of tank, options are "inner" or "outer".
    - `W::Float64`: load carried by a stiffener ring (N).
    - `Rtank::Float64`: tank radius (m).
    - `perim::Float64`: tank perimeter (m).
    - `s_a::Float64`: maximum allowable stress in stiffener material (Pa).
    - `ρstiff::Float64`: stiffener density (kg/m^3).
    - `θ1::Float64`: angular position of bottom tank supports, measured from the bottom of the tank (rad).
    - `θ2::Float64`: angular position of top tank supports, measured from the bottom of the tank (rad). Only used with "outer" tank.
    - `Nstiff::Float64`: total number of stiffeners on outer vessel. Only used with "outer" tank.
    - `l_cyl::Float64`: length of cylindrical portion of tank (m). Only used with "outer" tank.
    - `E::Float64`: Young's modulus of stiffener material (Pa). Only used with "outer" tank.

    **Outputs:**
    - `Wstiff::Float64`: weight of a stiffener ring (N).
"""
function stiffener_weight(tanktype::String, W::Real, Rtank::Real, perim::Real, s_a::Float64, 
      ρstiff::Float64, θ1::Float64, θ2::Float64 = 0.0, Nstiff::Float64 = 2.0, l_cyl::Float64 = 0.0, E::Float64 = 0.0)
    
      if tanktype == "inner" 
            _, kmax = stiffeners_bendingM(θ1) #Find k = 2πM/(WR)
            Icollapse = 0.0 #inner vessel cannot collapse as it is pressurized

      elseif tanktype == "outer"
            _, kmax = stiffeners_bendingM_outer(θ1, θ2) #Find k = 2πM/(WR)
            # This is per ASME code in 1985. The required factor of safety may be substantially higher than aerospace requirements.
            pc = 4 * pref #Critical pressure is 4 times atmospheric pressure, Eq. (7.11) in Barron (1985)
            Do = 2 * Rtank #outer diameter

            L = l_cyl/ (Nstiff - 1) #Length of portion between supports
            Icollapse = pc * Do^3 * L / (24 * E) #Second moment of area needed to avoid collapse
      end

      Mmax = kmax * W * Rtank / (2π) #Maximum bending moment due to loads
      Z = Mmax / s_a #required section modulus to withstand bending loads

      #Assume sectional properties of a 100 x 100 I-beam
      W = 100e-3 #Flange width
      t_w = 7.1e-3 #Web thickness
      t_f = 8.8e-3 #Flange thickness

      #For an I-beam, I > W * H^2 * t_f / 2 + t_f^3 * W / 6
      #The required second moment of area is I = Icollapse + Z * (H/2 + t_f/2)
      #Find beam height by solving W * H^2 * t_f / 2 + t_f^3 * W / 6 = Icollapse + Z * (H/2 + t_f/2)

      a = t_f * W / 2 #Coefficients in quadratic equation
      b = -Z/2
      c = -1 * Icollapse - Z * t_f/2 + t_f^3 * W / 6

      #The discriminant is Z^2/4 + 4a(Icollapse + Z t_f/2 - t_f^3 W/6). The last term is a constant set by the fixed flange
      #geometry, so for a lightly loaded ring (small Z and Icollapse) the flanges alone already exceed the requirement,
      #the quadratic has no real root, and the beam only needs its minimum physical height. Bound the discriminant at zero
      #and the height at two flange thicknesses (flanges touching) rather than taking the square root of a negative number.
      disc = max(b^2 - 4 * a * c, zero(b^2 - 4 * a * c))
      H = max((-b + sqrt(disc)) / (2 * a), 2 * t_f) #Solve quadratic eq.
      S = 2 * W * t_f + (H - t_f) * t_w #Beam cross-sectional area

      Wstiff = gee * ρstiff * S * perim #Weight of a single stiffener running along perimeter
      return Wstiff
end

"""
    stiffeners_bendingM(θ)
This function can be used to calculate the bending moment distribution in a stiffener ring for an inner cryogenic tank.
It applies Eqs. (7.4) and (7.5) in Barron (1985) to find the bending moment distribution. The function returns the
maximum value of ``k = 2πM/(WR)`` on the ring's circumference.

Note: An attempt was made to use the NLopt.jl package to find the maximum value of k, 
but this was more expensive and less robust than the brute-force approach.

!!! details "🔃 Inputs and Outputs"
    **Inputs:**
    - `θ::Float64`: angular position of tank supports, measured from the bottom of the tank (rad).

    **Outputs:**
    - `ϕmax::Float64`: angular position of maximum bending moment on ring circumference (rad).
    - `kmax::Float64`: Maximum value of the ratio ``k = 2πM/(WR)`` on ring circumference.
"""
function stiffeners_bendingM(θ::Float64)
      ϕlist = [θ; LinRange(1.1,π/2,41);pi] #Take advantage of known form of maximum; extends to π/2, where the maximum lies for θ ≈ 45 deg
      k = zeros(length(ϕlist))

      for (i,ϕ) in enumerate(ϕlist)
            if 0 ≤ ϕ < θ
                  k[i] = 0.5*cos(ϕ) + ϕ*sin(ϕ) - (π - θ)*sin(θ) + cos(θ) + cos(ϕ)*(sin(θ)^2) # Barron 1985 eq. 7.4
            elseif θ ≤ ϕ ≤ π
                  k[i] = 0.5*cos(ϕ) - (π - ϕ)*sin(ϕ) + θ*sin(θ)  + cos(θ) + cos(ϕ)*(sin(θ)^2) # Barron 1985 eq. 7.5
            end
      end
      kmax, imax = findmax(abs.(k))
      ϕmax = ϕlist[imax]
      return ϕmax, kmax
end

"""
    stiffeners_bendingM_outer(θ1,θ2)
This function can be used to calculate the bending moment distribution in a stiffener ring for an outer cryogenic tank.
It applies Eqs. (7.13)-(7.15) in Barron (1985) to find the bending moment distribution. The function returns the
maximum value of ``k = 2πM/(WR)`` on the ring's circumference.

!!! details "🔃 Inputs and Outputs"
    **Inputs:**
    - `θ1::Float64`: angular position of bottom tank supports, measured from the bottom of the tank (rad).
    - `θ2::Float64`: angular position of top tank supports, measured from the bottom of the tank (rad).

    **Outputs:**
    - `ϕmax::Float64`: angular position of maximum bending moment on ring circumference (rad).
    - `kmax::Float64`: Maximum value of the ratio ``k = 2πM/(WR)`` on ring circumference.
"""
function stiffeners_bendingM_outer(θ1::Float64, θ2::Float64)
      ϕlist = LinRange(0.0, π, 361)
      k = zeros(length(ϕlist))

      for (i,ϕ) in enumerate(ϕlist)
            if 0 ≤ ϕ ≤ θ1
                  k[i] = cos(ϕ)*(sin(θ2)^2 - sin(θ1)^2 ) + (cos(θ2) - cos(θ1) ) +
                              - (π-θ2)*sin(θ2) +  (π-θ1)*sin(θ1)
            elseif θ1 ≤ ϕ ≤ θ2
                  k[i] = cos(ϕ)*(sin(θ2)^2 - sin(θ1)^2 ) + (cos(θ2) - cos(θ1) ) +
                              - (π-θ2)*sin(θ2) +  π*sin(ϕ) -θ1*sin(θ1)
            elseif θ2 ≤ ϕ ≤ π
                  k[i] = cos(ϕ)*(sin(θ2)^2 - sin(θ1)^2 ) + (cos(θ2) - cos(θ1) )  +
                              (θ2*sin(θ2) - θ1*sin(θ1)  )
            end
      end
      kmax, imax = findmax(abs.(k))
      ϕmax = ϕlist[imax]
      return ϕmax, kmax
end


"""
    optimize_outer_tank(fuse::Fuselage, fuse_tank::fuselage_tank, Winnertank, l_cyl, θ1, θ2)
This function optimizes the number of intermediate stiffener rings to minimize the weight of an outer vessel.

!!! details "🔃 Inputs and Outputs"
    **Inputs:**
    - `fuse::Fuselage`: fuselage object.
    - `fuse_tank::fuselage_tank`: fuselage tank object.
    - `Winnertank::Float64`: weight of inner vessel and contents (N).
    - `l_cyl::Float64`: length of cylindrical portion of outer vessel (m).

    **Outputs:**
    - `Ninterm::Float64`: optimum number of intermediate stiffener rings.
"""
function optimize_outer_tank(fuse::Fuselage, fuse_tank::fuselage_tank, Winnertank::Float64, l_cyl::Float64;
      Router::Float64 = fuse.layout.radius - fuse_tank.clearance_fuse, AR_head::Float64 = fuse_tank.ARtank)

      obj(x, grad) = size_outer_tank(fuse, fuse_tank, Winnertank, l_cyl, x[1]; Router, AR_head).Wtank #Minimize Wtank

      #Set bounds
      lower = 0.0
      upper = 50.0

      #Start strictly inside the bounds: a Nelder-Mead simplex started on the lower bound (e.g. from a previous 
      #solution of Ninterm = 0) can collapse there and never explore larger values.
      initial_x = [clamp(fuse_tank.Ninterm, lower .+ 0.5, upper .- 0.5)]
      
      #Use NLopt.jl to minimize function 
      opt = Opt(:LN_NELDERMEAD, length(initial_x))
      opt.lower_bounds = lower
      opt.upper_bounds = upper
      opt.ftol_rel = 1e-9
      opt.maxeval = 100  # Set the maximum number of function evaluations

      opt.min_objective = obj

      (minf,xopt,ret) = NLopt.optimize(opt, initial_x)
      Ninterm = xopt[1]
      return Ninterm
end

"""
      outer_cylinder_thickness_ratio(E, ν, L_Do, pc)

Wall thickness to diameter ratio `t/D` of a short outer-vessel cylinder that buckles at the collapsing pressure `pc`,
from the short-cylinder form `pc = 2.42 E (t/D)^(5/2) / ((1 - ν²)^(3/4) (L/D - 0.45 sqrt(t/D)))` (Barron 1985).

The formula is only valid for `0 ≤ t/D < (L/D / 0.45)²`: the residual is `-pc` at `t/D = 0`, increases monotonically,
and diverges at the upper limit, so a unique root always lies in that interval. The root is therefore found by
bisection on a bracket inside it, so the iteration can never leave the valid domain (a secant step to `t/D < 0` would
otherwise raise a `DomainError` in `(t/D)^(5/2)`).

!!! details "🔃 Inputs and Outputs"
      **Inputs:**
      - `E::Real`: Young's modulus used for buckling (Pa).
      - `ν::Real`: Poisson ratio used for buckling.
      - `L_Do::Real`: stiffener spacing to outer diameter ratio.
      - `pc::Real`: collapsing pressure (Pa).

      **Outputs:**
      - `t_Do::Real`: wall thickness to diameter ratio, strictly positive.
"""
function outer_cylinder_thickness_ratio(E::Real, ν::Real, L_Do::Real, pc::Real)
      (L_Do > 0 && isfinite(L_Do)) ||
            throw(DomainError(L_Do, "outer vessel stiffener spacing to diameter ratio must be positive and finite"))

      residual(t_D) = 2.42*E*t_D^(5/2) / ( (1 - ν^2)^(3/4) * (L_Do - 0.45*sqrt(t_D)) ) - pc

      t_pole = (L_Do / 0.45)^2 #Upper limit of validity, where the denominator vanishes
      #Upper end of the bracket: approach the pole until the residual is positive (it always is close enough to the pole)
      t_hi = NaN
      for ε in (1e-3, 1e-6, 1e-9, 1e-12)
            t_try = t_pole * (1 - ε)
            if residual(t_try) > 0
                  t_hi = t_try
                  break
            end
      end
      isnan(t_hi) && throw(DomainError(L_Do, "no buckling thickness found: outer vessel stiffener spacing to diameter ratio is too small"))

      return find_zero(residual, (zero(t_hi), t_hi), Bisection())
end

"""
      find_K1_head(AR)

This function calculates the equivalent radius for hemi-ellipsoidal heads under pressure.
      
!!! details "🔃 Inputs and Outputs"
      **Inputs:**
      - `AR::Float64`: head aspect ratio.

      **Outputs:**
      - `K1::Float64`: equivalent radius parameter, such that `R = K1 * D``, where `R` is 
      the equivalent radius and `D` is the ellipse major diameter.
"""
function find_K1_head(AR)
      # See table 7.6 for data in Barron (1985) p. 367
      ARs = [1.0, 1.2, 1.4, 1.6, 1.8, 2.0, 2.2, 2.4, 2.6, 2.8, 3.0]
      K1s = [0.50,0.57,0.65,0.73,0.81,0.9,0.99,1.08,1.18,1.27,1.36]
      if (AR > maximum(ARs)) || (AR < minimum(ARs)) #If case is outside of range
            println("ARtank of heads not supported, see size_outer_tank()")
            K1 = 1.0
      else
            i = 1
            while AR > ARs[i] #Find index of entry that is not smaller than AR
                  i += 1
            end
            if AR ≈ ARs[i] #If it is exactly that entry
                  K1 = K1s[i]
            else #Otherwise interpolate linearly
                  K1 = K1s[i-1] + (K1s[i] - K1s[i-1]) / (ARs[i] - ARs[i-1]) * (AR - ARs[i-1])
            end
      end
      return K1
end




"""
      inner_wall(mat::StructuralAlloy, fuse_tank::fuselage_tank, Rtank_outer::Real; AR_head::Real = fuse_tank.ARtank)

`inner_wall` sizes the cylinder and head wall thicknesses of a metal inner tank with the Barron (1985) pressure-vessel
formulas, (7.1) and (7.2), at an allowable stress of 1/4 of the ultimate tensile strength and the weld efficiency `fuse_tank.ew`.
For a `CompositeLaminate` wall it is `composite_inner_wall`.

!!! details "🔃 Inputs and Outputs"
      **Inputs:**
      - `mat::StructuralAlloy`: inner tank wall alloy.
      - `fuse_tank::fuselage_tank`: fuselage tank object.
      - `Rtank_outer::Real`: outer radius of the inner vessel (m).
      - `AR_head::Real`: aspect ratio of the heads (-), defaults to `fuse_tank.ARtank`.

      **Outputs:**
      - `t_cyl::Real`: cylinder wall thickness (m).
      - `t_head::Real`: head wall thickness (m).
"""
function inner_wall(mat::StructuralAlloy, fuse_tank::fuselage_tank, Rtank_outer::Real; AR_head::Real = fuse_tank.ARtank)
      Δp = fuse_tank.pvent
      AR = AR_head
      weld_eff = fuse_tank.ew
      s_a = mat.UTS / 4 #Maximum allowable stress is approx. 1/4 Ultimate tensile strength (Barron 1985, p. 359)

      #Cylindrical portion wall thickness
      tskin = Δp * (2 * Rtank_outer) / (2 * s_a * weld_eff + 0.8 * Δp) #(7.1) in Barron (1985)

      #Ellipsoid wall thickness
      K = (1/6) * (AR^2 + 2) # Barron pg 359
      t_head = Δp* Rtank_outer * K/ (s_a * weld_eff + Δp * (K - 0.1)) #(7.2) in Barron (1985)

      return (t_cyl = tskin, t_head = t_head)
end

inner_wall(mat::CompositeLaminate, fuse_tank::fuselage_tank, Rtank_outer::Real; AR_head::Real = fuse_tank.ARtank) =
      composite_inner_wall(mat, fuse_tank, Rtank_outer; AR_head)

"""
      inner_wall_diagnostics(mat, wall)

Wall build-up and governing modes of an inner-vessel wall returned by `inner_wall`. A `StructuralAlloy` wall has no plies
and both modes are `:alloy`. The thicknesses are stored as plain `Float64` (`ForwardDiff.value` of any dual).
"""
inner_wall_diagnostics(mat::StructuralAlloy, wall) =
      WallDiagnostics(0, 0, ForwardDiff.value(wall.t_cyl), ForwardDiff.value(wall.t_head), :alloy, :alloy)
inner_wall_diagnostics(mat::CompositeLaminate, wall) =
      WallDiagnostics(wall.n_cyl, wall.n_head, wall.t_cyl, wall.t_head, wall.mode_cyl, wall.mode_head)

"""
      stiffener_ring_material(wall_material, fuse_tank::fuselage_tank)

`stiffener_ring_material` returns the material of the main stiffener rings of a tank: the wall alloy for a
`StructuralAlloy` wall (`fuse_tank.stiffener_material` is ignored), `fuse_tank.stiffener_material` for a
`CompositeLaminate` wall.

!!! details "🔃 Inputs and Outputs"
      **Inputs:**
      - `wall_material::Union{StructuralAlloy, CompositeLaminate}`: inner tank wall material.
      - `fuse_tank::fuselage_tank`: fuselage tank object.

      **Outputs:**
      - `::StructuralAlloy`: ring material.
"""
stiffener_ring_material(wall_material::StructuralAlloy, fuse_tank::fuselage_tank) = wall_material
stiffener_ring_material(wall_material::CompositeLaminate, fuse_tank::fuselage_tank) = fuse_tank.stiffener_material

"""
      composite_strain_allowable(mat::CompositeLaminate, fuse_tank::fuselage_tank)

Design tensile strain allowable of a composite tank wall before the cryogenic and process knockdowns.

Returns `fuse_tank.design_strain_limit` if positive. Otherwise returns the value for `fuse_tank.fueltype`
from the `composite_strain_allowable_by_fuel` table of `MaterialProperties.toml` (case-insensitive).
Throws an `ArgumentError` if the fuel is not in the table; there is no fallback.

!!! details "🔃 Inputs and Outputs"
      **Inputs:**
      - `mat::CompositeLaminate`: tank wall laminate.
      - `fuse_tank::fuselage_tank`: fuselage tank object.

      **Outputs:**
      - `::Float64`: design strain allowable (-).
"""
function composite_strain_allowable(mat::CompositeLaminate, fuse_tank::fuselage_tank)::Float64
      fuse_tank.design_strain_limit > 0 && return fuse_tank.design_strain_limit
      table = get(MaterialProperties, "composite_strain_allowable_by_fuel", Dict{String,Any}())
      key = uppercase(fuse_tank.fueltype)
      haskey(table, key) || throw(ArgumentError("No composite strain allowable for fuel '$(fuse_tank.fueltype)' ($(mat.name)). Add the fuel to the 'composite_strain_allowable_by_fuel' table in MaterialProperties.toml or set design_strain_limit. Valid fuels: $(join(sort(collect(keys(table))), ", "))."))
      return Float64(table[key])
end

"""
      composite_inner_wall(mat::CompositeLaminate, fuse_tank::fuselage_tank, Rtank_outer::Real; AR_head::Real = fuse_tank.ARtank)

`composite_inner_wall` sizes the wall of a composite inner tank from membrane loads and a strain allowable,
and rounds it up to a whole number of laminate stacks.

The barrel carries the hoop load `N_θ = Δp R` on `E2` and the axial load `N_x = Δp R / 2` on `E1`; the thicker of
the two governs. The domes carry the membrane load `Δp R K`, with `K = (AR² + 2)/6`, on the weaker of `E1` and `E2`,
reduced by `fuse_tank.dome_layup_efficiency`. The strain allowable is `composite_strain_allowable(mat, fuse_tank)`, times the cryogenic and process knockdowns of the material. No weld efficiency is applied.

The wall is built from whole stacks of `mat` (`mat.plies_per_stack` plies each): the number of stacks is
`ceil(t_req / (n_stack * mat.t_ply) - 1e-9)`, with `n_stack = mat.plies_per_stack`, and a minimum of
`max(1, fuse_tank.composite_min_stacks)` stacks.

!!! details "🔃 Inputs and Outputs"
      **Inputs:**
      - `mat::CompositeLaminate`: inner tank wall laminate.
      - `fuse_tank::fuselage_tank`: fuselage tank object.
      - `Rtank_outer::Real`: outer radius of the inner vessel (m).
      - `AR_head::Real`: aspect ratio of the domes (-), defaults to `fuse_tank.ARtank`.

      **Outputs:**
      - `t_cyl::Float64`: barrel wall thickness, `n_cyl * mat.t_ply` (m).
      - `t_head::Float64`: dome wall thickness, `n_head * mat.t_ply` (m).
      - `n_cyl::Int`, `n_head::Int`: number of plies of the barrel and dome (multiples of `mat.plies_per_stack`).
      - `mode::Symbol`: `:hoop` or `:axial`, the barrel load direction that governs before rounding.
      - `mode_cyl::Symbol`: `:min_stack` if the minimum sets the barrel ply count, otherwise `mode`.
      - `mode_head::Symbol`: `:min_stack` if the minimum sets the dome ply count, otherwise `:strength`.
      - `floor_governs::Bool`: true if the strength requirement of the barrel or dome is no more than the minimum (`max(1, fuse_tank.composite_min_stacks)` stacks), so the minimum sets its ply count.
"""
function composite_inner_wall(mat::CompositeLaminate, fuse_tank::fuselage_tank, Rtank_outer::Real; AR_head::Real = fuse_tank.ARtank)
      Δp = fuse_tank.pvent
      AR = AR_head
      η = fuse_tank.eta_composite
      epsa = composite_strain_allowable(mat, fuse_tank) * mat.knockdown_cryo * mat.knockdown_process #Design strain allowable

      #Barrel
      t_hoop  = Δp * 2 * Rtank_outer / (2*mat.E2 * epsa * η + 0.8*Δp) # Barron 1985 eq. 7.1
      t_axial = Δp * Rtank_outer / 2 / (mat.E1 * epsa * η) # axial stress from internal pressure only
      mode = t_hoop >= t_axial ? :hoop : :axial
      t_cyl_req = max(t_hoop, t_axial)

      #Domes
      K = (AR^2 + 2) / 6 # Barron 1985 eq. 7.3
      t_head_req = Δp * Rtank_outer * K / (min(mat.E1, mat.E2) * epsa * η * fuse_tank.dome_layup_efficiency + Δp * (K - 0.1)) # Barron 1985 eq. 7.2

      #Round up to whole stacks, with a minimum of `composite_min_stacks` (at least one) stacks
      n_stack = mat.plies_per_stack
      stacks_floor = max(1, fuse_tank.composite_min_stacks)
      stacks_cyl_strength  = required_stacks(mat, t_cyl_req)
      stacks_head_strength = required_stacks(mat, t_head_req)
      n_cyl  = n_stack * max(stacks_cyl_strength,  stacks_floor)
      n_head = n_stack * max(stacks_head_strength, stacks_floor)
      floor_governs = stacks_floor >= stacks_cyl_strength || stacks_floor >= stacks_head_strength #strength needs no more than the minimum
      mode_cyl = stacks_floor >= stacks_cyl_strength ? :min_stack : mode
      mode_head = stacks_floor >= stacks_head_strength ? :min_stack : :strength

      return (t_cyl = n_cyl * mat.t_ply, t_head = n_head * mat.t_ply, n_cyl = n_cyl, n_head = n_head,
              mode = mode, mode_cyl = mode_cyl, mode_head = mode_head, floor_governs = floor_governs)
end
