"""
tether.py — Tension-only elastic tether model for RAWES simulation.

Models a Dyneema SK75 1.9 mm braided tether attached from a fixed ground
anchor to the hub.  Force is zero when slack; elastic + damped when taut.
"""

import numpy as np

from frames import cross3


class TetherModel:
    """
    Tension-only elastic tether.

    Models a Dyneema SK75 1.9 mm braided tether attached from a fixed ground
    anchor to the hub node.  Force is zero when the tether is slack (hub
    closer to anchor than rest_length) and elastic when taut.

    Physical basis — Dyneema SK75 1.9 mm braided
    -----------------------------------------------
    Fibre:      UHMWPE (Dyneema SK75)
    Diameter:   1.9 mm (nominal)
    Area:       A = π × (0.00095 m)² = 2.835 × 10⁻⁶ m²
    Fibre E:    109 GPa  (DSM Dyneema SK75 datasheet)
    Braid eff.: ~0.91 (typical braid efficiency for a tight 12-strand braid)
    EA:         109 × 10⁹ × 2.835 × 10⁻⁶ × 0.91 ≈ 281 kN → use 280 kN
    Break load: ≈ 620 N  (DSM SK75, 2 mm; conservative for 1.9 mm)
    Lin. mass:  ρ_fibre × A × braid_fill = 970 × 2.835e-6 / 0.91 ≈ 3.0 g/m
                (fill factor inverted because braid is looser than solid rod)
                Practical value: ~2.1 g/m (manufacturer data for 2 mm SK75)

    Stiffness at operating length L
    --------------------------------
    k(L) = EA / L          [N/m]  — nonlinear: stiffer for shorter tether

    Structural damping
    ------------------
    Damping uses a constant damping ratio ζ (fraction of critical):
        c_eff(L) = ζ · 2 · √(k_eff · m)  where k_eff = EA / L
    This keeps ζ constant regardless of tether length, correctly scaling
    with the 4× stiffness change between L=200 m and L=50 m.
    UHMWPE material loss tangent: δ ≈ 0.04 → ζ_material ≈ 0.02.
    Default ζ = 0.05 adds a small aerodynamic-drag contribution not
    otherwise captured in the single-body model.
    Damping is applied only while tether is extending (no negative damping).
    At L=50 m, m=5 kg, ζ=0.05:  c_eff = 0.05 × 2 × √(5620×5) = 16.8 N·s/m
    At L=200 m, m=5 kg, ζ=0.05: c_eff = 0.05 × 2 × √(1405×5) =  8.4 N·s/m

    Scenario
    ---------
    rest_length = 200 m : tether paid out to 200 m by the ground winch.
    Hub starts at Z = 50 m ≪ 200 m → slack zone; no tether force at launch.
    Tether activates only if hub flies farther than 200 m from anchor.
    This represents a realistic AWE flight test scenario where the winch has
    pre-deployed 200 m of tether before the hub launches.
    """

    # Dyneema SK75 1.9 mm braided — material constants
    DIAMETER_M       = 0.0019           # nominal diameter [m]
    AREA_M2          = np.pi * (DIAMETER_M / 2) ** 2   # = 2.835e-6 m²
    E_FIBRE_PA       = 109e9            # fibre axial modulus [Pa]
    BRAID_EFFICIENCY = 0.91             # braid efficiency (axial stiffness fraction)
    EA_N             = E_FIBRE_PA * AREA_M2 * BRAID_EFFICIENCY  # ≈ 281 kN [N]
    LINEAR_MASS_KG_M = 0.0021          # linear mass density [kg/m]  (2.1 g/m)
    BREAK_LOAD_N     = 620.0           # conservative breaking load [N]

    def __init__(
        self,
        anchor_ned:            np.ndarray = np.array([0.0, 0.0, 0.0]),
        rest_length:           float      = None,    # unstretched tether length [m] — required; no default
        EA:                    float      = None,    # override axial stiffness [N]
        zeta:                  float      = 0.05,    # damping ratio (fraction of critical)
        hub_mass:              float      = 5.0,     # hub mass [kg] — sets critical damping scale
        axle_attachment_length: float     = 0.3,     # distance from CoM to tether attach point along -body_Z [m]
    ):
        if rest_length is None:
            raise ValueError("TetherModel requires rest_length — no default (mission-specific parameter)")
        self.anchor      = np.asarray(anchor_ned, dtype=float)
        self.rest_length = float(rest_length)
        self.EA          = float(EA) if EA is not None else self.EA_N
        self.zeta        = float(zeta)
        self.hub_mass    = float(hub_mass)
        self.axle_attach = float(axle_attachment_length)
        self._last_info: dict = {}

    @property
    def damping(self) -> float:
        """Effective damping coefficient at rest_length (for logging)."""
        k_ref = self.EA / max(self.rest_length, 1.0)
        return self.zeta * 2.0 * np.sqrt(k_ref * self.hub_mass)

    def compute(
        self,
        hub_pos: np.ndarray,   # hub position in NED world frame [m]
        hub_vel: np.ndarray,   # hub velocity in NED world frame [m/s]
        R_hub:   np.ndarray = None,  # body→world rotation matrix (for restoring torque)
    ) -> tuple:
        """
        Return (force, moment) on hub due to tether, both in NED world frame [N] / [N·m].

        When R_hub is provided and tether is taut, also computes a restoring moment
        M = r_attach × F_tether that aligns the hub axle (body Z) with the tether.
        The attachment point is 'axle_attachment_length' below the CoM along -body Z.

        Stores diagnostic info in self._last_info for telemetry logging.
        """
        r = hub_pos - self.anchor           # vector: anchor → hub
        L = float(np.linalg.norm(r))

        _zero3 = np.zeros(3)
        if L < 1e-6:
            self._last_info = dict(slack=True, tension=0.0, extension=0.0,
                                   length=0.0, k_eff=0.0)
            return _zero3, _zero3

        unit = r / L                        # unit vector anchor → hub
        extension = L - self.rest_length

        if extension <= 0.0:               # tether is slack
            self._last_info = dict(slack=True, tension=0.0, extension=extension,
                                   length=L, k_eff=0.0)
            return _zero3, _zero3

        # Stiffness: k = EA / L  (nonlinear — stiffer when shorter)
        k_eff = self.EA / L

        # Elastic tension
        T_elastic = k_eff * extension

        # Proportional damping: c = zeta * 2 * sqrt(k_eff * m)
        # This maintains a constant damping ratio regardless of tether length,
        # correctly scaling with k_eff which changes 4× between L=200m and L=50m.
        c_eff    = self.zeta * 2.0 * np.sqrt(k_eff * self.hub_mass)
        v_radial = float(np.dot(hub_vel, unit))   # rate of length increase
        T_damp   = c_eff * max(0.0, v_radial)

        T_total  = T_elastic + T_damp

        # Force on hub: directed from hub toward anchor (opposes extension)
        force = -T_total * unit

        # Restoring moment: tether pulls on the axle bottom, not the CoM.
        # r_attach = offset from CoM to tether attachment point in world frame.
        # body Z = R_hub[:,2]; attachment is 'axle_attach' metres below CoM.
        if R_hub is not None:
            body_z_world = R_hub[:, 2]              # disk normal in world frame
            r_attach = -self.axle_attach * body_z_world   # toward axle bottom
            moment = cross3(r_attach, force)
        else:
            moment = _zero3

        self._last_info = dict(
            slack        = False,
            tension      = T_total,
            t_elastic    = T_elastic,
            t_damp       = T_damp,
            extension    = extension,
            length       = L,
            k_eff        = k_eff,
        )
        return force, moment
