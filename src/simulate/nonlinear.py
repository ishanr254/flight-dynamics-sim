"""Six-degree-of-freedom nonlinear simulation built on an aircraft file.

Model and assumptions: docs/decisions/2026-10-11-nonlinear-flight-model.md. Body axes are
the trim stability axes (x forward, y starboard, z down); the earth frame is
north-east-down. Linearizing this model about trim returns the linear models in
src/dynamics.

State vector (13): north, east, down, u, v, w, q0, q1, q2, q3, p, q, r.
Control vector (4): thrust [N], delta_m, delta_l, delta_n [rad].
"""

import math
from dataclasses import dataclass, replace

import numpy as np

CONTROLS = ("thrust", "delta_m", "delta_l", "delta_n")


class ModelError(ValueError):
    pass


def quaternion_from_euler(phi, theta, psi):
    cr, sr = math.cos(phi / 2), math.sin(phi / 2)
    cp, sp = math.cos(theta / 2), math.sin(theta / 2)
    cy, sy = math.cos(psi / 2), math.sin(psi / 2)
    return (cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy)


def euler_from_quaternion(q0, q1, q2, q3):
    """(phi, theta, psi): bank, pitch, heading."""
    phi = math.atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2))
    theta = math.asin(max(-1.0, min(1.0, 2 * (q0 * q2 - q3 * q1))))
    psi = math.atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3))
    return phi, theta, psi


@dataclass(frozen=True)
class Envelope:
    """Range in which the linear aerodynamics is taken as valid; outside it is flagged."""

    alpha_min: float = math.radians(-8.0)
    alpha_max: float = math.radians(12.0)
    beta_max: float = math.radians(15.0)


class NonlinearAircraft:
    def __init__(self, aircraft, condition, thrust_max=math.inf,
                 deflection_max=math.radians(30.0), envelope=Envelope(), performance=None):
        """performance: optional src.performance.Performance. When given, its drag polar
        replaces the one reconstructed from the aircraft file and lift is capped at its
        CL_max; the model then no longer linearizes exactly to the linear phugoid."""
        ac, fc = aircraft, condition
        if ac.lateral is None or ac.longitudinal is None:
            raise ModelError(f"{ac.name}: the nonlinear model needs both [lateral] and "
                             "[longitudinal] sections in the aircraft file")
        self.aircraft, self.condition = ac, fc
        self.m, self.S, self.l, self.rho, self.g = ac.m, ac.S, ac.l, fc.rho, fc.g
        self.V0, self.theta0 = fc.V, fc.theta
        self.thrust_max, self.deflection_max, self.envelope = (thrust_max, deflection_max,
                                                               envelope)
        lon = ac.longitudinal
        self.CL0 = -lon.CZ_u / 2.0
        CD0 = -lon.CX_u / 2.0
        self.CLa = -lon.CZ_alpha - CD0
        if self.CL0 <= 0.0 or self.CLa <= 0.0:
            raise ModelError(f"{ac.name}: trim lift coefficient ({self.CL0:.4f}) and lift-curve "
                             f"slope ({self.CLa:.4f}) must both be positive")
        # parabolic polar whose slope at trim reproduces CX_alpha
        self.k = (self.CL0 - lon.CX_alpha) / (2.0 * self.CL0 * self.CLa)
        self.CDmin = CD0 - self.k * self.CL0 ** 2
        self.CL_max = math.inf
        self.performance = performance
        if performance is not None:
            self.k, self.CDmin, self.CL_max = performance.k, performance.CD0, performance.CL_max
            # with a lift limit known, the valid range runs up to the angle that reaches it
            self.envelope = replace(envelope, alpha_max=(self.CL_max - self.CL0) / self.CLa)
            CD0 = self.CDmin + self.k * self.CL0 ** 2
        qS = 0.5 * self.rho * self.V0 ** 2 * self.S
        self.thrust_trim = qS * CD0 + self.m * self.g * math.sin(self.theta0)
        self.inertia = np.array([[ac.A, 0.0, -ac.E], [0.0, ac.B, 0.0], [-ac.E, 0.0, ac.C]])
        self._inertia_inv = np.linalg.inv(self.inertia)

    # ------------------------------------------------------------------ states
    def trim_state(self, north=0.0, east=0.0, altitude=0.0, heading=0.0):
        q = quaternion_from_euler(0.0, self.theta0, heading)
        return np.array([north, east, -altitude, self.V0, 0.0, 0.0, *q, 0.0, 0.0, 0.0])

    def trim_controls(self):
        return np.array([self.thrust_trim, 0.0, 0.0, 0.0])

    def limit(self, controls):
        thrust, dm, dl, dn = controls
        d = self.deflection_max
        return (min(max(thrust, 0.0), self.thrust_max), min(max(dm, -d), d),
                min(max(dl, -d), d), min(max(dn, -d), d))

    # ------------------------------------------------------------ aerodynamics
    def air_data(self, u, v, w):
        V = math.sqrt(u * u + v * v + w * w)
        return V, math.atan2(w, u), math.asin(max(-1.0, min(1.0, v / V)))

    def aerodynamics(self, V, alpha, beta, p, q, r, dm, dl, dn):
        """Body-axis aerodynamic force (X, Y, Z) and moment (L, M, N), and lift coefficient."""
        lon, lat = self.aircraft.longitudinal, self.aircraft.lateral
        qS = 0.5 * self.rho * V * V * self.S
        ph, qh, rh = p * self.l / V, q * self.l / V, r * self.l / V
        CL_static = self.CL0 + self.CLa * alpha
        CL = CL_static - lon.CZ_q * qh - lon.CZ_dm * dm
        if self.performance is None:
            CD = self.CDmin + self.k * CL_static * CL_static
        else:
            CL = max(-self.CL_max, min(self.CL_max, CL))   # flat-topped lift beyond the limit
            CD = self.CDmin + self.k * CL * CL
        ca, sa = math.cos(alpha), math.sin(alpha)
        Cx = CL * sa - CD * ca + lon.CX_dm * dm
        Cz = -CL * ca - CD * sa
        Cm = (lon.Cm_alpha * alpha + lon.Cm_q * qh + lon.Cm_dm * dm
              + lon.Cm_u * (V - self.V0) / self.V0)
        CY = (lat.CY_beta * beta + lat.CY_p * ph + lat.CY_r * rh
              + lat.CY_dl * dl + lat.CY_dn * dn)
        Cl = (lat.Cl_beta * beta + lat.Cl_p * ph + lat.Cl_r * rh
              + lat.Cl_dl * dl + lat.Cl_dn * dn)
        Cn = (lat.Cn_beta * beta + lat.Cn_p * ph + lat.Cn_r * rh
              + lat.Cn_dl * dl + lat.Cn_dn * dn)
        ql = qS * self.l
        return (qS * Cx, qS * CY, qS * Cz), (ql * Cl, ql * Cm, ql * Cn), CL

    # ---------------------------------------------------------------- dynamics
    def derivative(self, x, controls):
        _, _, _, u, v, w, q0, q1, q2, q3, p, q, r = x
        thrust, dm, dl, dn = controls
        V, alpha, beta = self.air_data(u, v, w)
        (X, Y, Z), (L, M, N), _ = self.aerodynamics(V, alpha, beta, p, q, r, dm, dl, dn)
        g, m = self.g, self.m

        # body -> earth rotation matrix from the quaternion
        r11 = 1 - 2 * (q2 * q2 + q3 * q3)
        r12 = 2 * (q1 * q2 - q0 * q3)
        r13 = 2 * (q1 * q3 + q0 * q2)
        r21 = 2 * (q1 * q2 + q0 * q3)
        r22 = 1 - 2 * (q1 * q1 + q3 * q3)
        r23 = 2 * (q2 * q3 - q0 * q1)
        r31 = 2 * (q1 * q3 - q0 * q2)
        r32 = 2 * (q2 * q3 + q0 * q1)
        r33 = 1 - 2 * (q1 * q1 + q2 * q2)

        du = r * v - q * w + (X + thrust) / m + g * r31
        dv = p * w - r * u + Y / m + g * r32
        dw = q * u - p * v + Z / m + g * r33

        I = self.inertia
        hx = I[0, 0] * p + I[0, 2] * r
        hy = I[1, 1] * q
        hz = I[2, 0] * p + I[2, 2] * r
        rates = self._inertia_inv @ (L - (q * hz - r * hy), M - (r * hx - p * hz),
                                     N - (p * hy - q * hx))
        return np.array([
            r11 * u + r12 * v + r13 * w,
            r21 * u + r22 * v + r23 * w,
            r31 * u + r32 * v + r33 * w,
            du, dv, dw,
            0.5 * (-q1 * p - q2 * q - q3 * r),
            0.5 * (q0 * p + q2 * r - q3 * q),
            0.5 * (q0 * q + q3 * p - q1 * r),
            0.5 * (q0 * r + q1 * q - q2 * p),
            rates[0], rates[1], rates[2]])

    def outputs(self, x, controls):
        """Named quantities derived from a state, for logging and control."""
        north, east, down, u, v, w, q0, q1, q2, q3, p, q, r = x
        V, alpha, beta = self.air_data(u, v, w)
        phi, theta, psi = euler_from_quaternion(q0, q1, q2, q3)
        (_, _, Z), _, CL = self.aerodynamics(V, alpha, beta, p, q, r, *controls[1:])
        vn = ((1 - 2 * (q2 * q2 + q3 * q3)) * u + 2 * (q1 * q2 - q0 * q3) * v
              + 2 * (q1 * q3 + q0 * q2) * w)
        ve = (2 * (q1 * q2 + q0 * q3) * u + (1 - 2 * (q1 * q1 + q3 * q3)) * v
              + 2 * (q2 * q3 - q0 * q1) * w)
        vd = (2 * (q1 * q3 - q0 * q2) * u + 2 * (q2 * q3 + q0 * q1) * v
              + (1 - 2 * (q1 * q1 + q2 * q2)) * w)
        e = self.envelope
        return {
            "north": north, "east": east, "altitude": -down, "V": V, "alpha": alpha,
            "beta": beta, "phi": phi, "theta": theta, "psi": psi, "p": p, "q": q, "r": r,
            "load_factor": -Z / (self.m * self.g), "CL": CL,
            "course": math.atan2(ve, vn), "climb_rate": -vd,
            "ground_speed": math.hypot(vn, ve),
            "valid": float(e.alpha_min <= alpha <= e.alpha_max and abs(beta) <= e.beta_max
                           and abs(CL) < self.CL_max),
            **dict(zip(CONTROLS, controls)),
        }


@dataclass(frozen=True)
class Trajectory:
    """Time history of a nonlinear run: t, the raw states x, and named output arrays."""

    t: np.ndarray
    x: np.ndarray
    data: dict

    def __getitem__(self, name):
        return self.data[name]

    @property
    def fraction_outside_envelope(self):
        return float(1.0 - self.data["valid"].mean())


def simulate_nonlinear(model, t_end, dt=0.01, controller=None, x0=None, stop=None):
    """Fixed-step RK4. controller(t, outputs, model) -> controls, held over each step.

    stop() is checked after each controller call; when it returns True the run ends there.
    """
    x = model.trim_state() if x0 is None else np.array(x0, float)
    trim = tuple(model.trim_controls())
    steps = int(round(t_end / dt))
    times, states, rows = [], [], []
    controls = trim
    for i in range(steps + 1):
        t = i * dt
        out = model.outputs(x, controls)
        if controller is not None:
            controls = model.limit(controller(t, out, model))
            out = model.outputs(x, controls)
        out.update(getattr(controller, "telemetry", None) or {})
        times.append(t)
        states.append(x.copy())
        rows.append(out)
        if i == steps or (stop is not None and stop()):
            break
        k1 = model.derivative(x, controls)
        k2 = model.derivative(x + 0.5 * dt * k1, controls)
        k3 = model.derivative(x + 0.5 * dt * k2, controls)
        k4 = model.derivative(x + dt * k3, controls)
        x = x + dt / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)
        x[6:10] /= math.sqrt(x[6] ** 2 + x[7] ** 2 + x[8] ** 2 + x[9] ** 2)
    data = {key: np.array([row[key] for row in rows]) for key in rows[0]}
    return Trajectory(np.array(times), np.array(states), data)
