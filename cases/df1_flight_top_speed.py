"""Level top speed and excess power from the DF1 flight log: python3 -m cases.df1_flight_top_speed

The pilot never held full throttle in steady level flight, so top speed cannot be read off
directly. Instead, every full-throttle sample near wings-level and near 1 g gives the
specific excess power  Ps = dh/dt + (V/g) dV/dt  (rate of gain of height plus speed-height).
Ps falls with speed and is zero at the level top speed. The same quantity from the model
is (thrust - drag) V / weight, so the two can be compared across the speed range.

Flight: Mission 1, empty (per Ishan). See docs/flight-validation.md.
"""

import math
from pathlib import Path

import numpy as np
from pyulog import ULog
from scipy.signal import savgol_filter

from cases.df1_flight_log import LOG, bank_and_pitch_deg, event_time, seconds, topic
from src.aircraft import load_aircraft
from src.performance import Envelope, load_performance

AIRCRAFT = "data/aircraft/df1_m1_asbuilt.toml"
PERFORMANCE = "data/performance/df1_m1.toml"
STEP = 0.1                 # s, common time base
SMOOTH = 15                # samples (1.5 s) in the smoothing window
FULL_THROTTLE = 0.97
BANK_LIMIT = 25.0          # deg
LOAD_FACTOR_BAND = 0.25    # keep samples with |n - 1| below this
BINS = range(18, 38, 2)    # m/s
G = 9.81


def flight_samples():
    """Smoothed airspeed, height, throttle, bank and load factor on a common time base."""
    log = ULog(str(LOG))
    takeoff, landing = event_time(log, "Takeoff detected"), event_time(log, "Landing detected")
    t = np.arange(takeoff + 5.0, landing - 5.0, STEP)
    air = topic(log, "airspeed_validated")
    valid = np.isfinite(air["true_airspeed_m_s"])
    position = topic(log, "vehicle_local_position")
    motors = topic(log, "actuator_motors")
    attitude = topic(log, "vehicle_attitude")
    acceleration = topic(log, "vehicle_acceleration")
    bank, _ = bank_and_pitch_deg(attitude)
    V = savgol_filter(np.interp(t, seconds(air, log)[valid], air["true_airspeed_m_s"][valid]),
                      SMOOTH, 2)
    h = savgol_filter(np.interp(t, seconds(position, log), -position["z"]), SMOOTH, 2)
    return {
        "t": t, "V": V, "h": h,
        "throttle": np.interp(t, seconds(motors, log), motors["control[0]"]),
        "bank": np.interp(t, seconds(attitude, log), bank),
        "load_factor": savgol_filter(
            np.interp(t, seconds(acceleration, log), -acceleration["xyz[2]"] / G), 5, 1),
        "excess_power": np.gradient(h, t) + V * np.gradient(V, t) / G,
        "rho": float(np.median(topic(log, "vehicle_air_data")["rho"])),
    }


def selected(s):
    """Full throttle, near wings-level, near 1 g."""
    return ((s["throttle"] >= FULL_THROTTLE) & (np.abs(s["bank"]) < BANK_LIMIT)
            & (np.abs(s["load_factor"] - 1.0) < LOAD_FACTOR_BAND))


def binned(s, keep):
    """(mean speed, median Ps, 25th, 75th percentile, count) per speed bin with enough data."""
    rows = []
    for low in BINS:
        k = keep & (s["V"] >= low) & (s["V"] < low + 2)
        if k.sum() >= 8:
            ps = s["excess_power"][k]
            rows.append((float(s["V"][k].mean()), float(np.median(ps)),
                         float(np.percentile(ps, 25)), float(np.percentile(ps, 75)), int(k.sum())))
    return rows


def top_speed_from(rows):
    """Speed at which a straight line through the binned Ps reaches zero, and its slope."""
    V = np.array([r[0] for r in rows])
    ps = np.array([r[1] for r in rows])
    slope, intercept = np.polyfit(V, ps, 1, w=np.sqrt([r[4] for r in rows]))
    return -intercept / slope, slope


def model_envelope(rho):
    aircraft, condition = load_aircraft(AIRCRAFT)
    performance = load_performance(PERFORMANCE, rho)
    return Envelope(aircraft.m, aircraft.S, rho, G, performance, performance.powertrain.tabulate())


def main():
    s = flight_samples()
    keep = selected(s)
    envelope = model_envelope(s["rho"])
    full = s["throttle"] >= FULL_THROTTLE
    print(f"{LOG.name}: air density {s['rho']:.3f} kg/m^3")
    print(f"Full throttle for {full.sum() * STEP:.0f} s of {len(s['t']) * STEP:.0f} s airborne; "
          f"{keep.sum() * STEP:.0f} s of that near wings-level and near 1 g.")

    peak = int(np.argmax(s["V"]))
    print(f"\nHighest airspeed {s['V'][peak]:.1f} m/s ({s['V'][peak] * 2.23694:.0f} mph) at "
          f"{s['t'][peak]:.0f} s, while descending at {-np.gradient(s['h'], s['t'])[peak]:.0f} m/s "
          f"at full throttle: a dive, not level flight.")

    rows = binned(s, keep)
    print("\nSpecific excess power at full throttle (m/s; positive means able to climb or accelerate):")
    print(f"{'speed':>9} {'samples':>8} {'flight (median)':>16} {'middle half':>16} {'model':>8}")
    for V, median, low, high, count in rows:
        print(f"{V:>7.1f}   {count:>8d} {median:>+16.1f} {low:>+8.1f} to {high:>+5.1f} "
              f"{envelope.climb_rate(V):>+8.1f}")

    measured, slope = top_speed_from(rows)
    model_rows = [(r[0], envelope.climb_rate(r[0]), 0, 0, r[4]) for r in rows]
    _, model_slope = top_speed_from(model_rows)
    print(f"\nLevel top speed (where excess power is zero):")
    print(f"  flight, straight-line fit   {measured:.1f} m/s ({measured * 2.23694:.0f} mph)")
    print(f"  model at this air density   {envelope.top_speed:.1f} m/s "
          f"({envelope.top_speed * 2.23694:.0f} mph)")
    print(f"Fall of excess power with speed: flight {slope:.2f}, model {model_slope:.2f} "
          "m/s per m/s")


if __name__ == "__main__":
    main()
