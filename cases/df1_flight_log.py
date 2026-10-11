"""Summary of the DF1 flight log: python3 -m cases.df1_flight_log

Prints what the PX4 log contains and whether it holds hands-off segments from which
natural modes could be read. This is an inventory of the data, not a comparison with the
model; see docs/flight-validation.md.
"""

from pathlib import Path

import numpy as np
from pyulog import ULog
from scipy.signal import welch

LOG = Path("data/flight/df1_log_178_2026-08-29.ulg")
NAV_STATES = {0: "Manual", 1: "Altitude", 2: "Position", 15: "Stabilized"}
MARGIN = 5.0           # seconds trimmed after takeoff and before landing
QUIET_SPAN = 0.04      # stick travel (fraction of full) allowed in a "hands-off" window
QUIET_SECONDS = 3.0
LEVEL_BANK_DEG = 10.0


def topic(log, name):
    return next(d for d in log.data_list if d.name == name and d.multi_id == 0).data


def seconds(data, log):
    return (data["timestamp"].astype(float) - log.start_timestamp) / 1e6


def event_time(log, text):
    return next((m.timestamp - log.start_timestamp) / 1e6
                for m in log.logged_messages if text in m.message)


def mode_segments(log):
    status = topic(log, "vehicle_status")
    t, nav = seconds(status, log), status["nav_state"]
    segments, start = [], 0
    for i in range(1, len(nav) + 1):
        if i == len(nav) or nav[i] != nav[start]:
            segments.append((max(t[start], 0.0), t[i - 1],
                             NAV_STATES.get(int(nav[start]), f"state {nav[start]}")))
            start = i
    return segments


def bank_and_pitch_deg(attitude):
    q0, q1, q2, q3 = (attitude[f"q[{i}]"] for i in range(4))
    bank = np.degrees(np.arctan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 ** 2 + q2 ** 2)))
    pitch = np.degrees(np.arcsin(np.clip(2 * (q0 * q2 - q3 * q1), -1, 1)))
    return bank, pitch


def quiet_windows(t, sticks, in_flight):
    """Windows where every stick axis stays within QUIET_SPAN for QUIET_SECONDS or more."""
    windows, i = [], 0
    while i < len(t):
        j = i
        while (j + 1 < len(t) and in_flight[j + 1]
               and np.all(np.ptp(sticks[i:j + 2], axis=0) < QUIET_SPAN)):
            j += 1
        if in_flight[i] and t[j] - t[i] >= QUIET_SECONDS:
            windows.append((t[i], t[j]))
            i = j + 1
        else:
            i += 1
    return windows


def main():
    log = ULog(str(LOG))
    takeoff, landing = event_time(log, "Takeoff detected"), event_time(log, "Landing detected")
    lo, hi = takeoff + MARGIN, landing - MARGIN
    print(f"{LOG.name}: {(log.last_timestamp - log.start_timestamp) / 1e6:.0f} s logged, "
          f"takeoff at {takeoff:.1f} s, landing at {landing:.1f} s\n")

    print("Flight modes:")
    totals = {}
    for start, end, name in mode_segments(log):
        print(f"  {start:6.1f} - {end:6.1f} s  {name}")
        overlap = max(0.0, min(end, landing) - max(start, takeoff))
        totals[name] = totals.get(name, 0.0) + overlap
    print("  airborne time by mode: "
          + ", ".join(f"{name} {total:.0f} s" for name, total in totals.items()))

    air = topic(log, "airspeed_validated")
    tas = air["true_airspeed_m_s"][(seconds(air, log) > lo) & (seconds(air, log) < hi)]
    rho = np.median(topic(log, "vehicle_air_data")["rho"])
    print(f"\nTrue airspeed: {np.nanmin(tas):.1f} to {np.nanmax(tas):.1f} m/s, "
          f"median {np.nanmedian(tas):.1f} m/s.  Air density {rho:.3f} kg/m^3")

    attitude = topic(log, "vehicle_attitude")
    flying = (seconds(attitude, log) > lo) & (seconds(attitude, log) < hi)
    bank, pitch = bank_and_pitch_deg(attitude)
    print(f"Bank angle: {np.percentile(bank[flying], 5):.0f} to "
          f"{np.percentile(bank[flying], 95):.0f} deg (5th-95th percentile); within "
          f"{LEVEL_BANK_DEG:.0f} deg of level {np.mean(np.abs(bank[flying]) < LEVEL_BANK_DEG):.0%} "
          f"of the time.  Pitch: {np.percentile(pitch[flying], 5):.0f} to "
          f"{np.percentile(pitch[flying], 95):.0f} deg")

    print("\nLogging rates (Hz):")
    for name, label in (("sensor_combined", "gyro and accelerometer"),
                        ("vehicle_angular_velocity", "body rates"),
                        ("vehicle_torque_setpoint", "commanded control torques"),
                        ("vehicle_attitude", "attitude"),
                        ("actuator_servos", "servo outputs"),
                        ("airspeed_validated", "airspeed"),
                        ("manual_control_setpoint", "stick positions")):
        t = seconds(topic(log, name), log)
        print(f"  {label:28} {(len(t) - 1) / (t[-1] - t[0]):6.1f}")

    manual = topic(log, "manual_control_setpoint")
    t = seconds(manual, log)
    sticks = np.column_stack([manual[k] for k in ("roll", "pitch", "yaw")])
    windows = quiet_windows(t, sticks, (t > lo) & (t < hi))
    print(f"\nHands-off windows (all sticks within {QUIET_SPAN:.0%} of travel for "
          f"{QUIET_SECONDS:.0f} s or more): {len(windows)}")
    for start, end in windows:
        print(f"  {start:6.1f} - {end:6.1f} s")

    gyro = topic(log, "sensor_combined")
    t = seconds(gyro, log)
    rate = (len(t) - 1) / (t[-1] - t[0])
    print("\nStrongest components of the body rates between 0.3 and 8 Hz:")
    for axis, label in enumerate(("roll rate", "pitch rate", "yaw rate")):
        f, power = welch(gyro[f"gyro_rad[{axis}]"][(t > lo) & (t < hi)], fs=rate, nperseg=2048)
        band = (f > 0.3) & (f < 8.0)
        peaks = np.sort(f[band][np.argsort(power[band])[-4:]])
        print(f"  {label:11} " + ", ".join(f"{p:.2f}" for p in peaks) + " Hz")


if __name__ == "__main__":
    main()
