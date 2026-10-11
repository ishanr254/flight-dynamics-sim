# Flight data

## df1_log_178_2026-08-29.ulg

PX4 flight log from DF1 (Pixhawk 6C, `PX4_FMU_V6C`), uploaded by Ishan on 2026-10-11.
Original file name: `log_178_2026-8-29-10-40-30 (1).ulg`. Read with the `pyulog` package.

What the log means for validating the tool, and the planned comparison, are in
`docs/flight-validation.md`. Regenerate the summary below with
`python3 -m cases.df1_flight_log`.

First look (2026-10-11); no comparison with the model has been made yet:

- Duration 260 s; takeoff detected at 28.7 s, landing at 257.3 s.
- Flight modes: Manual to 86.8 s (with three short Altitude-mode spells), Stabilized
  86.8-152.1 s, Manual 152.6-219.4 s, Stabilized 219.5-254.5 s.
- True airspeed 15.5-36.6 m/s, median 24.2 m/s. Air density about 1.107 kg/m^3.
- Bank angle between -55 and +58 deg (5th-95th percentile); within 10 deg of level for
  about 30% of the flight.
- Logging rates: gyro and accelerometer 200 Hz (`sensor_combined`), body rates and
  torque/thrust setpoints 50 Hz, attitude 20 Hz, servo outputs 10 Hz, airspeed 5 Hz,
  stick positions 5 Hz.
- No window of 3 s or longer with all three sticks held still, so there are no clean
  hands-off (free-response) segments. Rate spectra are dominated by pilot inputs below
  1 Hz.

Not recorded in the log (`WEIGHT_GROSS` is unset). Per Ishan, 2026-10-11: this was the
Mission 1 empty configuration, flown on what he believes were mid rates (about 75% of
maximum control deflection). Measured takeoff weight and CG are not available.
