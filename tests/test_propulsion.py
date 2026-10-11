"""Propeller, motor and battery model, and its check against the static thrust test."""

import math
from dataclasses import replace

import pytest

from src.propulsion import Battery, Motor, Powertrain, load_propeller

PROPELLER = "data/propulsion/apc_15x6E.json"
STATIC_TEST_THRUST = 46.0     # N, design report Table 8.1.2 (10.34 lbf)
STATIC_TEST_CURRENT = 60.39   # A


@pytest.fixture(scope="module")
def propeller():
    return load_propeller(PROPELLER)


@pytest.fixture(scope="module")
def powertrain(propeller):
    return Powertrain(Motor(520.0, 1200.0), Battery(6, 4.2, 35.0), propeller, 1.225, 100.0)


def test_static_thrust_follows_the_coefficient_table(propeller):
    """At a tabulated RPM and zero speed, T = Ct rho n^2 D^4 with the table's first Ct."""
    J, Ct, _ = propeller.tables[list(propeller.rpms).index(6000.0)]
    assert J[0] == 0.0
    expected = Ct[0] * 1.225 * 100.0 ** 2 * propeller.diameter ** 4
    assert propeller.thrust(6000.0, 0.0, 1.225) == pytest.approx(expected)


def test_thrust_falls_with_airspeed_and_goes_negative_when_windmilling(propeller):
    thrusts = [propeller.thrust(10000.0, V, 1.225) for V in (0.0, 15.0, 30.0, 45.0)]
    assert thrusts == sorted(thrusts, reverse=True)
    assert thrusts[-1] < 0.0


def test_motor_and_battery_constants(powertrain):
    assert powertrain.motor.kt == pytest.approx(60.0 / (2 * math.pi * 520.0))
    assert 0.01 < powertrain.motor.resistance < 0.06
    assert powertrain.battery.voltage == pytest.approx(22.2)
    assert powertrain.battery.resistance == pytest.approx(0.013 / 4.2 * 6)
    assert powertrain.battery.usable_ah == pytest.approx(4.2 * 0.85)


def test_operating_point_balances_the_voltages(powertrain):
    for throttle, speed in ((1.0, 0.0), (0.6, 20.0), (1.0, 28.0)):
        op = powertrain.operating_point(throttle, speed)
        needed = op.rpm / powertrain.motor.kv + op.current * powertrain.motor.resistance
        assert needed == pytest.approx(throttle * op.voltage, rel=1e-6)
        assert op.electrical_power == pytest.approx(op.current * op.voltage)


def test_zero_throttle_gives_nothing(powertrain):
    op = powertrain.operating_point(0.0, 20.0)
    assert op.thrust == 0.0 and op.current == 0.0


def test_thrust_rises_with_throttle(powertrain):
    thrusts = [powertrain.operating_point(t, 15.0).thrust for t in (0.3, 0.5, 0.7, 1.0)]
    assert thrusts == sorted(thrusts)


def test_current_limit_is_enforced(powertrain):
    fused = replace(powertrain, current_limit=30.0)
    op = fused.operating_point(1.0, 0.0)
    assert op.current_limited and op.current == pytest.approx(30.0, rel=1e-6)
    assert op.thrust < powertrain.operating_point(1.0, 0.0).thrust


def test_uncalibrated_model_overpredicts_the_static_test(powertrain):
    """As the report's own MATLAB and eCalc curves do (Figure 8.1.2)."""
    op = powertrain.operating_point(1.0, 0.0)
    assert 1.10 < op.thrust / STATIC_TEST_THRUST < 1.35
    assert op.current == pytest.approx(STATIC_TEST_CURRENT, rel=0.15)


def test_calibration_factor_scales_thrust_only(powertrain):
    calibrated = replace(powertrain, thrust_factor=0.83)
    a, b = powertrain.operating_point(1.0, 10.0), calibrated.operating_point(1.0, 10.0)
    assert b.thrust == pytest.approx(0.83 * a.thrust)
    assert b.current == pytest.approx(a.current) and b.rpm == pytest.approx(a.rpm)


def test_lookup_table_matches_the_direct_solution(powertrain):
    table = powertrain.tabulate()
    for throttle, speed in ((1.0, 0.0), (0.55, 13.0), (0.9, 27.7), (1.0, 31.0)):
        op = powertrain.operating_point(throttle, speed)
        assert table.thrust(throttle, speed) == pytest.approx(op.thrust, abs=0.4)
        assert table.current(throttle, speed) == pytest.approx(op.current, abs=0.8)
    assert table.max_thrust(20.0) == pytest.approx(powertrain.operating_point(1.0, 20.0).thrust)


def test_throttle_for_inverts_thrust(powertrain):
    table = powertrain.tabulate()
    throttle = table.throttle_for(10.0, 25.0)
    assert 0.0 < throttle < 1.0
    assert table.thrust(throttle, 25.0) == pytest.approx(10.0, abs=0.05)
    assert table.throttle_for(1e6, 25.0) == 1.0
