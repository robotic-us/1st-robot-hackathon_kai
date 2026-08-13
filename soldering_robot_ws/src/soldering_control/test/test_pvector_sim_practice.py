import pytest

from soldering_control.pvector_sim_practice import quintic_point


def test_quintic_has_expected_boundary_conditions():
    start = quintic_point(-2.0, 5.0, 0.0, 5.0)
    end = quintic_point(-2.0, 5.0, 5.0, 5.0)

    assert start.position_deg == pytest.approx(-2.0)
    assert start.velocity_deg_s == pytest.approx(0.0)
    assert start.acceleration_deg_s2 == pytest.approx(0.0)
    assert end.position_deg == pytest.approx(5.0)
    assert end.velocity_deg_s == pytest.approx(0.0)
    assert end.acceleration_deg_s2 == pytest.approx(0.0)


def test_quintic_rejects_non_positive_duration():
    with pytest.raises(ValueError):
        quintic_point(0.0, 1.0, 0.0, 0.0)
