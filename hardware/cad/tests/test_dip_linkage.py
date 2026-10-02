"""Checks the DIP-PIP coupling linkage of every V1 finger (hardware/cad/dip_linkage.py)."""

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import dip_linkage as dl

LINKAGES = dl.linkages()
PIP = np.radians(np.linspace(*dl.PIP_RANGE_DEG, 101))


def pins(d, pip, dip):
    """Pin A and pin B (mm) in the middle phalanx's frame, straight from the design numbers."""
    a, ga = d["pin_a"]["r_mm"], math.radians(d["pin_a"]["angle_deg"])
    b, gb = d["pin_b"]["r_mm"], math.radians(d["pin_b"]["angle_deg"])
    pa = np.stack([a * np.cos(ga + pip), a * np.sin(ga + pip)], axis=-1)
    pb = np.stack([d["middle_length_mm"] + b * np.cos(gb - dip), b * np.sin(gb - dip)], axis=-1)
    return pa, pb


def poly_dip(d, pip):
    return sum(c * pip**k for k, c in enumerate(d["polycoef"]))


def test_every_finger_has_a_linkage():
    assert set(LINKAGES) == set(dl.FINGERS)
    lengths = dl.middle_lengths()
    for f, d in LINKAGES.items():
        assert d["middle_length_mm"] == pytest.approx(lengths[f], abs=1e-3)


@pytest.mark.parametrize("finger", dl.FINGERS)
def test_bar_keeps_its_length_along_the_fitted_curve(finger):
    """The independent check: with the DIP on the fitted curve, the rigid bar fits at every angle."""
    d = LINKAGES[finger]
    pa, pb = pins(d, PIP, poly_dip(d, PIP))
    bar = np.linalg.norm(pa - pb, axis=1)
    assert np.abs(bar - d["bar_mm"]).max() < 0.02  # mm


@pytest.mark.parametrize("finger", dl.FINGERS)
def test_dip_follows_the_ratio(finger):
    d = LINKAGES[finger]
    err = np.degrees(np.abs(poly_dip(d, PIP) - dl.RATIO * PIP)).max()
    assert err < 1.0
    assert poly_dip(d, 0.0) == 0.0  # straight finger: straight fingertip


@pytest.mark.parametrize("finger", dl.FINGERS)
def test_linkage_fits_inside_the_finger_and_never_locks(finger):
    d = LINKAGES[finger]
    for key in ("pin_a", "pin_b"):
        assert dl.PIN_R_MIN - 1e-9 <= d[key]["r_mm"] <= dl.PIN_R_MAX + 1e-9
    pa, pb = pins(d, PIP, poly_dip(d, PIP))
    assert max(np.abs(pa[:, 1]).max(), np.abs(pb[:, 1]).max()) <= dl.HALF_HEIGHT + 1e-6
    assert d["axis_clear_mm"] >= dl.AXIS_CLEAR
    assert d["min_transmission_deg"] >= 30
