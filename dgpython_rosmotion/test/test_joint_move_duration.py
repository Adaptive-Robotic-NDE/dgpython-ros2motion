"""The duration bound on a planned joint move.

A planned joint move is bounded only by its velocity scaling, and a scaling says
nothing about how fast the TOOL goes -- that is the Jacobian's business. So the
one handle a caller has on the speed of a joint move near a part is the duration
it asks for, and for a long time that handle did nothing: `secs` was read by the
direct fallback and dropped by the planned route.

The cost was measured. azure_gauge.dpi's height gauge computes a duration from
the straight-line distance at the "near the part" class limit of 10 mm/s,
precisely so that "the duration asked for cannot itself be the thing that
exceeds the limit". On two of three measurements the Cartesian route was refused
for reach and the gauge fell through to a planned joint move, which then ran at
JOINT_MOVE_SCALING's 0.015 instead: 437.8 mm of descent in 13.8 s, 31.6 mm/s at
the tool and 92% of wrist 1's scaled limit, three times the limit the gauge had
just computed for itself, on an approach to the part.

Unlike the Cartesian case the conversion here is sound and worth testing as
arithmetic: a joint-space scaling bounds joint velocity directly, so the
duration a scaling implies is max_j(|dq_j| / (scaling * vmax_j)) with no
kinematics in it.

Runs on the HOST with no ROS. motion.py imports rclpy and half of MoveIt, so the
helper is lifted out of its source with ast rather than imported, the same way
test_wrist3_band.py does it.
"""
import ast
import os

MOTION_PY = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         "..", "dgpython_rosmotion", "motion.py")


class _FakeState(object):
    def __init__(self, names, positions):
        self.name = list(names)
        self.position = list(positions)


class _Node(object):
    """The two attributes _scaling_for_duration actually touches."""
    JOINT_VMAX_CEILING = None       # filled from the source, not duplicated here

    def __init__(self, state):
        self._joint_state = state


def _scaling_for_duration():
    """Lift _scaling_for_duration and JOINT_VMAX_CEILING out of motion.py."""
    tree = ast.parse(open(MOTION_PY).read(), filename="motion.py")
    fn = None
    ceiling = None
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == "_scaling_for_duration":
            fn = node
        if isinstance(node, ast.Assign):
            for t in node.targets:
                if isinstance(t, ast.Name) and t.id == "JOINT_VMAX_CEILING":
                    ceiling = ast.literal_eval(node.value)
    assert fn is not None, "_scaling_for_duration is gone from motion.py"
    assert ceiling is not None, "JOINT_VMAX_CEILING is gone from motion.py"
    mod = ast.Module(body=[fn], type_ignores=[])
    ns = {}
    exec(compile(ast.fix_missing_locations(mod), "motion.py", "exec"), ns)
    _Node.JOINT_VMAX_CEILING = ceiling
    _Node._scaling_for_duration = ns["_scaling_for_duration"]
    return ceiling


JOINTS = ("shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
          "wrist_1_joint", "wrist_2_joint", "wrist_3_joint")


def _node(here):
    _scaling_for_duration()
    return _Node(_FakeState(JOINTS, here))


def _call(here, target, secs):
    n = _node(here)
    return n._scaling_for_duration(list(target), list(JOINTS), secs)


def test_the_measured_gauge_descent_comes_back_to_its_class_limit():
    """The real event: 0.5973 rad of wrist 1 over a 437.8 mm, 10 mm/s descent.

    It ran at 31.6 mm/s. The duration this must derive a scaling for is
    437.8/10 = 43.8 s, and the scaling it returns has to put the move at or
    under its class limit rather than at three times it.
    """
    ceiling = _scaling_for_duration()
    secs = 0.4378 / 0.010
    s = _call([0.0]*6, [0.0, 0.0, 0.0, 0.5973, 0.0, 0.0], secs)
    assert s is not None
    # Duration the scaling implies for the joint that actually travels.
    implied = 0.5973 / (s * 3.1416)
    assert implied >= secs * 0.95, (
        "the derived scaling is faster than the duration asked for: %.1f s "
        "against %.1f s" % (implied, secs))
    tool_mps = 0.4378 / implied
    assert tool_mps <= 0.0105, (
        "tool speed %.1f mm/s still exceeds the 10 mm/s class limit" % (tool_mps*1000))
    assert s < 0.015, (
        "the derived scaling %g is no slower than JOINT_MOVE_SCALING, so this "
        "changes nothing for the move that was measured at 31.6 mm/s" % s)
    assert ceiling >= 3.1416, (
        "the vmax ceiling must be an UPPER bound on every joint limit, or the "
        "derived scaling comes out too FAST; got %g" % ceiling)


def test_it_errs_slow_rather_than_fast():
    """A single upper-bound vmax must never imply a shorter move than asked.

    This is the whole reason the largest joint limit is used for every joint
    instead of per-joint values: getting it wrong in this direction would make
    a duration bound into a duration floor.
    """
    for dq in (0.1, 0.5, 1.0, 2.5):
        for secs in (4.0, 10.0, 43.8, 120.0):
            s = _call([0.0]*6, [dq, 0, 0, 0, 0, 0], secs)
            if s is None or s >= 1.0:
                continue
            for vmax in (2.0944, 3.1416):      # every real UR joint limit
                implied = dq / (s * vmax)
                assert implied >= secs * 0.95, (
                    "dq=%g secs=%g vmax=%g implies %.2f s, shorter than asked"
                    % (dq, secs, vmax, implied))


def test_it_declines_rather_than_guesses_when_it_cannot_measure():
    """No joint states, an unknown joint, or no travel -> None, not a number.

    Returning a scaling here would be inventing a bound from nothing, and the
    caller leaves the scaling alone on None. A move of no length is also None:
    there is nothing to slow down and a zero would stall it.
    """
    _scaling_for_duration()
    assert _Node(None)._scaling_for_duration([0.0]*6, list(JOINTS), 10.0) is None
    n = _node([0.0]*6)
    assert n._scaling_for_duration([0.0]*6, ["no_such_joint"]*6, 10.0) is None
    assert n._scaling_for_duration([0.0]*6, list(JOINTS), 10.0) is None  # no travel
    assert _call([0.0]*6, [1.0, 0, 0, 0, 0, 0], 0.0) is None
    assert _call([0.0]*6, [1.0, 0, 0, 0, 0, 0], -5.0) is None
    assert _call([0.0]*6, [1.0, 0, 0, 0, 0, 0], "nonsense") is None


def test_a_short_move_is_not_driven_below_the_floor():
    """Clamped into (0, 1]: a huge duration must not return 0 and stall."""
    s = _call([0.0]*6, [1e-6, 0, 0, 0, 0, 0], 100000.0)
    assert s is not None and s > 0.0
    s = _call([0.0]*6, [6.0, 0, 0, 0, 0, 0], 0.001)
    assert s is not None and s <= 1.0
