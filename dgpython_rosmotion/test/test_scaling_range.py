"""Scalings outside (0, 1] must be refused, not turned into full speed.

moveit_core's verifyScalingFactor accepts only (0, 1]. Handed anything else --
negative, above one, NaN, and critically **0.0** -- it logs one WARN and
substitutes **1.0**. So the failure mode of a scaling that is zero or missing,
because a caller left a field unset or a lookup like `vel=table.get(name)`
missed, is not a refused move and not a slow one. It is a move at 100% of every
joint limit, next to the part, with one warning line in a log nobody reads while
a scan is running.

The margin for error is the whole point: the scan classes in this cell run
between 0.0025 and 0.02, so a mistake of this shape is three orders of magnitude
the wrong way, and the arm has 2.09-3.14 rad/s of joint speed to spend on it.

There is deliberately NO LOWER BOUND. 0.00254 is a real, correct, measured
scaling for a 6 mm/s contact pass, and a floor set anywhere above zero would
refuse the work this cell exists to do. That is asserted below, because it is
exactly the thing a later "safety" change would get wrong.

Runs on the HOST with no ROS. motion.py imports rclpy and half of MoveIt, so the
function is lifted out of its source with ast rather than imported, the same way
test_joint_move_duration.py and test_wrist3_band.py do it.
"""
import ast
import math
import os

import pytest

MOTION_PY = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         "..", "dgpython_rosmotion", "motion.py")


def _source():
    return open(MOTION_PY).read()


def _check_scalings():
    """Lift check_scalings out of motion.py and return it."""
    tree = ast.parse(_source(), filename="motion.py")
    fn = None
    for node in tree.body:          # module level only; not a method
        if isinstance(node, ast.FunctionDef) and node.name == "check_scalings":
            fn = node
    assert fn is not None, (
        "check_scalings is gone from motion.py, or is no longer a module-level "
        "function. Every move method depends on it.")
    mod = ast.Module(body=[fn], type_ignores=[])
    ns = {"math": math}
    exec(compile(ast.fix_missing_locations(mod), "motion.py", "exec"), ns)
    return ns["check_scalings"]


# ---------------------------------------------------------------------------
# What must be refused
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("vel,acc", [
    (0.0, 0.1),          # THE one that matters: MoveIt reads this as 1.0
    (0.1, 0.0),
    (0.0, 0.0),
    (-0.1, 0.1),
    (0.1, -0.1),
    (1.1, 0.1),          # above one is also substituted with 1.0
    (0.1, 1.1),
    (float("nan"), 0.1),
    (0.1, float("nan")),
    (float("inf"), 0.1),
    (None, 0.1),         # the shape an unset dict lookup takes
    (0.1, None),
    ("fast", 0.1),       # a string that is not a number at all
])
def test_refused(vel, acc):
    check = _check_scalings()
    with pytest.raises(ValueError):
        check(vel, acc, "test")


def test_zero_is_refused_and_the_message_says_why():
    """The message has to name full speed, or nobody reading it will act."""
    check = _check_scalings()
    with pytest.raises(ValueError) as exc:
        check(0.0, 0.1, "CartesianMove")
    text = str(exc.value)
    assert "CartesianMove" in text, "the message must say which call was wrong"
    assert "vel" in text
    assert "FULL SPEED" in text, (
        "the message must say what MoveIt would have done instead, which is the "
        "only reason this check exists")


def test_nan_is_refused_rather_than_passing_a_range_test():
    """NaN compares false both ways, so a bare range test lets it through."""
    check = _check_scalings()
    nan = float("nan")
    assert not (nan > 1.0) and not (nan <= 0.0), (
        "this test's premise is that a naive range check cannot see NaN")
    with pytest.raises(ValueError):
        check(nan, nan, "test")


# ---------------------------------------------------------------------------
# What must be allowed -- the real scan classes
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("scaling", [
    0.00254,     # measured, 6 mm/s contact pass
    0.00473,     # measured, 10 mm/s eddy-current pass
    0.00630,
    0.01649,
    0.002,       # JOINT_SCALING_FLOOR
    0.05, 0.1, 0.5,
    1.0,         # the inclusive upper end
])
def test_real_scan_scalings_pass(scaling):
    check = _check_scalings()
    check(scaling, scaling, "test")          # must not raise


def test_there_is_no_lower_bound():
    """A floor above zero would refuse the work this cell exists to do.

    Asserted explicitly because a later well-meant "surely nothing should be
    this slow" change is exactly how a 6 mm/s contact pass stops being
    commandable. The low-scaling problem is MoveIt's numerical one and is
    handled by the planner's TOTG retry ladder, not by refusing the request.
    """
    check = _check_scalings()
    check(1e-6, 1e-6, "test")
    check(1e-12, 1e-12, "test")


def test_integers_and_strings_that_are_numbers_are_accepted():
    """float() is the coercion, so a 1 or a "0.5" is a valid scaling."""
    check = _check_scalings()
    check(1, 1, "test")
    check("0.5", "0.5", "test")


# ---------------------------------------------------------------------------
# Nothing may reach MoveIt without going through it
# ---------------------------------------------------------------------------

#: Every method that puts a scaling into an action goal. Keep this in step with
#: motion.py: a new one that forgets the check is the bug this test exists for.
GOAL_SENDING_METHODS = ("CartesianMove", "FreeMove", "JointMove", "Move")


@pytest.mark.parametrize("method", GOAL_SENDING_METHODS)
def test_every_goal_sender_checks_its_scalings(method):
    tree = ast.parse(_source(), filename="motion.py")
    fn = None
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == method:
            fn = node
            break
    assert fn is not None, "%s is gone from motion.py" % method
    called = any(
        isinstance(n, ast.Call) and isinstance(n.func, ast.Name)
        and n.func.id == "check_scalings"
        for n in ast.walk(fn))
    assert called, (
        "%s assigns a velocity scaling into an action goal without calling "
        "check_scalings first, so a 0.0 from an unset field reaches MoveIt and "
        "becomes full speed" % method)


def test_the_list_of_goal_senders_is_still_complete():
    """Catch a NEW method that sets goal.velocity and was never added above.

    Without this, the parametrised test passes while the new method goes
    unchecked -- the test would be guarding a list rather than the code.
    """
    tree = ast.parse(_source(), filename="motion.py")
    senders = set()
    for node in ast.walk(tree):
        if not isinstance(node, ast.FunctionDef):
            continue
        for n in ast.walk(node):
            if (isinstance(n, ast.Attribute) and n.attr == "velocity"
                    and isinstance(n.ctx, ast.Store)):
                senders.add(node.name)
    unknown = senders - set(GOAL_SENDING_METHODS)
    assert not unknown, (
        "these methods assign a goal velocity but are not in "
        "GOAL_SENDING_METHODS, so nothing checks that they validate it: %s"
        % sorted(unknown))
