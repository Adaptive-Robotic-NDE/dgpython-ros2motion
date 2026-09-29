"""The wrist_3 cable band ApproachInConfiguration() enforces.

Runs on the HOST with no ROS: motion.py imports rclpy and half of MoveIt, so the
two pure helpers are lifted out of its source with ast rather than imported.
That is deliberate -- the arithmetic is what decides whether a pass is refused,
and it should be checkable without a container, a robot or a planning scene.

The angles checked below are ones an Azure pan sweep commands at wrist_3 --
29.3, 21.9, -32.3, -55.1 and -113.0 degrees -- against a measured cable band of
31..301 degrees. None of them is inside it, and a sweep that flies them has to
be stopped by hand, so every one must be refused.

THE BAND IS AN ABSOLUTE JOINT INTERVAL and is not read modulo a turn. +247 and
-113 degrees are the same tool orientation and are not the same place for a
cable, because one has a turn wound into it that the other has not. That is the
rule ROS-Scanning's cable_envelope.dpi applies, and this file exists partly to
hold motion.py to it: if the two disagree, a plan is checked against one meaning
and flown against the other.
"""
import ast
import math
import os

MOTION_PY = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         "..", "dgpython_rosmotion", "motion.py")

BAND = (math.radians(31.0), math.radians(301.0))


def _helpers():
    """_wrist3_span_in_band and _wrist3_band_centre, lifted out of motion.py."""
    tree = ast.parse(open(MOTION_PY).read())
    want = ("_wrist3_span_in_band", "_wrist3_band_centre")
    body = [n for cls in ast.walk(tree)
            if isinstance(cls, ast.ClassDef)
            for n in cls.body
            if isinstance(n, ast.FunctionDef) and n.name in want]
    assert len(body) == 2, "motion.py no longer defines %s" % (want,)
    for n in body:                       # drop @staticmethod, call them plainly
        n.decorator_list = []
    ns = {"math": math}
    exec(compile(ast.fix_missing_locations(ast.Module(body=body,
                                                      type_ignores=[])),
                 "motion.py", "exec"), ns)
    return ns[want[0]], ns[want[1]]


in_band, centre = _helpers()


def at(deg):
    return math.radians(deg)


def test_every_commanded_roll_of_the_stopped_sweep_is_refused():
    for d in (29.3, 21.9, -32.3, -55.1, -113.0):
        assert not in_band(at(d), at(d), BAND), d


def test_the_band_is_absolute_and_not_read_modulo_a_turn():
    assert in_band(at(247.0), at(247.0), BAND)
    # The same orientation one turn either way is a different cable state.
    assert not in_band(at(247.0 - 360.0), at(247.0 - 360.0), BAND)
    assert not in_band(at(247.0 + 360.0), at(247.0 + 360.0), BAND)


def test_the_ends_of_the_band_are_inside_it():
    assert in_band(at(31.0), at(31.0), BAND)
    assert in_band(at(301.0), at(301.0), BAND)
    assert not in_band(at(30.0), at(30.0), BAND)
    assert not in_band(at(302.0), at(302.0), BAND)


def test_the_whole_sweep_is_checked_not_only_its_ends():
    # Both ends inside, the middle inside too.
    assert in_band(at(50.0), at(280.0), BAND)
    # An end outside is refused even when the other end is comfortable.
    assert not in_band(at(166.0), at(-10.0), BAND)
    assert not in_band(at(166.0), at(400.0), BAND)


def test_no_band_is_no_constraint():
    assert in_band(at(-720.0), at(720.0), None)


def test_the_centre_is_the_middle_of_the_band():
    assert abs(math.degrees(centre(BAND)) - 166.0) < 1e-9
    assert in_band(centre(BAND), centre(BAND), BAND)
