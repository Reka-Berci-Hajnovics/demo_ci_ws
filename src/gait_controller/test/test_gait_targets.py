from gait_controller.gait_targets import (
    RAW_TARGETS,
    STATE_DIRECTION,
    STATE_NAMES,
    build_targets,
)


def test_build_targets_contains_all_frequencies():
    targets = build_targets(RAW_TARGETS)

    assert set(targets.keys()) == {5, 10, 15, 20, 25}


def test_build_targets_cu_10hz():
    targets = build_targets(RAW_TARGETS)

    assert targets[10]["cu"]["in"] == (10, "decrease")
    assert targets[10]["cu"]["out"] == (2555, "increase")
    assert targets[10]["cu"]["door_open"] == (4000, "increase")


def test_build_targets_cp_steps():
    targets = build_targets(RAW_TARGETS)

    assert targets[10]["cp"]["cp_step1"] == (1480, "increase")
    assert targets[10]["cp"]["cp_step2"] == (1860, "increase")
    assert targets[10]["cp"]["cp_step3"] == (2240, "increase")
    assert targets[10]["cp"]["cp_step4"] == (2620, "increase")


def test_none_targets_are_excluded():
    targets = build_targets(RAW_TARGETS)

    assert "cp_step1" not in targets[10]["cu"]
    assert "cp_step2" not in targets[10]["cu"]
    assert "cp_step1" not in targets[10]["cl"]


def test_all_runtime_states_have_directions():
    for state in STATE_NAMES:
        assert state in STATE_DIRECTION
