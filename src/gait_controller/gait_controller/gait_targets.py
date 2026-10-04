# Target positions per axis/state -> compact position tuples.
RAW_TARGETS = {
    5: {
        "cu": (10, 3500, 2555, 2555, 4000, None, None, None, None),
        "cl": (10, 3500, 2355, 2355, 4000, None, None, None, None),
        "cp": (1100, None, None, 3000, None, 1480, 1860, 2240, 2620),
    },

    10: {
        "cu": (10, 3500, 2555, 2555, 4000, None, None, None, None),
        "cl": (10, 3500, 2355, 2355, 4000, None, None, None, None),
        "cp": (1100, None, None, 3000, None, 1480, 1860, 2240, 2620),
    },

    15: {
        "cu": (10, 3738, 2670, 2463, 4000, None, None, None, None),
        "cl": (10, 3662, 2482, 2261, 4000, None, None, None, None),
        "cp": (1100, None, None, 3000, None, 1480, 1860, 2240, 2620),
    },

    20: {
        "cu": (10, 3500, 2555, 2555, 4000, None, None, None, None),
        "cl": (10, 3500, 2355, 2355, 4000, None, None, None, None),
        "cp": (1100, None, None, 3000, None, 1480, 1860, 2240, 2620),
    },

    25: {
        "cu": (10, 3500, 2555, 2555, 4000, None, None, None, None),
        "cl": (10, 3500, 2355, 2355, 4000, None, None, None, None),
        "cp": (1100, None, None, 3000, None, 1480, 1860, 2240, 2620),
    },
}


STATE_NAMES = (
    "in",
    "door_intermediate_close",
    "door_close",
    "out",
    "door_open",
    "cp_step1",
    "cp_step2",
    "cp_step3",
    "cp_step4",
)


STATE_DIRECTION = {
    "in": "decrease",
    "door_intermediate_close": "decrease",
    "door_close": "decrease",
    "out": "increase",
    "door_open": "increase",
    "cp_step1": "increase",
    "cp_step2": "increase",
    "cp_step3": "increase",
    "cp_step4": "increase",
}


def build_targets(raw_targets):
    """
    Convert compact target configuration into the runtime lookup format.
    """
    return {
        frequency: {
            axis: {
                state: (position, STATE_DIRECTION[state])
                for state, position in zip(STATE_NAMES, positions)
                if position is not None
            }
            for axis, positions in axes.items()
        }
        for frequency, axes in raw_targets.items()
    }


TARGETS = build_targets(RAW_TARGETS)
