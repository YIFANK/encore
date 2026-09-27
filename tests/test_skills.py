

def test_every_skill_parameter_can_be_expressed_by_the_planner():
    """A closed plan schema silently drops arguments it has never heard of.

    Measured: the planner reasoned its way to lift_and_peek(cup, block) on the
    hide-and-seek scene and the executor received an empty argument list,
    because ArgsOut had no `occluder` field. Adding a skill without adding its
    parameters here makes the skill unusable however well the model plans.
    """
    from heron.orchestrator.gemini import ArgsOut
    from heron.skills import load_all

    known = set(ArgsOut.model_fields)
    missing = {}
    for name in load_all().names():
        params = set(load_all().get(name).params)
        gap = params - known
        if gap:
            missing[name] = sorted(gap)
    assert not missing, f"plan schema cannot express: {missing}"
