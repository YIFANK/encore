"""Placeholder: the real Model is RoboDojoBridge inside tools/fair_run_robodojo.py.
Kept so `robodojo.sh doctor` finds the policy directory well-formed."""
from XPolicyLab.model_template import ModelTemplate


class Model(ModelTemplate):
    def __init__(self, model_cfg):
        raise RuntimeError("Encore is served by tools/fair_run_robodojo.py, not setup_policy_server.py")
