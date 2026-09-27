"""Encore policy: episode loop identical to XPolicyLab's demo_policy. The
policy server side is heron.robot.robodojo_env.RoboDojoBridge, hosted by
tools/fair_run_robodojo.py; this module only exists so the RoboDojo eval
client can import XPolicyLab.policy.Encore.deploy."""


def eval_one_episode(TASK_ENV, model_client):
    model_client.call(func_name="reset")
    while not TASK_ENV.is_episode_end():
        obs = TASK_ENV.get_obs()
        model_client.call(func_name="update_obs", obs=obs)
        actions = model_client.call(func_name="get_action")
        for action_idx, action in enumerate(actions):
            TASK_ENV.take_action(action)
            if TASK_ENV.is_episode_end() or action_idx + 1 == len(actions):
                break
            obs = TASK_ENV.get_obs()
            model_client.call(func_name="update_obs", obs=obs)


def eval_one_episode_batch(TASK_ENV, model_client):
    raise NotImplementedError("Encore runs one env at a time")
