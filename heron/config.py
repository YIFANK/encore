"""Configuration: one YAML file describes the rig; env vars override secrets/model ids."""
from __future__ import annotations

import os
from pathlib import Path
from typing import Literal, Optional

import yaml
from pydantic import BaseModel, Field


def _load_dotenv() -> None:
    """Load KEY=VALUE lines from ./.env or the repo-root .env (secrets like
    GEMINI_API_KEY live there, git-ignored). Existing env vars win."""
    for candidate in (Path.cwd() / ".env", Path(__file__).resolve().parents[1] / ".env"):
        if not candidate.is_file():
            continue
        for line in candidate.read_text().splitlines():
            line = line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            key, _, value = line.partition("=")
            os.environ.setdefault(key.strip(), value.strip().strip("'\""))


_load_dotenv()

# Single place to bump when Google ships a new ER release.
# gemini-robotics-er-2-preview released 2026-07-30; ER 1.6 shuts down 2026-08-31.
DEFAULT_ER_MODEL = os.environ.get("HERON_ER_MODEL", "gemini-robotics-er-2-preview")


class ArmCfg(BaseModel):
    ip: str = ""
    # 4x4 world->this-arm-base transform. The stationary kit mounts the two arms
    # facing each other across the table; defaults live in configs/stationary.yaml.
    t_world_base: list[list[float]] = Field(
        default_factory=lambda: [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
    )
    # Angle-axis wrist orientation used for table-facing motion. The ready pose
    # holds the gripper HORIZONTAL, which sweeps the wrist into the table (and
    # into the rail on this rig) on any descent, so table work must command its
    # own orientation rather than inherit whatever home() left behind.
    # wxai_v0: tool +x is the approach axis, so R_y(90 deg) points it straight down.
    approach_rvec: list[float] = Field(default_factory=lambda: [0.0, 1.5708, 0.0])
    # Mass bolted to the wrist that the driver's standard end-effector model
    # does not know about (D405 + mount + cable pull). It feeds the
    # controller's gravity feed-forward: unmodelled mass under-compensates,
    # and the shortfall shows up as reach-dependent droop and the occasional
    # float-and-settle. Applied to the palm link at ee_extra_com_m (palm
    # frame) via set_end_effector at connect time.
    ee_extra_mass_kg: float = 0.0
    ee_extra_com_m: list[float] = Field(default_factory=lambda: [0.05, 0.0, 0.05])


class CameraCfg(BaseModel):
    serial: str = ""
    # UVC/AVFoundation device index — set this to use the OpenCV backend
    # (RGB-only; the lab Mac cannot open D405 depth). None -> RealSense SDK.
    index: Optional[int] = None
    # Sidecar depth server URL (tools/depth_server.py) — takes precedence over
    # index/serial for this camera and delivers aligned RGB-D.
    depth_server: Optional[str] = None
    kind: str = "overhead"  # overhead | front | wrist
    arm: Optional[str] = None  # for wrist cams
    extrinsics_file: Optional[str] = None  # .npz with t_world_cam (from `heron calibrate`)
    # Depth-free grounding: pixel->world-XY homography on the table plane
    # (from `heron calibrate --planar`).
    homography_file: Optional[str] = None
    # Depth-free 3D: 3x4 world->pixel projection matrix (from
    # `heron calibrate --projection`). Two calibrated cameras triangulate.
    projection_file: Optional[str] = None
    # Wrist cams: fixed camera-in-end-effector transform (hand-eye), 4x4.
    t_ee_cam: Optional[list[list[float]]] = None


class WorkspaceCfg(BaseModel):
    # Fair-stack envelope consumed by RigRobot._guard: an axis-aligned box in
    # the WORLD frame (meters) that every commanded target must fall inside,
    # for either arm. Deliberately separate from SafetyCfg.workspace_* — the
    # skill/primitive layer's box is sized for single-arm tabletop work, while
    # this one must admit whatever the demonstrations themselves did (the
    # rig_handover pack holds the block at z=0.277, above workspace_z's 0.18).
    min_xyz: Optional[list[float]] = None
    max_xyz: Optional[list[float]] = None
    # External-effort cap (N) checked before every motion command.
    max_effort_n: float = 25.0


class SafetyCfg(BaseModel):
    # Axis-aligned workspace box in the WORLD frame (meters). Motions outside are rejected.
    workspace_x: tuple[float, float] = (-0.25, 0.30)
    workspace_y: tuple[float, float] = (-0.35, 0.35)
    workspace_z: tuple[float, float] = (0.005, 0.45)
    max_step_m: float = 0.35  # largest single cartesian displacement allowed
    min_move_seconds: float = 1.0  # lower bound on commanded goal time (a zero-length move still takes this)
    # Tool speed ceiling, m/s of commanded goal time. Unset (0) keeps the old
    # rule, where min_move_seconds capped the speed by stretching EVERY segment
    # to the same duration — so a 50 mm descent cost as much as a 300 mm
    # traverse. Set it and each segment is charged for the distance it covers.
    max_tool_speed_mps: float = 0.0
    # Global slow-down: every commanded goal time is divided by this, so 0.6
    # means "60% speed". Lower it whenever the rig feels twitchy to an operator
    # standing next to it — reaction time is the real constraint, not the motors.
    speed_scale: float = 1.0
    gripper_effort_limit: float = 5.0
    confirm_actions: bool = False  # ask y/n in the terminal before each motion
    dry_run: bool = False  # log motions instead of executing


class BudgetCfg(BaseModel):
    # THE CLOCK IS THE BUDGET. Counts below are pathological-loop backstops,
    # set far above anything a healthy episode spends: search behaviours
    # (lift cup, look, restore, next) are many steps and many honest retries,
    # and the old tight counts (40/12/3) kept aborting exactly the episodes
    # that were making progress. wall_clock_s is what actually ends a run.
    max_steps_executed: int = 200
    max_edits: int = 100
    max_repairs_per_step: int = 8  # local repairs on one step before escalating
    # An UNCHANGED failure is a wall, and walls are detected fast: this many
    # identical reports trigger a full replan (cheap, genuinely changes the
    # approach); the same wall after the replan aborts.
    max_same_failure: int = 3
    wall_clock_s: float = 1800.0
    verify_max_age_s: float = 25.0  # belief older than this must be re-verified
    llm_timeout_s: float = 90.0


class ModelCfg(BaseModel):
    er_model: str = DEFAULT_ER_MODEL
    # A relay to send the model traffic through, and the env var holding its
    # key. Both empty means Google directly via GEMINI_API_KEY. The reason to
    # set them is price: grounding at minimal thinking is the bulk of every
    # episode, and a flash-tier model behind a relay answers it for a fraction
    # of ER's preview pricing.
    base_url: str = ""
    # The URL can come from the environment instead, so a per-deployment relay
    # address never lands in a committed YAML. `base_url` wins if both are set.
    base_url_env: str = ""
    api_key_env: str = ""
    # What protocol the endpoint speaks. "gemini" is google-genai against the
    # base_url (Google itself, or a Gemini-format relay); "openai" is
    # POST {base_url}/chat/completions with Bearer auth — the shape most
    # commercial relays actually serve.
    api_format: Literal["gemini", "openai"] = "gemini"

    def resolved_base_url(self) -> str:
        if self.base_url:
            return self.base_url
        return os.environ.get(self.base_url_env, "") if self.base_url_env else ""
    # Deliberation is cost-tiered via ER 2 thinking levels
    # (minimal | low | medium | high): spatial queries stay fast, plan/repair think.
    # Persistent Live (bidi) session for grounding/VQA roles. Measured:
    # 1-2 s per answer on a session that connects once, vs 2-3 s REST calls.
    live: bool = False
    live_model: str = "gemini-robotics-er-2-streaming-preview"
    thinking_ground: str = "minimal"
    thinking_verify: str = "low"
    thinking_plan: str = "low"
    thinking_repair: str = "low"


class LiberoCfg(BaseModel):
    suite: str = "libero_goal"
    task_id: int = 0
    episode: int = 0
    seed: int = 0
    image_size: int = 512
    # LIBERO-PRO evaluation: a perturbed problem file (and its matching init
    # states) generated by that benchmark, used INSTEAD of the stock task. The
    # language instruction travels with the bddl, so it must be passed too —
    # perturbed suites rewrite the instruction, which is half of the test.
    bddl_file: Optional[str] = None
    init_states_file: Optional[str] = None
    language: Optional[str] = None
    # Record one frame every N simulator steps for the episode GIF (0 = off).
    record_every: int = 5
    # Override the env's step horizon. None keeps LIBERO's own (the honest
    # setting for benchmark numbers); experiments that use the env as a data
    # factory may extend it — an augmentation rollout that needs 2400 steps
    # to finish is still a demonstration worth keeping.
    horizon: Optional[int] = None
    # Strip the robot's materials and paint it plain gray. This renderer
    # fails to load the arm's textures (green noise where the demos' own
    # films show a white Panda); flat gray is honest and legible. Off by
    # default: datasets already collected carry the green arm, and a policy
    # must be evaluated on the pixels it trained on.
    plain_robot: bool = False
    film_size: int = 0        # film-strip render size; 0 = reuse the policy's observation
    plain_table: bool = False


DEFAULT_EMBODIMENT = (
    "a bimanual stationary robot (two WidowX AI arms facing each other across a table, four "
    "RGB-D cameras: cam_high overhead, cam_low front, one wrist camera per arm). World frame: "
    "origin at table center, x forward, y left, z up, table top z=0. The left arm serves the "
    "y>0 half, the right arm the y<0 half; always set arm explicitly in pick/place args and "
    "use the same arm name in holding/gripper_empty predicates."
)


class HeronConfig(BaseModel):
    mode: str = "mock"  # mock | real | sim | libero
    arms: dict[str, ArmCfg] = Field(default_factory=lambda: {"left": ArmCfg(), "right": ArmCfg()})
    cameras: dict[str, CameraCfg] = Field(
        default_factory=lambda: {
            "cam_high": CameraCfg(kind="overhead"),
            "cam_low": CameraCfg(kind="front"),
            "cam_left_wrist": CameraCfg(kind="wrist", arm="left"),
            "cam_right_wrist": CameraCfg(kind="wrist", arm="right"),
        }
    )
    safety: SafetyCfg = Field(default_factory=SafetyCfg)
    # Fair-stack (RigRobot) workspace guard; None leaves the guard off and
    # only the effort cap active. See WorkspaceCfg.
    workspace: Optional[WorkspaceCfg] = None
    budgets: BudgetCfg = Field(default_factory=BudgetCfg)
    models: ModelCfg = Field(default_factory=ModelCfg)
    # World-frame height of the table surface (meters). Depth-free grounding
    # places every table-resting object on this plane.
    table_z: float = 0.0
    # Grasp/place height strategy: feel for the surface via external effort.
    # Disable on backends without a contact signal (LIBERO).
    contact_descend: bool = True
    # Whether PLACE feels for the surface even when pick does not. Depth quality
    # is a property of the surface, not of the rig: the block reads cleanly
    # (26 mm, plausible) while the glossy plate scatters badly — 25.5 mm of MAD
    # inside its mask, with points 84 mm BELOW the table. Descending onto that
    # by depth alone is how an object gets dropped from the wrong height or
    # pressed into the plate. None = follow contact_descend.
    # Added to every grasp descent target (metres, negative = deeper). A rig
    # whose fingers visibly stop short of the block's midline gets tuned here,
    # one yaml line, instead of by editing GRASP_DEPTH for every backend.
    grasp_z_trim_m: float = 0.0
    # Per-arm world-frame XY added to grasp/place aims: the measured offset
    # between where an arm is COMMANDED and where its fingertips LAND (the
    # left arm reads [-13.3, -3.0] mm on this rig, fingertip-camera closed
    # loop 2026-08-18 — a kinematic/servo property the base calibration
    # cannot absorb). {"left": [0.0133, 0.003]} counter-commands it.
    aim_trim_xy_m: dict[str, list[float]] = Field(default_factory=dict)
    # Reach-dependent fingertip sag compensation (rig LAWS L14): under the
    # gripper's weight the fingertips land SHORT radially — ~16 mm at
    # r=0.42-0.47 from the base, unmeasurable inside the comfort band, BOTH
    # arms. Deterministic: aim past the target by k*(r - r0) radially
    # outward. 0 disables.
    sag_comp_m_per_m: float = 0.0
    sag_comp_r0_m: float = 0.30
    # Arms whose grasp aim is closed through the overhead camera before the
    # descend (air-pinch, fingertip detection, ray projection — no model
    # calls, ~8 s per pick). The successor to aim_trim_xy_m where the
    # fingertip offset is position-dependent.
    pinch_refine_arms: list[str] = Field(default_factory=list)
    # Arms the agent may command. A dual-arm rig running single-arm (the
    # default operating mode: right arm does everything, left is parked)
    # sets ["right"]: the planner cannot assign work or goals to an arm the
    # system is not sensing. None = all arms the robot reports.
    active_arms: Optional[list[str]] = None
    # "step": verify pre/post visually at every step (the teaching standard —
    # only verified successes may teach). "terminal": steps trust proprio and
    # the in-skill guards; the camera is consulted once, at the goal. Half the
    # wall clock, at the cost of later failure discovery.
    verify_mode: str = "step"
    # "point": ER points, SAM refines (default). "proposal": SAM segments
    # everything, ER answers multiple-choice — precision from the mask, colour
    # judgement from a numbered-crop question. A/B in the twin (2026-08-03,
    # tools/grounding_ab.py): point median 2.3 mm / p90 100 mm, proposal
    # median 4.9 mm / p90 117 mm with unstable match rate — twin depth masks
    # already carry point mode. Built for the RIG, where homography
    # extrapolation and the specular plate are the tail; flip it there.
    # Where the reset puts the block back, when a fixed start is wanted.
    # None keeps the sampled layout, which spreads the training distribution;
    # a pair pins every episode to the same start, which is what a small
    # dataset and a demo both need.
    collect_reset_xy: Optional[list] = None
    grounding_mode: str = "point"
    # Closed-loop grasp: at the hover, re-ground the target through the WRIST
    # camera (hand-eye calibrated, 0.1-0.2 m from the object) and correct the
    # grasp xy. Absorbs the overhead camera's open-loop error — homography
    # extrapolation, table flex, remounts — which no periodic recalibration
    # keeps below fingertip tolerance for long. Costs 1-3 model calls per pick.
    wrist_refine: bool = False
    # Mirror the controller-FK residual into low descents (see
    # TrossenStationary._droop_feedforward). Only for rigs whose FK is honest
    # under load — on rig5090 the -y half's link flex makes FK lie, so this
    # stays off there and wrist_refine carries the correction instead.
    droop_feedforward: bool = False
    # Operator dialogue: after planning, narrate what the agent sees and
    # intends, wait for Enter (release) or a typed correction (replan under
    # it). `speak` reads the briefing aloud via macOS `say`. Rig-facing
    # configs turn these on; batch/twin runs leave them off.
    interactive: bool = False
    speak: bool = False
    contact_descend_place: Optional[bool] = None
    # One-paragraph robot description injected into the planner prompt.
    embodiment: str = DEFAULT_EMBODIMENT
    libero: LiberoCfg = Field(default_factory=LiberoCfg)
    # MuJoCo scene for mode=sim. Built by tools/build_sim_scene.py from the same
    # URDF and calibration files the hardware uses.
    sim_scene: Optional[str] = None
    # Camera the simulation records from, and how often. A run that cannot be
    # watched is a journal entry, not evidence.
    sim_record_camera: Optional[str] = "cam_high"
    sim_record_every: int = 40
    # TiPToP tamp_server URL (Pigey protocol: /pick /drop_above /perceive) — enables
    # the tamp_pick/tamp_place heavy-pipeline skills when set.
    tamp_url: Optional[str] = None
    # SAM2 mask service (tools/sam2_server.py). Unset means grounding works off
    # detection boxes alone, exactly as before — masks are an upgrade, not a
    # dependency.
    sam_url: Optional[str] = None
    # openpi policy server (scripts/serve_policy.py) serving pi05_libero over a
    # websocket. Set only in simulation: the backend drives the LIBERO
    # environment directly, because the policy emits actions in that
    # environment's own space.
    policy_url: Optional[str] = None
    # A locally-trained checkpoint (tools/train_local.py), driven in-process
    # against the twin. Takes precedence over policy_url: it is the loop's own
    # policy, and there is nothing for a server to add between a directory on
    # this disk and the simulator reading it.
    policy_checkpoint: Optional[str] = None
    # Where validated corrections live (heron/patches.py). Unset = no store,
    # every lookup returns its in-code default.
    patches_path: Optional[str] = "memory_store/patches.jsonl"
    # What "on the support" means. "centred" is the benchmark's criterion —
    # centres within ON_XY_MAX (LIBERO's own On() is 30 mm) — and stays the
    # default so simulated scores keep meaning what the benchmark means.
    # "support" is the physical criterion for the real cell: resting anywhere on
    # the support counts, measured against the support's own footprint. The
    # first hardware episode landed the block 47 mm from the plate's centre —
    # ON the plate, plainly, to the person standing there — and was scored a
    # failure by a tolerance that belongs to an exam it was not sitting.
    on_criterion: str = "centred"
    # THE STRONG LOW LEVEL DOES THE MANIPULATION; the planner orchestrates it.
    # Pigey's whole architecture is an orchestrator over TiPToP + pi0.5, and a
    # system claiming to be strictly stronger cannot stand on a weaker executor.
    # Measured on LIBERO-PRO with vla as a repair-time fallback: 24 invocations,
    # and on every spatial/goal episode it arrived AFTER the scripted primitives
    # had burned the budget — "ran 0.0s, stopped by environment ended". The
    # object cells it did reach in time, it saved. True = the planner is told to
    # delegate manipulation to `vla` and keep the primitives for perception and
    # for recovery — inverting the old order.
    vla_primary: bool = False
    # Scoped routing: verbs whose steps the planner is told to hand to `vla`,
    # with everything else staying on the scripted primitives. The blanket
    # inversion above was measured WORSE than grounding-first (3/18 vs 6/18 —
    # the policy follows perturbed instruction text into swap traps), but the
    # articulation verbs are exactly where the primitives have never once
    # succeeded on LIBERO-PRO, so the trade is different there: e.g.
    # ["open", "close", "push"]. Ignored when vla_primary already routes
    # everything.
    vla_verbs: list[str] = Field(default_factory=list)
    # M2T2 grasp-proposal service (tools/m2t2_service.py). Unset means pick
    # keeps its heuristic grasp point; set, pick consults the service and falls
    # back to the heuristic on any failure or timeout — a proposal is an
    # upgrade, never a dependency, and a dead tunnel must never block the rig.
    grasp_service_url: Optional[str] = None
    grasp_service_timeout_s: float = 2.0
    # Skills to hide from the planner and refuse at execution (per-deployment).
    disable_skills: list[str] = Field(default_factory=list)
    episodes_dir: str = "episodes"
    memory_dir: str = "memory_store"
    skills_dir: str = "skill_library"  # learned, agent-authored skills

    @classmethod
    def load(cls, path: str | Path | None = None) -> "HeronConfig":
        if path is None:
            return cls()
        data = yaml.safe_load(Path(path).read_text()) or {}
        cfg = cls.model_validate(data)
        # A deployment with no wrist camera cannot inspect: the skill would be
        # scheduled, fail with "no wrist camera", and stall verification (rig
        # 2026-08-03, broken wrist cable). Disable it up front so the planner
        # never offers it and verification falls back to geometry + VQA.
        has_wrist = any("wrist" in name for name in (cfg.cameras or {}))
        if not has_wrist and "inspect" not in cfg.disable_skills:
            cfg.disable_skills = [*cfg.disable_skills, "inspect"]
        return cfg
