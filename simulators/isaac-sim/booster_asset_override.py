from __future__ import annotations

import isaaclab.sim as sim_utils
from isaaclab.assets.articulation import ArticulationCfg

from booster_assets import BOOSTER_ASSETS_DIR
from booster_k1_sim.assets.actuator import DelayedImplicitActuatorCfg

ARMATURE_6416 = 0.095625
ARMATURE_4310 = 0.0282528
ARMATURE_6408 = 0.0478125
ARMATURE_4315 = 0.0339552
ARMATURE_ROB_14 = 0.001

STIFFNESS_6416 = 80.0
STIFFNESS_4310 = 80.0
STIFFNESS_6408 = 80.0
STIFFNESS_4315 = 80.0
STIFFNESS_ROB_14 = 4.0

DAMPING_6416 = 2.0
DAMPING_4310 = 2.0
DAMPING_6408 = 2.0
DAMPING_4315 = 2.0
DAMPING_ROB_14 = 1.0


def _k1_standing_init_state() -> ArticulationCfg.InitialStateCfg:
    return ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.57),
        joint_pos={
            "Left_Shoulder_Roll": -1.3,
            "Right_Shoulder_Roll": 1.3,
            ".*_Hip_Pitch": -0.2,
            ".*_Knee_Pitch": 0.4,
            ".*_Ankle_Pitch": -0.25,
        },
        joint_vel={".*": 0.0},
    )


def _build_k1_actuators() -> dict[str, DelayedImplicitActuatorCfg]:
    return {
        "legs": DelayedImplicitActuatorCfg(
            min_delay=0,
            max_delay=0,
            joint_names_expr=[
                ".*_Hip_Pitch",
                ".*_Hip_Roll",
                ".*_Hip_Yaw",
                ".*_Knee_Pitch",
            ],
            effort_limit_sim={
                ".*_Hip_Pitch": 30.0,
                ".*_Hip_Roll": 35.0,
                ".*_Hip_Yaw": 20.0,
                ".*_Knee_Pitch": 40.0,
            },
            velocity_limit_sim={
                ".*_Hip_Pitch": 8.0,
                ".*_Hip_Roll": 12.9,
                ".*_Hip_Yaw": 18.0,
                ".*_Knee_Pitch": 12.5,
            },
            stiffness={
                ".*_Hip_Pitch": STIFFNESS_6408,
                ".*_Hip_Roll": STIFFNESS_4315,
                ".*_Hip_Yaw": STIFFNESS_4310,
                ".*_Knee_Pitch": STIFFNESS_6416,
            },
            damping={
                ".*_Hip_Pitch": DAMPING_6408,
                ".*_Hip_Roll": DAMPING_4315,
                ".*_Hip_Yaw": DAMPING_4310,
                ".*_Knee_Pitch": DAMPING_6416,
            },
            armature={
                ".*_Hip_Pitch": ARMATURE_6408,
                ".*_Hip_Roll": ARMATURE_4315,
                ".*_Hip_Yaw": ARMATURE_4310,
                ".*_Knee_Pitch": ARMATURE_6416,
            },
        ),
        "feet": DelayedImplicitActuatorCfg(
            min_delay=0,
            max_delay=0,
            effort_limit_sim=20.0,
            velocity_limit_sim=18.0,
            joint_names_expr=[".*_Ankle_Pitch", ".*_Ankle_Roll"],
            stiffness=30.0,
            damping=2.0,
            armature=2.0 * ARMATURE_4310,
        ),
        "arms": DelayedImplicitActuatorCfg(
            min_delay=0,
            max_delay=0,
            joint_names_expr=[
                ".*_Shoulder_Pitch",
                ".*_Shoulder_Roll",
                ".*_Elbow_Pitch",
                ".*_Elbow_Yaw",
            ],
            effort_limit_sim=14.0,
            velocity_limit_sim=18.0,
            stiffness=STIFFNESS_ROB_14,
            damping=DAMPING_ROB_14,
            armature=ARMATURE_ROB_14,
        ),
        "head": DelayedImplicitActuatorCfg(
            min_delay=0,
            max_delay=0,
            joint_names_expr=[".*Head.*"],
            effort_limit_sim=6.0,
            velocity_limit_sim=20.0,
            stiffness=4.0,
            damping=1.0,
            armature=0.001,
        ),
    }


def _build_k1_cfg(asset_path: str) -> ArticulationCfg:
    return ArticulationCfg(
        spawn=sim_utils.UrdfFileCfg(
            fix_base=False,
            replace_cylinders_with_capsules=False,
            asset_path=asset_path,
            activate_contact_sensors=True,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False,
                retain_accelerations=False,
                linear_damping=0.0,
                angular_damping=0.0,
                max_linear_velocity=1000.0,
                max_angular_velocity=1000.0,
                max_depenetration_velocity=1.0,
            ),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=True,
                solver_position_iteration_count=8,
                solver_velocity_iteration_count=4,
            ),
            joint_drive=sim_utils.UrdfConverterCfg.JointDriveCfg(
                gains=sim_utils.UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=0, damping=0)
            ),
        ),
        init_state=_k1_standing_init_state(),
        soft_joint_pos_limit_factor=0.9,
        actuators=_build_k1_actuators(),
    )


BOOSTER_K1_CFG = _build_k1_cfg(f"{BOOSTER_ASSETS_DIR}/robots/K1/K1_22dof.urdf")
BOOSTER_K1_ZED_CFG = _build_k1_cfg(f"{BOOSTER_ASSETS_DIR}/robots/K1/K1_22dof-ZED.urdf")
