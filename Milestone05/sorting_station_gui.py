#!/usr/bin/env python3
"""
Automated MuJoCo sorting station demo used by launch_sorting_station.py.

The script loads the detailed sorting_scene.xml, opens the interactive MuJoCo
viewer, and drives the UR5e arm through a full Milestone 5 style sequence where
white and black boxes are picked from the conveyor and deposited into the
corresponding bins.

MODIFICATIONS:
1. Robot moves to a vertical "Sky" pose at the start.
2. Distinct drop positions for each box to arrange them inside the bins.
"""
from __future__ import annotations

from dataclasses import dataclass
import os
import time
from typing import Dict, Iterable, List, Optional, Tuple

import mujoco
import mujoco.viewer
import numpy as np

# Adjust imports to match your package structure
from mujoco_ros2.ik_ur5e import inverse_kinematics
from mujoco_ros2.trajectory_planner import generate_joint_trajectory


def _resolve_model_path() -> str:
    """Locate sorting_scene.xml from either the source tree or the installed share directory."""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    candidate = os.path.normpath(os.path.join(script_dir, "..", "model", "sorting_scene.xml"))
    if os.path.exists(candidate):
        return candidate

    try:
        from ament_index_python.packages import get_package_share_directory

        share_dir = get_package_share_directory("mujoco_ros2")
        candidate = os.path.join(share_dir, "model", "sorting_scene.xml")
        if os.path.exists(candidate):
            return candidate
    except Exception:
        pass

    raise FileNotFoundError("sorting_scene.xml could not be located in source or install paths.")


@dataclass(frozen=True)
class BoxTask:
    body: str
    color: str


class SortingStationSimulation:
    """Encapsulates the MuJoCo viewer loop and pick-and-place logic."""

    def __init__(self, model_path: str) -> None:
        if not os.path.exists(model_path):
            raise FileNotFoundError(f"Sorting scene not found: {model_path}")

        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        mujoco.mj_resetDataKeyframe(self.model, self.data, 0)

        self.viewer: Optional[object] = None
        self.status_text = "Initializing sorting station..."

        self.control_rate = 150.0  # Hz for the planned joint-space trajectory
        
        # Standard "Down" orientation for picking/placing
        self.down_orientation = np.array([[0.0, 0.0, 1.0],
                                          [0.0, 1.0, 0.0],
                                          [-1.0, 0.0, 0.0]])
        
        # Joint Configuration for "Sky Pose" (Vertical Up)
        # [Base, Shoulder, Elbow, Wrist1, Wrist2, Wrist3]
        # Shoulder -1.57 (Horizontal), Wrist1 +1.57 (Points UP).
        self.q_sky = np.array([0.0, -1.57, 0.0, 1.57, 0.0, 0.0])

        self.hover_offset = 0.15    # meters above a box/bin during travel
        self.pick_offset = 0.015    # meters above box center for grasp
        self.place_offset = 0.03    # meters above bin base for release

        self.kp = np.array([200.0, 180.0, 140.0, 80.0, 60.0, 40.0])
        self.kd = np.array([30.0, 28.0, 22.0, 12.0, 10.0, 8.0])
        self.steps_per_sample = max(
            1, int((1.0 / self.control_rate) / self.model.opt.timestep)
        )

        self.ee_site_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_SITE, "attachment_site"
        )
        if self.ee_site_id < 0:
            raise RuntimeError("attachment_site site is missing from the model")

        self.current_q = self.data.qpos[:6].copy()
        self.attached_box: Optional[str] = None
        self.attachment_offset = np.zeros(3)
        self.attachment_quat = np.array([1.0, 0.0, 0.0, 0.0])

        # Sequence alternates white/black boxes from the staging area in front of the robot.
        self.tasks: List[BoxTask] = [
            BoxTask("white_box_1", "white"),
            BoxTask("black_box_1", "black"),
            BoxTask("white_box_2", "white"),
            BoxTask("black_box_2", "black"),
        ]
        
        # Specific drop targets for each box to arrange them neatly in the bins
        # White bin approx Y = +0.38, Black bin approx Y = -0.38
        self.drop_targets: Dict[str, np.ndarray] = {
            "white_box_1": np.array([0.20, 0.38, 0.45]), 
            "white_box_2": np.array([0.30, 0.38, 0.45]),
            "black_box_1": np.array([0.20, -0.38, 0.45]),
            "black_box_2": np.array([0.30, -0.38, 0.45]),
        }
        
        self.box_pick_targets: Dict[str, np.ndarray] = {
            "white_box_1": np.array([-0.2, 0.18, 0.52]),
            "black_box_1": np.array([-0.2, -0.18, 0.52]),
            "white_box_2": np.array([0.0, 0.18, 0.52]),
            "black_box_2": np.array([0.0, -0.18, 0.52]),
        }

        box_names = sorted(self.box_pick_targets.keys())
        self.box_joint_index: Dict[str, Tuple[int, int]] = {
            name: self._compute_freejoint_indices(name) for name in box_names
        }
        self.body_ids: Dict[str, int] = {
            name: mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
            for name in box_names
        }

    def _hold_current_pose(self, duration: float) -> None:
        """Maintain the current joint pose for a specified duration."""
        steps = max(1, int(duration / self.model.opt.timestep))
        for _ in range(steps):
            if self.viewer and not self.viewer.is_running():
                break
            self._apply_pd(self.current_q)
            self._update_attachment()
            mujoco.mj_step(self.model, self.data)
            if self.viewer:
                self.viewer.sync()

    # ------------------------------------------------------------------ helpers
    def _compute_freejoint_indices(self, body_name: str) -> Tuple[int, int]:
        """Return (qpos_index, qvel_index) for a freejoint body."""
        body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        if body_id < 0:
            raise RuntimeError(f"Body '{body_name}' not found in scene.")

        joint_id = self.model.body_jntadr[body_id]
        if joint_id < 0:
            raise RuntimeError(f"Body '{body_name}' does not have a freejoint.")

        if self.model.jnt_type[joint_id] != mujoco.mjtJoint.mjJNT_FREE:
            raise RuntimeError(f"Body '{body_name}' is expected to use a freejoint.")

        qposadr = int(self.model.jnt_qposadr[joint_id])
        qveladr = int(self.model.jnt_dofadr[joint_id])
        return qposadr, qveladr

    def _get_box_position(self, body_name: str) -> np.ndarray:
        body_id = self.body_ids[body_name]
        return self.data.xpos[body_id].copy()

    def _solve_ik(self, position: np.ndarray) -> np.ndarray:
        pose = np.eye(4)
        pose[:3, :3] = self.down_orientation
        pose[:3, 3] = position

        q_goal, error = inverse_kinematics(pose, initial_guess=self.current_q)
        if error > 5e-3:
            print(f"[IK] Warning: residual error {error:.4f} for target {position}")
        return q_goal

    def _plan_and_follow(self, q_goal: np.ndarray, duration: float) -> None:
        if duration <= 0.0:
            return

        q_traj, _, _ = generate_joint_trajectory(
            self.data.qpos[:6].copy(), q_goal, duration, self.control_rate
        )
        for q_target in q_traj:
            if not self._step_with_target(q_target):
                break
        self.current_q = self.data.qpos[:6].copy()

    def _apply_pd(self, q_target: np.ndarray) -> None:
        q_err = q_target - self.data.qpos[:6]
        qd_err = -self.data.qvel[:6]
        self.data.ctrl[:] = self.kp * q_err + self.kd * qd_err

    def _update_attachment(self) -> None:
        if self.attached_box is None:
            return

        qpos_index, qvel_index = self.box_joint_index[self.attached_box]
        eff_pos = self.data.site_xpos[self.ee_site_id]
        target_pos = eff_pos + self.attachment_offset

        self.data.qpos[qpos_index:qpos_index + 3] = target_pos
        self.data.qpos[qpos_index + 3:qpos_index + 7] = self.attachment_quat
        self.data.qvel[qvel_index:qvel_index + 6] = 0.0

    def _step_with_target(self, q_target: np.ndarray) -> bool:
        for _ in range(self.steps_per_sample):
            if self.viewer and not self.viewer.is_running():
                return False
            self._apply_pd(q_target)
            self._update_attachment()
            mujoco.mj_step(self.model, self.data)
            if self.viewer:
                self.viewer.sync()
        return True

    def _attach_box(self, box_name: str) -> None:
        site_pos = self.data.site_xpos[self.ee_site_id].copy()
        box_pos = self._get_box_position(box_name)
        self.attached_box = box_name
        self.attachment_offset = box_pos - site_pos
        self.attachment_quat = np.array([1.0, 0.0, 0.0, 0.0])
        self.status_text = f"Attached {box_name}"

    def _release_box(self) -> None:
        if self.attached_box:
            print(f"[Sorting] Released {self.attached_box}")
        self.attached_box = None
        self.status_text = "Box released"

    # ----------------------------------------------------------------- sequence
    def run(self) -> None:
        with mujoco.viewer.launch_passive(
            self.model,
            self.data,
            show_left_ui=False,
            show_right_ui=False,
        ) as viewer:
            self.viewer = viewer
            viewer.cam.azimuth = 135
            viewer.cam.elevation = -25
            viewer.cam.distance = 3.2
            viewer.cam.lookat[:] = [0.0, 0.0, 0.6]

            # 1. Look Vertical to the Sky (Start Pose)
            self.status_text = "Moving to Sky Pose..."
            print("[Sorting] Moving to Sky Pose (Vertical)...")
            self._plan_and_follow(self.q_sky, 4.0) # Move smoothly over 4 seconds
            self._hold_current_pose(1.0) # Pause for a moment

            # 2. Execute Sorting Tasks
            for task in self.tasks:
                if not viewer.is_running():
                    break
                self._execute_task(task)

            self.status_text = "Sorting sequence complete. Press ESC to exit."
            while viewer.is_running():
                self._apply_pd(self.current_q)
                self._update_attachment()
                mujoco.mj_step(self.model, self.data)
                viewer.sync()

    def _execute_task(self, task: BoxTask) -> None:
        # Use specific drop target for this box body
        if task.body in self.drop_targets:
            drop_pos = self.drop_targets[task.body]
        else:
            # Fallback (should not happen with current logic)
            print(f"Warning: No specific drop target for {task.body}, defaulting.")
            drop_pos = np.array([0.25, 0.0, 0.47])

        pick_pos = self.box_pick_targets[task.body].copy()

        hover_above_pick = pick_pos.copy()
        hover_above_pick[2] += self.hover_offset
        grasp_pos = pick_pos.copy()
        grasp_pos[2] += self.pick_offset

        hover_above_drop = drop_pos.copy()
        hover_above_drop[2] += self.hover_offset
        release_pos = drop_pos.copy()
        release_pos[2] += self.place_offset

        # Sequence:
        # 1. Move from current pose (Sky or previous drop) to Hover Pick
        self.status_text = f"Moving above {task.body}"
        self._move_sequence([
            (hover_above_pick, 3.0),
            (grasp_pos, 1.5),
        ])

        # 2. Attach and Pick Up
        self._attach_box(task.body)
        self._move_sequence([
            (hover_above_pick, 1.5),
            (hover_above_drop, 3.0), # Move to bin
            (release_pos, 1.5),
        ])

        # 3. Release and Retreat
        self._release_box()
        self._move_sequence([
            (hover_above_drop, 1.5),
        ])

    def _move_sequence(self, targets: Iterable[Tuple[np.ndarray, float]]) -> None:
        for position, duration in targets:
            if self.viewer and not self.viewer.is_running():
                return
            q_goal = self._solve_ik(position)
            self._plan_and_follow(q_goal, max(duration, 0.8))


def main() -> None:
    model_path = _resolve_model_path()

    print("=" * 72)
    print("Sorting Station Simulation")
    print("=" * 72)
    print("Launching MuJoCo viewer with automated pick-and-place routine...")
    print(f"Model: {model_path}")

    sim = SortingStationSimulation(model_path)
    start_time = time.time()
    try:
        sim.run()
    except KeyboardInterrupt:
        print("\nSimulation interrupted by user.")
    finally:
        duration = time.time() - start_time
        print(f"Simulation finished after {duration:.1f} seconds.")


if __name__ == "__main__":
    main()