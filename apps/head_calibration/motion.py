"""Move and hold the simulated calibration arms with their native joint motors."""

import numpy as np

CALIBRATION_POSE_RAD = {
    f"{side}_{joint}_joint": angle
    for side, angles in (
        ("left", (-0.650, 0.257, -0.532, -0.130, 1.533, 0.849, 0.010)),
        ("right", (-0.850, -0.320, 0.500, 0.320, -1.300, 0.500, 0.030)),
    )
    for joint, angle in zip(
        ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll", "wrist_pitch", "wrist_yaw"),
        angles,
        strict=True,
    )
}


class CalibrationArmControl:
    POSITION_TOL_RAD = 0.02
    VELOCITY_TOL_RAD_S = 0.03
    HOLD_S = 0.4

    def __init__(self, model, data, target_rad, *, travel_s=3.0):
        import mujoco

        if not np.isfinite(travel_s) or travel_s <= 0 or not target_rad:
            raise ValueError("a nonempty calibration pose and positive motion duration are required")
        self._mujoco = mujoco
        self.model, self.data = model, data
        self.names = tuple(target_rad)
        self._travel = travel_s
        self._start_time = float(data.time)
        joints = np.array([model.joint(name).id for name in self.names], dtype=int)
        self.target = np.asarray(list(target_rad.values()), dtype=float)
        limits = model.jnt_range[joints]
        if (
            np.any(model.jnt_type[joints] != mujoco.mjtJoint.mjJNT_HINGE)
            or not np.isfinite(self.target).all()
            or np.any(model.jnt_limited[joints] & ((self.target < limits[:, 0]) | (self.target > limits[:, 1])))
        ):
            raise ValueError("calibration pose is outside the compiled hinge limits")
        self._qpos = model.jnt_qposadr[joints]
        self._dofs = model.jnt_dofadr[joints]
        motor_ids = []
        for joint in joints:
            motor = np.flatnonzero(
                (model.actuator_trntype == mujoco.mjtTrn.mjTRN_JOINT) & (model.actuator_trnid[:, 0] == joint)
            )
            if len(motor) != 1:
                raise ValueError(f"require one native motor for {model.joint(joint).name}")
            motor_ids.append(int(motor[0]))
        self.motors = np.array(motor_ids)
        self._gear = model.actuator_gear[self.motors, 0]
        self._ctrl_bounds = model.actuator_ctrlrange[self.motors]
        if (
            np.any(self._gear == 0)
            or not model.actuator_ctrllimited[self.motors].all()
            or np.any(model.actuator_dyntype[self.motors] != mujoco.mjtDyn.mjDYN_NONE)
            or np.any(model.actuator_gaintype[self.motors] != mujoco.mjtGain.mjGAIN_FIXED)
            or np.any(model.actuator_gainprm[self.motors, 0] != 1)
            or np.any(model.actuator_biastype[self.motors] != mujoco.mjtBias.mjBIAS_NONE)
        ):
            raise ValueError("calibration requires torque-limited direct joint motors")
        wrist = np.array(["_wrist_" in name for name in self.names])
        self._kp = np.where(wrist, 40.0, 85.0)
        self._kd = np.where(wrist, 2.8, 5.2)
        self.start = self.position_rad
        self.reference_rad = self.start.copy()
        self.reference_velocity_rad_s = np.zeros(len(self.names))
        self.command_time_s = self._start_time
        self._settled_since = None
        self.first_ready_s = None

    @property
    def position_rad(self):
        return self.data.qpos[self._qpos]

    @property
    def velocity_rad_s(self):
        return self.data.qvel[self._dofs]

    @property
    def ready(self):
        return (
            self._settled_since is not None
            and self.data.time - self._settled_since >= self.HOLD_S - 1e-9
            and self.max_error_rad <= self.POSITION_TOL_RAD
            and np.max(np.abs(self.velocity_rad_s)) <= self.VELOCITY_TOL_RAD_S
        )

    @property
    def max_error_rad(self):
        return float(np.max(np.abs(self.position_rad - self.target)))

    def step(self):
        self.command_time_s = float(self.data.time)
        duration = np.clip((self.command_time_s - self._start_time) / self._travel, 0.0, 1.0)
        displacement = self.target - self.start
        self.reference_rad = self.start + displacement * duration**3 * (10 + duration * (-15 + 6 * duration))
        self.reference_velocity_rad_s = displacement * (30 * duration**2 * (1 - duration) ** 2 / self._travel)
        self._mujoco.mj_step1(self.model, self.data)
        # MuJoCo model bias is available for the currently observed simulated
        # state. The motors apply bounded control and the integrator updates q.
        force = (
            self._kp * (self.reference_rad - self.position_rad)
            + self._kd * (self.reference_velocity_rad_s - self.velocity_rad_s)
            + self.data.qfrc_bias[self._dofs]
        )
        self.data.ctrl[self.motors] = np.clip(force / self._gear, self._ctrl_bounds[:, 0], self._ctrl_bounds[:, 1])
        self._mujoco.mj_step2(self.model, self.data)
        if (
            self.data.time - self._start_time >= self._travel - 1e-9
            and self.max_error_rad <= self.POSITION_TOL_RAD
            and np.max(np.abs(self.velocity_rad_s)) <= self.VELOCITY_TOL_RAD_S
        ):
            if self._settled_since is None:
                self._settled_since = float(self.data.time)
            if self.first_ready_s is None and self.ready:
                self.first_ready_s = float(self.data.time)
        else:
            self._settled_since = None
