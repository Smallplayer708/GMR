"""Multi-priority QP framework: custom mink tasks for the upper-body retargeting path.

Implements the energy terms of the framework (doc: 肘部运动优化-多优先级QP框架与实施记录.md §3):

  Goal 2  elbow virtual target p_e,des   -> ElbowVirtualTargetTask   (position task, soft)
  Goal 1  physiological elbow limits     -> ElbowPhysioLimitTask     (hinge on q_elbow)
  Goal 1b medial (adduction) penalty     -> MedialDeviationTask      (elbow stays outside)
  Goal 3  temporal smoothing             -> mink.PostureTask (target = current q, velocity damping)
  Goal 3b natural posture attractor      -> mink.PostureTask (target = q_rest)

The virtual target is computed in the HUMAN frame (same convention as the synthesis layer:
gravity = (0,-1,0), back = R_pelvis*(0,0,-1), lateral = R_pelvis*(±1,0,0)), then rotated into
the robot frame. Geometry reuses xrobot_utils._circle_geometry so the QP layer's phi0 is
consistent with the synthesis phi (S5 requirement).
"""
import numpy as np
import mujoco as mj
from mink import Task

from .xrobot_utils import _circle_geometry, _qrotate

# human frame <-> robot frame (same as GMR scripts' to_rh)
ROT_MAT = np.array([[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]])
GRAV_H = np.array([0.0, -1.0, 0.0])          # human-frame gravity (synthesis convention)
BACK_H = np.array([0.0, 0.0, -1.0])           # human-frame backward
LAT_L_H = np.array([-1.0, 0.0, 0.0])          # human-frame left (left arm outward)
LAT_R_H = np.array([1.0, 0.0, 0.0])           # human-frame right (right arm outward)


FROZEN_DOFS = set(range(6)) | {18, 19, 20}   # root + waist (anchor_root / freeze_dofs)


def _mask_jac(J):
    'Zero columns of frozen DOFs (root 0:6, waist 18:21): the QP must not use joints'
    'that are zeroed after the solve (otherwise it cheats and the post-hoc freeze'
    'breaks the task/equality solution).'
    J = J.copy()
    for d in FROZEN_DOFS:
        J[:, d] = 0.0
    return J


def _body_pos(configuration, body_name):
    return configuration.data.xpos[configuration.model.body(body_name).id]


class ElbowVirtualTargetTask(Task):
    """Goal 2: pull the elbow toward the (reachability-clamped) virtual target point.

    error = x_e - p_e,des  (3-dim, world frame)
    jacobian = position jacobian of the elbow body (free-joint columns zeroed:
               the root is anchored / frozen by anchor_root).
    """

    def __init__(self, model, body_name, cost=1.5, gain=1.0, lm_damping=1.0):
        super().__init__(cost=np.ones(3) * cost, gain=gain, lm_damping=lm_damping)
        self.model = model
        self.body_name = body_name
        self.p_e_des = None

    def set_target(self, p):
        self.p_e_des = np.asarray(p, dtype=float)

    def compute_error(self, configuration):
        if self.p_e_des is None:
            return np.zeros(3)
        return _body_pos(configuration, self.body_name) - self.p_e_des

    def compute_jacobian(self, configuration):
        nv = self.model.nv
        bid = self.model.body(self.body_name).id
        jacp = np.zeros((3, nv))
        mj.mj_jac(self.model, configuration.data, jacp, None,
                  configuration.data.xpos[bid], bid)
        return _mask_jac(jacp)


class ElbowPhysioLimitTask(Task):
    """Goal 1: soft hinge penalizing elbow hyperextension.

    Physiological angle theta_e = pi - (q_e - q_e0) with q_e0 the calibrated
    fully-extended offset. Hyperextension (theta_e > 180 deg) <-> q_e < q_e0.
    Penalty is active when q_e < q_thr (q_thr = q_e0 - margin, default -0.35 rad,
    i.e. beyond the natural hanging bend of about -18 deg = -0.31 rad).
    error = max(0, q_thr - q_e)   jacobian = -1 on the elbow column.
    """

    def __init__(self, model, qpos_idx, cost=4.0, q_thr=-0.35, gain=1.0):
        super().__init__(cost=np.array([cost]), gain=gain)
        self.model = model
        self.qpos_idx = qpos_idx      # qpos index of the elbow joint
        self.q_thr = q_thr
        self.dof = qpos_idx - 1       # revolute dof index (after 7-dof free joint)

    def compute_error(self, configuration):
        q = configuration.q[self.qpos_idx]
        return np.array([max(0.0, self.q_thr - q)])

    def compute_jacobian(self, configuration):
        J = np.zeros((1, self.model.nv))
        J[0, self.dof] = -1.0
        return J


class MedialDeviationTask(Task):
    """Goal 1b: keep the elbow outside the shoulder line (no adduction past midline).

    active when -n_out . (x_e - S) > delta_in; error = max(0, -n_out.(x_e-S) - delta_in).
    """

    def __init__(self, model, body_name, cost=4.0, delta_in=0.02, gain=1.0):
        super().__init__(cost=np.array([cost]), gain=gain)
        self.model = model
        self.body_name = body_name
        self.delta_in = delta_in
        self.shoulder = None
        self.n_out = None

    def set_geometry(self, shoulder, n_out):
        self.shoulder = np.asarray(shoulder, dtype=float)
        self.n_out = np.asarray(n_out, dtype=float)

    def compute_error(self, configuration):
        x = _body_pos(configuration, self.body_name)
        viol = -np.dot(self.n_out, x - self.shoulder) - self.delta_in
        return np.array([max(0.0, viol)])

    def compute_jacobian(self, configuration):
        nv = self.model.nv
        bid = self.model.body(self.body_name).id
        jacp = np.zeros((3, nv))
        mj.mj_jac(self.model, configuration.data, jacp, None,
                  configuration.data.xpos[bid], bid)
        return _mask_jac(-(self.n_out @ jacp).reshape(1, nv))


class VelocityEqualityTask(Task):
    """Equality constraint J_w . delta_q = 0 (used by cascaded level 2: keep the
    level-1 wrist solution fixed while refining in its nullspace)."""

    def __init__(self, model, body_name, gain=1.0):
        super().__init__(cost=np.zeros(3), gain=gain)
        self.model = model
        self.body_name = body_name

    def compute_error(self, configuration):
        return np.zeros(3)

    def compute_jacobian(self, configuration):
        nv = self.model.nv
        bid = self.model.body(self.body_name).id
        jacp = np.zeros((3, nv))
        mj.mj_jac(self.model, configuration.data, jacp, None,
                  configuration.data.xpos[bid], bid)
        return _mask_jac(jacp)



class ShoulderRhythmTask(Task):
    """Direct joint-space shoulder-rhythm coupling: q_yaw -> q_yaw_rest + k * theta_elev.

    This is the framework's lambda3 term made explicit: the virtual-target phi0 coupling
    alone is too weak against the wrist-rotation task's early-yaw pull (measured: yaw
    locks ~100+ deg while elevation < 30 deg).
    """

    def __init__(self, model, qpos_idx, cost=3.0, q_yaw_rest=0.0, k=0.8, gain=1.0):
        super().__init__(cost=np.array([cost]), gain=gain)
        self.model = model
        self.qpos_idx = qpos_idx
        self.q_yaw_rest = q_yaw_rest
        self.k = k
        self.elev = 0.0

    def set_elevation(self, elev_rad):
        self.elev = float(elev_rad)

    def compute_error(self, configuration):
        q = configuration.q[self.qpos_idx]
        return np.array([q - (self.q_yaw_rest + self.k * self.elev)])

    def compute_jacobian(self, configuration):
        J = np.zeros((1, self.model.nv))
        J[0, self.qpos_idx - 1] = 1.0
        return J


def build_energy_tasks(model, l1_robot=0.0821, l2_robot=0.1843,
                       lam_virtual=1.5, lam_physio=4.0, lam_medial=4.0,
                       lam_smooth=0.5, lam_posture=0.5, q_thr=-0.35,
                       phi_rest=0.0, k=1.0, reach_clamp=0.081,
                       lam_rhythm=0.0, q_yaw_rest=0.0, q_rest_arm=None,
                       q_yaw_rest_L=0.0, q_yaw_rest_R=0.0):
    # q_rest_arm: length-14 array (rad) for qpos 22..35 (L pitch,roll,yaw,elbow,wr,wp,wy,
    # R pitch,roll,yaw,elbow,wr,wp,wy). Default: G1 zero pose with a slight elbow bend.
    # The G1 zero pose is NOT the natural hang (measured: posture task with zeros pulled
    # the arms ~130 deg backward); use the A-path (real body) hang mean instead.
    """Construct the goal-1/2/3 task set + a per-frame pre_solve_hook.

    Returns (tasks, hook) where hook(configuration, gmr) is called by retarget()
    before every QP solve to refresh task targets.
    hook.human_pelvis_quat and hook.synth_body must be set by the caller each frame
    (the human-frame pelvis quaternion from synthesize_upper_body).
    """
    from mink import PostureTask

    tasks = []
    virtual = {}
    physio = {}
    medial = {}
    rhythm = {}
    for side, elbow_body, shoulder_body, lat_h in (
            ('L', 'left_elbow_link', 'left_shoulder_yaw_link', LAT_L_H),
            ('R', 'right_elbow_link', 'right_shoulder_yaw_link', LAT_R_H)):
        vt = ElbowVirtualTargetTask(model, elbow_body, cost=lam_virtual)
        pt = ElbowPhysioLimitTask(model, 25 if side == 'L' else 32, cost=lam_physio, q_thr=q_thr)
        md = MedialDeviationTask(model, elbow_body, cost=lam_medial)
        virtual[side] = (vt, shoulder_body)
        physio[side] = pt
        medial[side] = (md, shoulder_body)
        tasks += [vt, pt, md]
        if lam_rhythm > 0:
            yaw_rest_side = q_yaw_rest_L if side == 'L' else q_yaw_rest_R
            rt = ShoulderRhythmTask(model, 24 if side == 'L' else 31, cost=lam_rhythm,
                                    q_yaw_rest=yaw_rest_side, k=k)
            rhythm[side] = rt
            tasks.append(rt)

    # smoothing: velocity damping via PostureTask(target = current q)
    arm_cost = np.zeros(model.nv)
    arm_cost[20:35] = lam_smooth           # dofs 20..34 = waist_pitch..right_wrist_yaw
    smooth = PostureTask(model, cost=arm_cost)
    tasks.append(smooth)

    # natural posture attractor: A-path (real body) hang mean by default
    q_rest = np.zeros(model.nq)
    if q_rest_arm is not None:
        q_rest[22:36] = np.asarray(q_rest_arm, dtype=float)
    else:
        q_rest[25] = -0.20                     # left elbow natural hang
        q_rest[32] = -0.20                     # right elbow natural hang
    posture = {}
    for side, dofs in (('L', (21, 22, 23, 24)), ('R', (28, 29, 30, 31))):   # shoulders + elbow
        pt = PostureTask(model, cost=np.zeros(model.nv))
        pt.set_target(q_rest.copy())
        pt.side_dofs = dofs
        posture[side] = pt
        tasks.append(pt)

    hook = _VirtualTargetHook(model, virtual, medial, physio, smooth, posture,
                              l1_robot, l2_robot, phi_rest, k, reach_clamp, rhythm,
                              lam_posture=lam_posture)
    return tasks, hook


class _VirtualTargetHook:
    """Per-frame target refresh for the energy tasks (see build_energy_tasks)."""

    def __init__(self, model, virtual, medial, physio, smooth, posture,
                 l1, l2, phi_rest, k, reach_clamp, rhythm=None, lam_posture=0.5):
        self.model = model
        self.virtual = virtual            # side -> (task, shoulder_body)
        self.medial = medial
        self.physio = physio
        self.smooth = smooth
        self.posture = posture
        self.l1, self.l2 = l1, l2
        self.phi_rest = phi_rest
        self.k = k
        self.reach_clamp = reach_clamp
        self.rhythm = rhythm or {}
        self.lam_posture = lam_posture
        self.human_pelvis_quat = None     # set by the caller each frame
        self.synth_rh = None              # robot-frame synth dict (targets), set by caller
        self.opposite_shoulder = {'L': 'right_shoulder_yaw_link',
                                  'R': 'left_shoulder_yaw_link'}

    def __call__(self, configuration, gmr):
        data = configuration.data
        model = self.model
        mj.mj_forward(model, data)
        self.smooth.set_target(configuration.q.copy())
        if self.human_pelvis_quat is None:
            return
        pq = np.asarray(self.human_pelvis_quat, dtype=float)
        back_h = _qrotate(pq, BACK_H)
        for side, (vt, sh_body) in self.virtual.items():
            lat_h = LAT_L_H if side == 'L' else LAT_R_H
            lat_w_h = _qrotate(pq, lat_h)
            pole_h = GRAV_H + 0.35 * back_h + 0.5 * lat_w_h
            pole_h = pole_h / float(np.linalg.norm(pole_h))
            side_l = 'Left' if side == 'L' else 'Right'
            # S = the robot's ACTUAL shoulder (FK): the elbow target must be reachable
            # from the real shoulder, otherwise the clamp is meaningless (the synth
            # shoulder carries human pelvis roll/pitch the robot root does not have).
            S = data.xpos[model.body(sh_body).id]
            if self.synth_rh is None:
                w_name = ('left' if side == 'L' else 'right') + '_wrist_yaw_link'
                W = data.xpos[model.body(w_name).id]
            else:
                W = np.asarray(self.synth_rh[side_l + '_Wrist'][0], dtype=float)
            # arm elevation (human frame, target wrist vs FK shoulder) -> rhythm +
            # posture-cost scaling (decays with elevation so the overhead raise is not
            # blocked by the hang-bend posture pull)
            u0h = ROT_MAT.T @ (W - S)
            nl_u = float(np.linalg.norm(u0h))
            elev_rad = 0.0
            if nl_u > 1e-6:
                elev_rad = float(np.arccos(np.clip(np.dot(u0h / nl_u, GRAV_H), -1.0, 1.0)))
            if side in self.rhythm:
                self.rhythm[side].set_elevation(elev_rad)
            if side in self.posture:
                pt = self.posture[side]
                scale = 1.0 - 0.9 * min(1.0, np.degrees(elev_rad) / 60.0)
                cost = np.zeros(model.nv)
                for d in pt.side_dofs:
                    cost[d] = self.lam_posture * scale
                pt.set_cost(cost)
            # L3 virtual target = the synth elbow direction (anchored at the FK
            # shoulder): the L1 is now PICO-calibrated (5-8cm lateral + forward), and
            # following it keeps the recovery hover fixed (the pole-circle target made
            # the R recovery hover return at 67.7% of frames).
            S_synth = np.asarray(self.synth_rh[side_l + '_Shoulder'][0], dtype=float)
            E_synth = np.asarray(self.synth_rh[side_l + '_Elbow'][0], dtype=float)
            d = E_synth - S_synth
            nd = float(np.linalg.norm(d))
            if nd > 1e-4:
                vt.set_target(S + d / nd * self.reach_clamp)
            # human frame (rotation only; geometry is translation-invariant)
            Sh = ROT_MAT.T @ S
            Wh = ROT_MAT.T @ W
            g = _circle_geometry(Sh, Wh, self.l1, self.l2, pole_h)
            if g is None:
                vt.set_target(_body_pos(configuration, ('left' if side == 'L' else 'right') + '_elbow_link'))
                continue
            _, _, u0, _, h, C, pdir, perp = g
            phi0 = self.phi_rest + self.k * elev_rad
            p = C + h * (np.cos(phi0) * pdir + np.sin(phi0) * perp)
            d = float(np.linalg.norm(p - Sh))
            if d > self.reach_clamp:
                p = Sh + (p - Sh) * (self.reach_clamp / d)
            vt.set_target(ROT_MAT @ p)
            # medial geometry: outward = away from the opposite shoulder;
            # reference = torso midline so the inward standing elbow is allowed
            So = data.xpos[model.body(self.opposite_shoulder[side]).id]
            n_out = S - So
            nl = float(np.linalg.norm(n_out))
            if nl > 1e-6:
                n_out = n_out / nl
            md, _ = self.medial[side]
            md.set_geometry(0.5 * (S + So), n_out)
