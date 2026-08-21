from rich import print

try:
    import xrobotoolkit_sdk as xrt
except:
    print("[bold red]xrobotoolkit_sdk not found, skip for now. If you do not use XRobotStreamer, it's fine.[/bold red]")
import time
import numpy as np
from .rot_utils import quat_mul_np, quat_rotate_inverse_np
from scipy.spatial.transform import Rotation as R
import json
import cv2
import os

DEFAULT_TEMPLATE_PATH = os.path.join(os.path.expanduser("~"), ".xrobotkit", "standing_template.json")

UPPER_BODY_JOINT_NAMES = [
    "Pelvis", "Spine3", "Head",
    "Left_Shoulder", "Left_Elbow", "Left_Wrist",
    "Right_Shoulder", "Right_Elbow", "Right_Wrist",
]

# ============================= Step 1: math helpers (upper-body skeleton) =============================

def _qmul(q1, q2):
    """Multiply two quaternions (scalar-first [w, x, y, z])."""
    return quat_mul_np(np.asarray(q1, dtype=float), np.asarray(q2, dtype=float), scalar_first=True)


def _qconj(q):
    q = np.asarray(q, dtype=float)
    return np.array([q[0], -q[1], -q[2], -q[3]])


def _qnormalize(q):
    q = np.asarray(q, dtype=float)
    n = float(np.linalg.norm(q))
    if n < 1e-8:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return q / n


def _qrotate(q, v):
    """Rotate vector v by quaternion q (forward rotation, scalar-first)."""
    return quat_rotate_inverse_np(_qconj(q), np.asarray(v, dtype=float), scalar_first=True)


def _quat_yaw(q):
    """World yaw (radians, rotation about the up axis +Y) of a scalar-first quaternion."""
    q = np.asarray(q, dtype=float)
    w, x, y, z = q
    return float(np.arctan2(2.0 * (w * y - x * z), 1.0 - 2.0 * (y * y + z * z)))


def _yaw_quat(yaw):
    """Upright quaternion with only a world yaw rotation (about +Y)."""
    c, s = np.cos(yaw / 2.0), np.sin(yaw / 2.0)
    return np.array([c, 0.0, s, 0.0])


def _world_up_y(quat):
    """Y component of the frame's up axis after rotating world +Y by `quat`."""
    up = _qrotate(np.asarray(quat, dtype=float), np.array([0.0, 1.0, 0.0]))
    return float(up[1])


def _qslerp(q1, q2, t):
    """Spherical linear interpolation between two quaternions (scalar-first)."""
    q1 = np.asarray(q1, dtype=float)
    q2 = np.asarray(q2, dtype=float)
    d = float(np.clip(np.dot(q1, q2), -1.0, 1.0))
    if d < 0.0:
        q2 = -q2
        d = -d
    if d > 0.9995:
        res = q1 + t * (q2 - q1)
    else:
        th = np.arccos(d)
        sth = np.sin(th)
        res = (np.sin((1.0 - t) * th) * q1 + np.sin(t * th) * q2) / sth
    return _qnormalize(res)


def _blend_body_dicts(dict_a, dict_b, t):
    """Per-joint lerp (position) + slerp (orientation) blend of two body dicts."""
    out = {}
    for name in dict_b.keys():
        pa = np.asarray(dict_a[name][0], dtype=float)
        qa = np.asarray(dict_a[name][1], dtype=float)
        pb = np.asarray(dict_b[name][0], dtype=float)
        qb = np.asarray(dict_b[name][1], dtype=float)
        out[name] = [(pa + t * (pb - pa)).tolist(), _qslerp(qa, qb, t).tolist()]
    return out


def _pose7_valid(pose7):
    """A pose is valid only when its position is non-zero (SDK returns zeros when no data)."""
    if pose7 is None or len(pose7) < 7:
        return False
    return float(np.linalg.norm(np.asarray(pose7[:3], dtype=float))) > 1e-3


ELBOW_LOCK_ANGLE = np.deg2rad(170.0)


def _arm_max_reach(l1, l2):
    """Max shoulder-wrist distance before the elbow hits its lock angle (rigid arm, slight bend)."""
    return float(np.sqrt(max(l1 * l1 + l2 * l2 - 2.0 * l1 * l2 * np.cos(ELBOW_LOCK_ANGLE), 0.0)))


def _quat_angle(q1, q2):
    """Angular distance (rad) between two quaternions."""
    q1, q2 = np.asarray(q1, dtype=float), np.asarray(q2, dtype=float)
    d = float(min(np.linalg.norm(q1 - q2), np.linalg.norm(q1 + q2)))
    return 2.0 * float(np.arcsin(np.clip(d / 2.0, 0.0, 1.0)))


def _flip_about_axis(quat, axis):
    """Rotate a quaternion 180 deg about a unit axis (undoes elbow-frame azimuth flips)."""
    q_axis = R.from_rotvec(np.asarray(axis, dtype=float) * np.pi).as_quat(scalar_first=True)
    return _qmul(q_axis, quat)


# ---- self-check optimization knobs (mutated by SelfCheckMonitor.optimize) ----
XU_WRIST_P0_LAT = 0.04        # standing wrist lateral correction (m)
XU_ELBOW_P0_LAT_L = 0.04      # standing elbow lateral target, left (m)
XU_ELBOW_P0_LAT_R = 0.04      # standing elbow lateral target, right (m)
XU_ELBOW_P0_FWD = 0.04        # standing elbow forward target (m)
XU_ELBOW_P0_DY = 0.05         # standing elbow lowering (m)
XU_ENV_W = 25.0               # phi-grid envelope weight (self-check raises it vs elbow_high)


def _circle_geometry(shoulder, wrist, l1, l2, pole):
    """2-bone solution circle geometry: returns S, W, u0, a, h, C, pdir, perp.

    pdir = reference azimuth (pole projection on the circle plane, world-down fallback),
    perp = cross(u0, pdir); elbow on the circle = C + h*(cos(phi)*pdir + sin(phi)*perp).
    """
    S = np.asarray(shoulder, dtype=float)
    W = np.asarray(wrist, dtype=float)
    d_vec = W - S
    raw_d = float(np.linalg.norm(d_vec))
    if raw_d < 1e-6:
        return None
    u0 = d_vec / raw_d
    d_max = _arm_max_reach(l1, l2)
    d = float(np.clip(raw_d, abs(l1 - l2) + 1e-4, d_max))
    a = float(np.clip((l1 * l1 - l2 * l2 + d * d) / (2.0 * d), 0.0, l1 - 1e-6))
    h = float(np.sqrt(max(l1 * l1 - a * a, 0.0)))
    C = S + a * u0
    pdir = None
    if pole is not None:
        p = np.asarray(pole, dtype=float)
        g = p - u0 * float(np.dot(p, u0))
        ng = float(np.linalg.norm(g))
        if ng > 1e-6:
            pdir = g / ng
    if pdir is None:
        ref = np.array([0.0, -1.0, 0.0])  # world down
        g = ref - u0 * float(np.dot(ref, u0))
        ng = float(np.linalg.norm(g))
        if ng < 1e-6:
            for cand in (np.array([1.0, 0.0, 0.0]), np.array([0.0, 0.0, 1.0])):
                g = cand - u0 * float(np.dot(cand, u0))
                ng = float(np.linalg.norm(g))
                if ng > 1e-6:
                    break
        pdir = g / ng
    perp = np.cross(u0, pdir)
    return S, W, u0, a, h, C, pdir, perp


def _elbow_phi_grid(shoulder, wrist, l1, l2, pole, phi_learned, pelvis_quat, side, phi_prev=None,
                      circle=None):
    """Constrained 1-DOF elbow-azimuth optimization (L2/L3 revised).

    Minimizes a naturalness cost over the circle azimuth phi:
        w1*|phi - phi_learned|^2  (operator prior, when available)
      + w2*|phi|^2               (pull toward the natural body-frame pole)
      + w3*|phi - phi_prev|^2    (temporal coherence, prevents per-frame argmin jumps)
      + hard penalties for violating the position envelope:
          - the elbow must stay on the arm's outward side of the shoulder (no adduction
            past the midline / no crossed arms),
          - the elbow must not go farther than 0.25 m behind the shoulder (no "elbow
            behind the torso").
    """
    g = circle if circle is not None else _circle_geometry(shoulder, wrist, l1, l2, pole)
    if g is None:
        return 0.0
    _, _, u0, a, h, C, pdir, perp = g
    pq = np.asarray(pelvis_quat, dtype=float)
    sign = 1.0 if side == "Left" else -1.0  # left: elbow at/left of shoulder; right: at/right
    # near-vertical (hanging) arms: the natural elbow points backward; forbid poking forward
    vertical = float(np.dot(u0, np.array([0.0, -1.0, 0.0]))) > 0.90
    # when the wrist is below the shoulder the elbow must hang below the shoulder line
    # (fixes the "elbow hovering above the shoulder" standing pose: the circle is
    # vertical for a wrist at shoulder height and phi would otherwise be free to lift)
    wrist_below = float(np.asarray(wrist, dtype=float)[1] - np.asarray(shoulder, dtype=float)[1]) < 0.0
    best_phi, best_cost = 0.0, float("inf")
    for phi in np.linspace(-np.pi, np.pi, 73):
        E = C + h * (np.cos(phi) * pdir + np.sin(phi) * perp)
        Eb = _qrotate(_qconj(pq), E - np.asarray(shoulder, dtype=float))
        lat_viol = max(0.0, sign * Eb[0] - 0.06)
        back_viol = max(0.0, -Eb[2] - 0.25)
        fwd_viol = max(0.0, Eb[2] - 0.05) if vertical else 0.0
        up_viol = max(0.0, Eb[1] - 0.02) if wrist_below else 0.0
        cost = (8.0 * (phi - phi_learned) ** 2) if phi_learned is not None else 0.0
        cost += 0.3 * phi * phi
        if phi_prev is not None:
            dphi_t = np.arctan2(np.sin(phi - phi_prev), np.cos(phi - phi_prev))
            cost += 0.5 * dphi_t * dphi_t
        cost += XU_ENV_W * (lat_viol + back_viol + fwd_viol + up_viol)
        if cost < best_cost:
            best_cost, best_phi = cost, phi
    return float(best_phi)


def _two_bone_ik(shoulder, wrist, l1, l2, prev_elbow=None, pole=None, phi=None, circle=None):
    """Analytic 2-bone IK: elbow position from shoulder S, wrist W, upper-arm l1, forearm l2.

    The elbow lies on a circle perpendicular to the S-W axis (radius h, center C). The
    azimuth on that circle is set by, in priority order: `prev_elbow` (temporal
    continuity, prevents flips), `pole` (natural elbow direction, e.g. torso-biased
    gravity), or world down (fallback). Unreachable targets clamp to the elbow lock
    angle (170 deg) so the arm keeps a slight natural bend instead of a straight line.
    """
    S = np.asarray(shoulder, dtype=float)
    W = np.asarray(wrist, dtype=float)
    d_vec = W - S
    raw_d = float(np.linalg.norm(d_vec))
    if raw_d < 1e-6:
        return S + np.array([0.0, -l1, 0.0])
    u0 = d_vec / raw_d
    d_max = _arm_max_reach(l1, l2)
    d = float(np.clip(raw_d, abs(l1 - l2) + 1e-4, d_max))
    a = float(np.clip((l1 * l1 - l2 * l2 + d * d) / (2.0 * d), 0.0, l1 - 1e-6))
    h = float(np.sqrt(max(l1 * l1 - a * a, 0.0)))
    C = S + a * u0
    # reference azimuth direction on the circle plane: pole projection (or world-down fallback)
    pdir = None
    if pole is not None:
        p = np.asarray(pole, dtype=float)
        g = p - u0 * float(np.dot(p, u0))
        ng = float(np.linalg.norm(g))
        if ng > 1e-6:
            pdir = g / ng
    if pdir is None:
        ref = np.array([0.0, -1.0, 0.0])  # world down
        g = ref - u0 * float(np.dot(ref, u0))
        ng = float(np.linalg.norm(g))
        if ng < 1e-6:
            for cand in (np.array([1.0, 0.0, 0.0]), np.array([0.0, 0.0, 1.0])):
                g = cand - u0 * float(np.dot(cand, u0))
                ng = float(np.linalg.norm(g))
                if ng > 1e-6:
                    break
        pdir = g / ng
    if phi is not None:
        if circle is not None:
            _, _, _, _, hh, Cc, pp, pe = circle
            return Cc + hh * (np.cos(phi) * pp + np.sin(phi) * pe)
        perp = np.cross(u0, pdir)
        return C + h * (np.cos(phi) * pdir + np.sin(phi) * perp)
    if prev_elbow is not None:
        v = np.asarray(prev_elbow, dtype=float) - C
        v_plane = v - u0 * float(np.dot(v, u0))
        nv = float(np.linalg.norm(v_plane))
        if nv > 1e-6:
            return C + h * (v_plane / nv)
    return C + h * pdir


def _forearm_frame_quat(fore_dir):
    """Build an elbow-frame quaternion: x-axis along the forearm, y as up as possible, z = x * y."""
    x = np.asarray(fore_dir, dtype=float)
    xn = float(np.linalg.norm(x))
    if xn < 1e-6:
        return np.array([1.0, 0.0, 0.0, 0.0])
    x = x / xn
    y = np.array([0.0, 1.0, 0.0]) - x * np.dot(x, np.array([0.0, 1.0, 0.0]))
    yn = float(np.linalg.norm(y))
    if yn < 1e-4:
        for cand in (np.array([0.0, 0.0, 1.0]), np.array([1.0, 0.0, 0.0])):
            y = cand - x * np.dot(x, cand)
            yn = float(np.linalg.norm(y))
            if yn > 1e-4:
                break
    if yn < 1e-6:
        return np.array([1.0, 0.0, 0.0, 0.0])
    y = y / yn
    z = np.cross(x, y)
    return R.from_matrix(np.stack([x, y, z], axis=1)).as_quat(scalar_first=True)


class StandingTemplate:
    """Full-body standing skeleton stored as pelvis-relative joint poses (raw PICO frame).

    Loaded from a captured template file (e.g. ~/.xrobotkit/standing_template.json,
    VERSION 2, incl. controller->wrist and geometric->elbow arm calibration) or
    generated procedurally from human height. `compose()` re-anchors the skeleton at an
    arbitrary pelvis pose; `arm_lengths()` returns per-side arm link lengths.
    """

    VERSION = 2

    def __init__(self, joints, pelvis_from_head, pelvis_quat,
                 controller_to_wrist=None, geo_to_elbow=None):
        self.joints = joints  # name -> (rel_pos(3), rel_quat(4), scalar-first)
        self.pelvis_from_head = pelvis_from_head  # (pos(3), quat(4)) pelvis expressed in head frame
        self.pelvis_quat = np.asarray(pelvis_quat, dtype=float)
        # side -> (pos_offset(3) in controller frame, quat(4)) ; None when not calibrated
        self.controller_to_wrist = controller_to_wrist or {}
        # side -> quat(4) ; None when not calibrated
        self.geo_to_elbow = geo_to_elbow or {}

    def compose(self, pelvis_pos, pelvis_quat=None):
        """Rebuild the full 24-joint dict anchored at the given pelvis pose (raw PICO frame)."""
        pelvis_quat = self.pelvis_quat if pelvis_quat is None else np.asarray(pelvis_quat, dtype=float)
        out = {}
        for name, (rel_pos, rel_quat) in self.joints.items():
            pos = np.asarray(pelvis_pos, dtype=float) + _qrotate(pelvis_quat, rel_pos)
            quat = _qmul(pelvis_quat, rel_quat)
            out[name] = [pos.tolist(), _qnormalize(quat).tolist()]
        return out

    def arm_lengths(self):
        """Upper-arm / forearm lengths for both sides, derived from the template."""
        lengths = {}
        for side in ("Left", "Right"):
            sh = np.asarray(self.joints[side + "_Shoulder"][0], dtype=float)
            el = np.asarray(self.joints[side + "_Elbow"][0], dtype=float)
            wr = np.asarray(self.joints[side + "_Wrist"][0], dtype=float)
            lengths[side] = (float(np.linalg.norm(el - sh)), float(np.linalg.norm(wr - el)))
        return lengths

    @staticmethod
    def load(path):
        if not os.path.exists(path):
            return None
        try:
            with open(path) as f:
                data = json.load(f)
            joints = {}
            for name, v in data["joints"].items():
                joints[name] = (np.asarray(v["pos"], dtype=float), np.asarray(v["quat"], dtype=float))
            phf = (np.asarray(data["pelvis_from_head"]["pos"], dtype=float),
                   np.asarray(data["pelvis_from_head"]["quat"], dtype=float))
            ctw = {}
            for side, v in data.get("controller_to_wrist", {}).items():
                ctw[side] = (np.asarray(v["pos"], dtype=float), np.asarray(v["quat"], dtype=float))
            gte = {side: np.asarray(v, dtype=float)
                   for side, v in data.get("geo_to_elbow", {}).items()}
            return StandingTemplate(joints, phf, np.asarray(data["pelvis_quat"], dtype=float), ctw, gte)
        except Exception as e:
            print(f"[StandingTemplate] 加载站立模板失败，将使用参数化默认站姿: {e}")
            return None

    @staticmethod
    def parametric(height=1.7):
        """Procedural standing skeleton from human height (fallback when no capture exists).

        Proportions (fractions of height H, ground at 0): pelvis 0.53H, knee 0.28H,
        ankle 0.04H, head/eye 0.93H; torso pelvis->shoulder 0.28H, hip width 0.2,
        shoulder width 0.22. All joints are pelvis-relative (raw PICO frame, y up).
        """
        H = float(height)
        ident = [1.0, 0.0, 0.0, 0.0]

        def j(x, y, z):
            return ([float(x), float(y), float(z)], list(ident))

        shoulder_y = 0.28 * H
        elbow_y = shoulder_y - 0.19 * H
        wrist_y = elbow_y - 0.16 * H
        joints = {
            "Pelvis": j(0.0, 0.0, 0.0),
            "Left_Hip": j(-0.10, -0.02, 0.0),
            "Right_Hip": j(0.10, -0.02, 0.0),
            "Spine1": j(0.0, 0.12 * H, 0.0),
            "Left_Knee": j(-0.10, -0.24 * H, 0.0),
            "Right_Knee": j(0.10, -0.24 * H, 0.0),
            "Spine2": j(0.0, 0.21 * H, 0.0),
            "Left_Ankle": j(-0.10, -0.49 * H, 0.0),
            "Right_Ankle": j(0.10, -0.49 * H, 0.0),
            "Spine3": j(0.0, 0.29 * H, 0.0),
            "Left_Foot": j(-0.10, -0.52 * H, 0.05),
            "Right_Foot": j(0.10, -0.52 * H, 0.05),
            "Neck": j(0.0, 0.32 * H, 0.0),
            "Left_Collar": j(-0.04, 0.30 * H, 0.0),
            "Right_Collar": j(0.04, 0.30 * H, 0.0),
            "Head": j(0.0, 0.40 * H, 0.03),
            "Left_Shoulder": j(-0.11, shoulder_y, 0.0),
            "Right_Shoulder": j(0.11, shoulder_y, 0.0),
            "Left_Elbow": j(-0.16, elbow_y, 0.0),
            "Right_Elbow": j(0.16, elbow_y, 0.0),
            "Left_Wrist": j(-0.16, wrist_y, 0.0),
            "Right_Wrist": j(0.16, wrist_y, 0.0),
            "Left_Hand": j(-0.16, wrist_y, 0.0),
            "Right_Hand": j(0.16, wrist_y, 0.0),
        }
        pelvis_from_head = (j(0.0, -0.40 * H, 0.0)[0], ident)
        return StandingTemplate(joints, pelvis_from_head, np.array(ident, dtype=float),
                                controller_to_wrist={}, geo_to_elbow={})


def synthesize_upper_body(head_pose7, left_controller7, right_controller7, template,
                          prev_elbows=None, arm_lengths=None, elbow_prior=None):
    """Build the 9-joint upper-body dict (raw PICO frame) from live head + controllers + template.

    - Pelvis: template anchored straight below the live head, upright, yaw follows the live head
    - Spine3 / Shoulders: from the template (fixed torso)
    - Head: live headset pose
    - Wrists: live controller poses corrected by the template controller->wrist calibration
    - Elbows: analytic 2-bone IK with the template arm lengths; the IK branch keeps temporal
      continuity via `prev_elbows`; orientation = geometric forearm frame corrected by the
      template geo->elbow calibration.
    Returns (body_dict, new_prev_elbows). All data in the raw PICO (Unity left-handed) frame.
    """
    head_pos = np.asarray(head_pose7[:3], dtype=float)
    head_quat = np.asarray([head_pose7[6], head_pose7[3], head_pose7[4], head_pose7[5]], dtype=float)

    pelvis_quat = _yaw_quat(_quat_yaw(head_quat))
    pelvis_from_head_pos = np.asarray(template.pelvis_from_head[0], dtype=float)
    pelvis_pos = head_pos + np.array([0.0, pelvis_from_head_pos[1], 0.0])
    body = template.compose(pelvis_pos, pelvis_quat)

    body["Head"] = [head_pos.tolist(), _qnormalize(head_quat).tolist()]

    lengths = arm_lengths or template.arm_lengths()
    new_prev = dict(prev_elbows) if prev_elbows else {}
    spine3_pos = np.asarray(body["Spine3"][0], dtype=float)

    for side, ctrl7 in (("Left", left_controller7), ("Right", right_controller7)):
        shoulder = np.asarray(body[side + "_Shoulder"][0], dtype=float)
        l1, l2 = lengths[side]
        if l1 < 1e-4 or l2 < 1e-4:
            continue
        wrist_pos = np.asarray(body[side + "_Wrist"][0], dtype=float)
        wrist_quat = np.asarray(body[side + "_Wrist"][1], dtype=float)
        if _pose7_valid(ctrl7):
            ctrl_q = np.asarray(ctrl7[3:7], dtype=float)
            if abs(float(np.linalg.norm(ctrl_q)) - 1.0) < 0.05:
                if float(np.linalg.norm(np.asarray(ctrl7[:3], dtype=float) - shoulder)) < 1.0:
                    wrist_pos = np.asarray(ctrl7[:3], dtype=float)
                    # input cleaning on the raw controller position (valid frames only):
                    # a 5-frame median filter removes isolated tracking glitches (up to
                    # ~60 cm single-frame spikes) while fast but monotonic motion passes
                    # through unchanged (magnitude thresholds cannot tell them apart).
                    ring = new_prev.get(side + "_ctrl_ring")
                    if ring is None:
                        ring = []
                    ring.append(wrist_pos.copy())
                    if len(ring) > 5:
                        ring.pop(0)
                    if len(ring) >= 3:
                        wrist_pos = np.median(np.array(ring), axis=0)
                    new_prev[side + "_ctrl_ring"] = ring
                    wrist_quat = np.asarray([ctrl7[6], ctrl7[3], ctrl7[4], ctrl7[5]], dtype=float)
                    calib = template.controller_to_wrist.get(side)
                    if calib is not None:
                        pos_offset, quat_offset = calib
                        wrist_pos = wrist_pos + _qrotate(wrist_quat, pos_offset)
                        wrist_quat = _qmul(wrist_quat, quat_offset)
        # input cleaning: reject controller position glitches (up to ~60 cm jumps observed)
        # and lightly smooth the wrist target so the elbow/arm do not snap between frames.

        # rigid skeleton: clamp the wrist into the arm's reach (keeps bone lengths constant)
        sw = wrist_pos - shoulder
        sw_len = float(np.linalg.norm(sw))
        max_reach = _arm_max_reach(l1, l2)
        if sw_len > max_reach and sw_len > 1e-6:
            wrist_pos = shoulder + (sw / sw_len) * max_reach
        # natural-rest wrist correction: the operator holds the controllers slightly in front
        # of the body, so a hanging arm's wrist sits forward of the shoulder line and the
        # forearm pokes forward. When the arm hangs (wrist below the shoulder), blend the
        # wrist's horizontal position toward hanging directly below the shoulder - the
        # forearm then hangs vertically and the robot arm does not swing out/back. The
        # correction ramps in with depth below the shoulder to avoid a threshold snap.
        depth = float(shoulder[1] - wrist_pos[1])
        if depth > 0.10:
            w = float(min(1.0, (depth - 0.05) / 0.20))
            back_w = _qrotate(pelvis_quat, np.array([0.0, 0.0, -1.0]))
            lat_dir = np.array([-1.0, 0.0, 0.0]) if side == "Left" else np.array([1.0, 0.0, 0.0])
            lat_w = _qrotate(pelvis_quat, lat_dir)
            # wrist hangs slightly OUTWARD of the shoulder line (G1 natural stance: arms
            # slightly abducted, clears the thigh for forward raises)
            target_h = np.asarray([shoulder[0] + 0.00 * back_w[0] + XU_WRIST_P0_LAT * lat_w[0],
                                   shoulder[2] + 0.00 * back_w[2] + XU_WRIST_P0_LAT * lat_w[2]], dtype=float)
            cur_h = np.asarray([wrist_pos[0], wrist_pos[2]], dtype=float)
            wrist_pos = np.asarray(wrist_pos, dtype=float)
            dx = 0.9 * w * (target_h[0] - cur_h[0])
            dz = 0.9 * w * (target_h[1] - cur_h[1])
            dm = float(np.hypot(dx, dz))
            if dm > 0.03:  # per-frame cap: no wrist-target snaps from the correction
                dx *= 0.03 / dm; dz *= 0.03 / dm
            wrist_pos[0] += dx
            wrist_pos[2] += dz
        # natural elbow pole in the BODY frame: gravity + 0.35*back + 0.2*lateral (rotated by pelvis yaw)
        back_world = _qrotate(pelvis_quat, np.array([0.0, 0.0, -1.0]))
        lat_dir = np.array([-1.0, 0.0, 0.0]) if side == "Left" else np.array([1.0, 0.0, 0.0])
        lateral_world = _qrotate(pelvis_quat, lat_dir)
        # pole = gravity + 0.3 * (desired hang-elbow horizontal direction).  The
        # desired direction comes from the same per-side offsets the P0 correction
        # uses (elbow ~5cm medial + fwd_off forward/back), so the phi-grid's natural
        # azimuth IS the corrected target and the two stop fighting (previously the
        # pole's fwd/lateral terms pinned the elbow ~15cm off the target).
        fwd_world = _qrotate(pelvis_quat, np.array([0.0, 0.0, 1.0]))
        fwd_off = XU_ELBOW_P0_FWD
        p0_lat = XU_ELBOW_P0_LAT_L if side == "Left" else XU_ELBOW_P0_LAT_R
        # hang direction must match the P0 correction target exactly:
        # target = shoulder + p0_lat*lat_dir2 + fwd_off*forward
        hang_dir_b = np.array([p0_lat * lat_dir[0], 0.0, fwd_off], dtype=float)
        n_hd = float(np.linalg.norm(hang_dir_b))
        if n_hd > 1e-6:
            hang_dir_b = hang_dir_b / n_hd
            hang_dir_w = _qrotate(pelvis_quat, hang_dir_b)
            pole = np.array([0.0, -1.0, 0.0]) + 0.3 * hang_dir_w
            pole = pole / float(np.linalg.norm(pole))
        else:
            pole = np.array([0.0, -1.0, 0.0])
        # pole-based solution circle. The lateral weight (0.5) keeps the pole away from the
        # hanging-arm direction, so the phi-0 reference (pdir) is well defined at rest and
        # does not flip (the collinearity that caused the forward elbow + jumps).
        circle = _circle_geometry(shoulder, wrist_pos, l1, l2, pole)
        # learned elbow azimuth (soft prior) -> constrained 1-DOF optimization with a
        # natural position envelope (no adduction past the midline, no elbow behind the back,
        # and - for hanging arms - no elbow poking forward)
        phi_learned = new_prev.get(side + "_phi_rest")
        if phi_learned is None and elbow_prior is not None:
            phi_learned = elbow_prior.predict(side, pelvis_quat, shoulder, wrist_pos, wrist_quat)
        prev_phi = new_prev.get(side + "_phi")
        phi = _elbow_phi_grid(shoulder, wrist_pos, l1, l2, pole, phi_learned, pelvis_quat, side,
                              prev_phi, circle=circle)
        if prev_phi is not None:
            dphi = np.arctan2(np.sin(phi - prev_phi), np.cos(phi - prev_phi))
            dphi = float(np.clip(dphi, -0.35, 0.35))  # cap: <=20 deg/frame
            g = circle
            if g is not None:
                _, _, u0s, _, hh, Cc, pdir, perp = g
                sign = 1.0 if side == "Left" else -1.0
                vertical = float(np.dot(u0s, np.array([0.0, -1.0, 0.0]))) > 0.90
                accepted = None
                wrist_below_env = float(wrist_pos[1] - shoulder[1]) < 0.0
                for alpha in (0.5, 0.25, 0.125, 0.0625, 0.0):
                    phi_try = prev_phi + alpha * dphi
                    Et = Cc + hh * (np.cos(phi_try) * pdir + np.sin(phi_try) * perp)
                    Ebt = _qrotate(_qconj(pelvis_quat), Et - shoulder)
                    env_ok = (sign * Ebt[0] <= 0.06 and Ebt[2] >= -0.25
                              and (not vertical or Ebt[2] <= 0.05)
                              and (not wrist_below_env or Ebt[1] <= 0.02))
                    if env_ok:
                        accepted = phi_try
                        break  # smoothing accepted only inside the natural envelope
                if accepted is not None:
                    phi = accepted
                else:
                    phi = prev_phi + 0.0625 * dphi  # continuity fallback (no per-frame snap)
        new_prev[side + "_phi"] = phi
        prev = new_prev.get(side)
        elbow_pos = _two_bone_ik(shoulder, wrist_pos, l1, l2, prev_elbow=prev, pole=pole, phi=phi,
                                 circle=circle)
        # natural-hang elbow: the real PICO skeleton hangs with the elbow ~2 cm FORWARD of
        # the shoulder and the wrist ~5 cm back (forearm pointing backward, 33 deg bend).
        # The pole puts the synth elbow behind the wrist, so for a hanging arm move the
        # elbow's horizontal position to ~2 cm in front of the shoulder (y stays from IK).
        depth2 = float(shoulder[1] - wrist_pos[1])
        if depth2 > 0.10:
            fwd_w = _qrotate(pelvis_quat, np.array([0.0, 0.0, 1.0]))
            fnh = float(np.linalg.norm([fwd_w[0], fwd_w[2]]))
            if fnh > 1e-4:
                fwd_h = np.asarray([fwd_w[0], fwd_w[2]], dtype=float) / fnh
                w2 = float(min(1.0, (depth2 - 0.05) / 0.20))
                lat_dir2 = np.array([-1.0, 0.0, 0.0]) if side == "Left" else np.array([1.0, 0.0, 0.0])
                lat_w2 = _qrotate(pelvis_quat, lat_dir2)
                fwd_off = XU_ELBOW_P0_FWD
                p0_lat = XU_ELBOW_P0_LAT_L if side == "Left" else XU_ELBOW_P0_LAT_R
                target_eh = np.asarray([shoulder[0] + fwd_off * fwd_h[0] + p0_lat * lat_w2[0],
                                        shoulder[2] + fwd_off * fwd_h[1] + p0_lat * lat_w2[1]], dtype=float)
                elbow_pos = np.asarray(elbow_pos, dtype=float)
                dx = w2 * (target_eh[0] - elbow_pos[0])
                dz = w2 * (target_eh[1] - elbow_pos[1])
                dy = -XU_ELBOW_P0_DY * w2           # lower the hanging elbow (self-check knob)
                dm = float(np.hypot(dx, dz))
                if dm > 0.02:  # per-frame cap: no 10+ cm single-frame elbow-target jumps
                    dx *= 0.02 / dm; dz *= 0.02 / dm
                elbow_pos[0] += dx
                elbow_pos[1] += dy
                elbow_pos[2] += dz
                # (no phi-state sync needed: the pole now points at the corrected target,
                # so the grid's natural azimuth already is the target)
        # output continuity guard: when the wrist passes near the shoulder the IK is
        # ill-conditioned and the elbow can snap; halve any >12 cm single-frame move.
        if prev is not None and elbow_pos is not None:
            d_e = float(np.linalg.norm(elbow_pos - prev))
            if d_e > 0.12:
                elbow_pos = prev + 0.5 * (elbow_pos - prev)
        new_prev[side] = elbow_pos
        fore_dir = wrist_pos - elbow_pos
        elbow_quat = _forearm_frame_quat(fore_dir) if float(np.linalg.norm(fore_dir)) > 1e-6 \
            else np.asarray(body[side + "_Elbow"][1], dtype=float)
        geo_calib = template.geo_to_elbow.get(side)
        if geo_calib is not None:
            elbow_quat = _qmul(elbow_quat, geo_calib)
        # continuity limit: if the elbow frame flipped ~180 deg vs the previous frame
        # (world-up degeneracy of the geometric forearm frame), flip it back about the
        # forearm axis so the elbow never appears to snap outward.
        prev_q = new_prev.get(side + "_q")
        if prev_q is not None and _quat_angle(elbow_quat, prev_q) > np.pi / 2:
            fh = float(np.linalg.norm(fore_dir))
            elbow_quat = _flip_about_axis(elbow_quat, fore_dir / fh) if fh > 1e-6 else prev_q
        new_prev[side + "_q"] = _qnormalize(elbow_quat)
        body[side + "_Elbow"] = [elbow_pos.tolist(), _qnormalize(elbow_quat).tolist()]
        body[side + "_Wrist"] = [wrist_pos.tolist(), _qnormalize(wrist_quat).tolist()]
        body[side + "_Hand"] = [wrist_pos.tolist(), _qnormalize(wrist_quat).tolist()]

    upper = {name: body[name] for name in UPPER_BODY_JOINT_NAMES}
    return upper, new_prev


def facing_yaw(headset7, synth_rh, prev_yaw=None):
    """Yaw (rad) so the robot's model-forward (+x) faces the operator.

    The raw headset forward (+z, Unity frame) is mapped to the GMR right-handed frame,
    projected onto the horizontal plane, and the sign is disambiguated by the synthesized
    skeleton's left/right side: if the human's left wrist lies on the robot-left side of
    the pelvis, the robot faces the operator; otherwise it is facing their back (+= pi).

    Robustness (deadband + continuity): when the left wrist is near the body centerline
    (crossed arms / hands in front) the dot-product sign is noise and can flip the facing
    by 180 deg frame-to-frame, swinging the whole arm; in that case (and for any >90 deg
    step) the previous yaw is kept.
    """
    rot_mat = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])
    rot_quat = R.from_matrix(rot_mat).as_quat(scalar_first=True)
    q_raw = np.asarray([headset7[6], headset7[3], headset7[4], headset7[5]], dtype=float)
    q_rh = quat_mul_np(rot_quat, q_raw, scalar_first=True)
    f = R.from_quat(q_rh, scalar_first=True).apply([0.0, 0.0, 1.0])
    fh = f.copy()
    fh[2] = 0.0
    n = float(np.linalg.norm(fh))
    base = 0.0
    if n > 1e-3:
        fh /= n
        base = float(np.arctan2(fh[1], fh[0]))
    left = np.array([-np.sin(base), np.cos(base), 0.0])
    pel = np.asarray(synth_rh["Pelvis"][0], dtype=float)
    lw = np.asarray(synth_rh["Left_Wrist"][0], dtype=float)
    side = float(np.dot(lw - pel, left))
    if side < -0.04:
        base += np.pi
    elif abs(side) <= 0.04 and prev_yaw is not None:
        base = prev_yaw  # ambiguous crossing: keep the previous facing
    yaw = float(np.arctan2(np.sin(base), np.cos(base)))
    if prev_yaw is not None:
        dy = float(np.arctan2(np.sin(yaw - prev_yaw), np.cos(yaw - prev_yaw)))
        if abs(dy) > np.pi / 2:
            return prev_yaw  # reject a 180-deg flip
        yaw = prev_yaw + 0.5 * dy  # half-step toward the new yaw (smooth)
    return yaw


class XRobotStreamer:
    def __init__(self, template_path=None, human_height=1.7, upper_body_only=False, debug=False,
                 elbow_prior_path=None):
        xrt.init()
        self.template_path = template_path or DEFAULT_TEMPLATE_PATH
        self.human_height = human_height
        self.upper_body_only = upper_body_only
        self.debug = debug
        self.elbow_prior = None
        if elbow_prior_path:
            try:
                from .elbow_prior import ElbowPrior
                self.elbow_prior = ElbowPrior.load(elbow_prior_path)
                print(f"[XRobotStreamer] elbow prior loaded: {elbow_prior_path}")
            except Exception as e:
                print(f"[XRobotStreamer] failed to load elbow prior {elbow_prior_path}: {e}")

        template = StandingTemplate.load(self.template_path)
        if template is None:
            print(f"[XRobotStreamer] 未找到模板 {self.template_path}，使用参数化默认站姿 (h={human_height})")
            template = StandingTemplate.parametric(human_height)
        self.template = template
        self.prev_elbows = {}

        # Joint names for reference
        self.body_joint_names = [
                "Pelvis", "Left_Hip", "Right_Hip", "Spine1", "Left_Knee", "Right_Knee",
                "Spine2", "Left_Ankle", "Right_Ankle", "Spine3", "Left_Foot", "Right_Foot",
                "Neck", "Left_Collar", "Right_Collar", "Head", "Left_Shoulder", "Right_Shoulder",
                "Left_Elbow", "Right_Elbow", "Left_Wrist", "Right_Wrist", "Left_Hand", "Right_Hand"
            ]


        self.hand_joint_names = [
            "Wrist", "Palm",
            "ThumbMetacarpal", "ThumbProximal", "ThumbDistal", "ThumbTip",
            "IndexMetacarpal", "IndexProximal", "IndexIntermediate", "IndexDistal", "IndexTip",
            "MiddleMetacarpal", "MiddleProximal", "MiddleIntermediate", "MiddleDistal", "MiddleTip", 
            "RingMetacarpal", "RingProximal", "RingIntermediate", "RingDistal", "RingTip",
            "LittleMetacarpal", "LittleProximal", "LittleIntermediate", "LittleDistal", "LittleTip"
        ]

        self.last_left_hand_data = {}
        self.last_right_hand_data = {}

    
    def get_controller_data(self):
        left_trigger = xrt.get_left_trigger()
        right_trigger = xrt.get_right_trigger()

        left_grip = xrt.get_left_grip()
        right_grip = xrt.get_right_grip()

        # Buttons
        a_button_pressed = xrt.get_A_button()
        b_button_pressed = xrt.get_B_button()
        x_button_pressed = xrt.get_X_button()
        y_button_pressed = xrt.get_Y_button()

        # Axes
        left_axis = xrt.get_left_axis()
        right_axis = xrt.get_right_axis()

        left_axis_click = xrt.get_left_axis_click()
        right_axis_click = xrt.get_right_axis_click()

        # Timestamp
        timestamp = xrt.get_time_stamp_ns()

        # return
        return {
            'LeftController': {
                'index_trig': left_trigger,
                'grip': left_grip,
                'key_one': x_button_pressed,
                'key_two': y_button_pressed,
                'axis': left_axis,
                'axis_click': left_axis_click,
            },
            'RightController': {
                'index_trig': right_trigger,
                'grip': right_grip,
                'key_one': a_button_pressed,
                'key_two': b_button_pressed,
                'axis': right_axis,
                'axis_click': right_axis_click,
            },
            'timestamp': timestamp,
        }


    def get_headset_pose(self):
        headset_pose = xrt.get_headset_pose()
        return headset_pose
    
    def get_left_controller_pose(self):
        left_pose = xrt.get_left_controller_pose()
        return left_pose
    
    def get_right_controller_pose(self):
        right_pose = xrt.get_right_controller_pose()
        return right_pose

    def get_left_hand_data(self):
        left_hand_tracking_state = xrt.get_left_hand_tracking_state()
        left_hand_is_active = xrt.get_left_hand_is_active()
        hand_data_dict = {}
        for i, joint_name in enumerate(self.hand_joint_names):
            pos = [left_hand_tracking_state[i][0], left_hand_tracking_state[i][1], left_hand_tracking_state[i][2]] # xyz
            rot = [left_hand_tracking_state[i][6], left_hand_tracking_state[i][3], left_hand_tracking_state[i][4], left_hand_tracking_state[i][5]] # scalar first wxyz
            hand_data_dict["LeftHand" + joint_name] = [pos, rot]
        hand_data_dict = self.coordinate_transform_unity_data(hand_data_dict).copy()
        return left_hand_is_active, hand_data_dict
    
    def get_right_hand_data(self):
        right_hand_tracking_state = xrt.get_right_hand_tracking_state()
        right_hand_is_active = xrt.get_right_hand_is_active()
        hand_data_dict = {}
        for i, joint_name in enumerate(self.hand_joint_names):
            pos = [right_hand_tracking_state[i][0], right_hand_tracking_state[i][1], right_hand_tracking_state[i][2]] # xyz
            rot = [right_hand_tracking_state[i][6], right_hand_tracking_state[i][3], right_hand_tracking_state[i][4], right_hand_tracking_state[i][5]] # scalar first wxyz
            hand_data_dict["RightHand" + joint_name] = [pos, rot]
        hand_data_dict = self.coordinate_transform_unity_data(hand_data_dict).copy()
        return right_hand_is_active, hand_data_dict

    def get_raw_body_data(self):

        if not xrt.is_body_data_available():
            # print("No body tracking data. return None", end="\r")
            return None, None, None, None, None

        if xrt.is_body_data_available():
            
            body_poses = xrt.get_body_joints_pose() # list of [x, y, z, qx, qy, qz, qw]
            body_velocities = xrt.get_body_joints_velocity() # vx, vy, vz, wx, wy, wz
            body_accelerations = xrt.get_body_joints_acceleration() # ax, ay, az, wax, way, waz
            imu_timestamps = xrt.get_body_joints_timestamp() # list of [timestamp]
            body_timestamp = xrt.get_body_timestamp_ns() # timestamp in ns

            return body_poses, body_velocities, body_accelerations, imu_timestamps, body_timestamp
        else:
            raise Exception("Body tracking data is not available!")
    
    def set_upper_body_only(self, flag):
        self.upper_body_only = bool(flag)

    def close(self):
        """Release the PICO SDK (must be called before process exit, else std::terminate)."""
        try:
            xrt.close()
        except Exception as e:
            print(f"[XRobotStreamer] close failed: {e}")

    def _build_raw_body_dict(self):
        """Build the body joint dict (raw PICO frame) from live body tracking data."""
        body_poses, _, _, _, _ = self.get_raw_body_data()
        if body_poses is None:
            return None
        body_pose_dict = {}
        for i, joint_name in enumerate(self.body_joint_names):
            pos = [body_poses[i][0], body_poses[i][1], body_poses[i][2]]  # xyz
            rot = [body_poses[i][6], body_poses[i][3], body_poses[i][4], body_poses[i][5]]  # scalar first
            body_pose_dict[joint_name] = [pos, rot]
        return body_pose_dict

    def _extract_upper_body(self, body_dict):
        """Take the 9 upper-body joints from a full 24-joint dict (raw PICO frame)."""
        return {name: body_dict[name] for name in UPPER_BODY_JOINT_NAMES if name in body_dict}

    def _live_body_is_sane(self, raw_body):
        """Reject degenerate live body frames (pelvis above head, invalid quats)."""
        if raw_body is None:
            return False
        head = np.asarray(raw_body["Head"][0], dtype=float)
        pelvis = np.asarray(raw_body["Pelvis"][0], dtype=float)
        if pelvis[1] > head[1] - 0.15:  # raw PICO frame: y up, pelvis below head
            return False
        for name in UPPER_BODY_JOINT_NAMES:
            q = np.asarray(raw_body[name][1], dtype=float)
            if abs(float(np.linalg.norm(q)) - 1.0) > 0.1:
                return False
        return True

    def _make_upper_synth_body(self):
        """Synthesize the 9-joint upper body from live head + controllers + standing template."""
        headset_pose = xrt.get_headset_pose()
        if not _pose7_valid(headset_pose):
            return None
        head_q = np.asarray(headset_pose[3:7], dtype=float)
        if abs(float(np.linalg.norm(head_q)) - 1.0) > 0.05:  # reject non-unit head quat
            return None
        left_pose = xrt.get_left_controller_pose()
        right_pose = xrt.get_right_controller_pose()
        body, self.prev_elbows = synthesize_upper_body(
            headset_pose, left_pose, right_pose, self.template, self.prev_elbows,
            elbow_prior=self.elbow_prior)
        return body

    def get_upper_body_dict(self):
        """9-joint upper body (raw PICO frame): extract from live full body when available
        and sane, otherwise synthesize from headset + controllers + template."""
        raw = self._build_raw_body_dict()
        if raw is not None and self._live_body_is_sane(raw):
            return self._extract_upper_body(raw)
        return self._make_upper_synth_body()

    def get_processed_body_data(self, use_hands=False):

        if self.upper_body_only:
            body_pose_dict = self.get_upper_body_dict()
        else:
            body_poses, _, _, _, _ = self.get_raw_body_data()
            if body_poses is None:
                return None
            body_pose_dict = {}
            for i, joint_name in enumerate(self.body_joint_names):
                pos = [body_poses[i][0], body_poses[i][1], body_poses[i][2]]  # xyz
                rot = [body_poses[i][6], body_poses[i][3], body_poses[i][4], body_poses[i][5]]  # scalar first
                body_pose_dict[joint_name] = [pos, rot]

        if body_pose_dict is None:
            return None

        # from unity coordinate to right-hand coordinate
        body_pose_dict = self.coordinate_transform_unity_data(body_pose_dict).copy()

        if use_hands:
            left_hand_is_active, left_hand_data = self.get_left_hand_data()
            right_hand_is_active, right_hand_data = self.get_right_hand_data()
            if left_hand_is_active:
                body_pose_dict.update(left_hand_data)
                self.last_left_hand_data = left_hand_data
            else:
                # use last frame's hand data
                body_pose_dict.update(self.last_left_hand_data)
                
            if right_hand_is_active:
                body_pose_dict.update(right_hand_data)
                self.last_right_hand_data = right_hand_data
            else:
                # use last frame's hand data
                body_pose_dict.update(self.last_right_hand_data)

        return body_pose_dict


    def coordinate_transform_unity_data(self, body_pose_dict):

        for body_name, value in body_pose_dict.items():
                x, y, z = value[0]
                qw, qx, qy, qz = value[1]

                # from unity coordinate to right-hand coordinate
                rotation_matrix = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])
                rotation_quat = R.from_matrix(rotation_matrix).as_quat(scalar_first=True)
                orientation = quat_mul_np(rotation_quat, np.array([qw, qx, qy, qz]), scalar_first=True)
                position = np.array([x, y, z]) @ rotation_matrix.T  # cm to m

                body_pose_dict[body_name][0] = position.tolist()
                body_pose_dict[body_name][1] = orientation.tolist()

        return body_pose_dict
    
    def get_current_frame(self):
        body_pose_dict = self.get_processed_body_data()
        left_hand_data = self.get_left_hand_data()
        right_hand_data = self.get_right_hand_data()
        controller_data = self.get_controller_data()
        headset_pose = self.get_headset_pose()
        return body_pose_dict, left_hand_data, right_hand_data, controller_data, headset_pose


class XRobotRecorder:
    """
    Load and process recorded XRobot data from MP4 and TXT files.
    Similar to XRobotStreamer but for recorded data instead of real-time streaming.
    Data is preprocessed during initialization for better performance.
    """
    
    def __init__(self, mp4_path, txt_path):
        """
        Initialize the recorder with paths to MP4 and TXT files.
        All data is preprocessed during initialization.
        
        Args:
            mp4_path: Path to the MP4 video file
            txt_path: Path to the tracking data TXT file
        """
        self.mp4_path = mp4_path
        self.txt_path = txt_path
        
        # Joint names (same as XRobotStreamer)
        self.body_joint_names = [
            "Pelvis", "Left_Hip", "Right_Hip", "Spine1", "Left_Knee", "Right_Knee",
            "Spine2", "Left_Ankle", "Right_Ankle", "Spine3", "Left_Foot", "Right_Foot",
            "Neck", "Left_Collar", "Right_Collar", "Head", "Left_Shoulder", "Right_Shoulder",
            "Left_Elbow", "Right_Elbow", "Left_Wrist", "Right_Wrist", "Left_Hand", "Right_Hand"
        ]
        
        self.hand_joint_names = [
            "Wrist", "Palm",
            "ThumbMetacarpal", "ThumbProximal", "ThumbDistal", "ThumbTip",
            "IndexMetacarpal", "IndexProximal", "IndexIntermediate", "IndexDistal", "IndexTip",
            "MiddleMetacarpal", "MiddleProximal", "MiddleIntermediate", "MiddleDistal", "MiddleTip", 
            "RingMetacarpal", "RingProximal", "RingIntermediate", "RingDistal", "RingTip",
            "LittleMetacarpal", "LittleProximal", "LittleIntermediate", "LittleDistal", "LittleTip"
        ]
        
        # Load raw data
        self.video_frames = []
        self.tracking_data = []
        self.camera_params = None
        self.initial_timestamp = 0
        
        # Preprocessed data (indexed by frame)
        self.processed_body_data = []
        self.processed_left_hand_data = []
        self.processed_right_hand_data = []
        self.processed_controller_data = []
        self.processed_headset_poses = []
        
        self._load_and_process_data()
        
        # Initialize legacy support for backwards compatibility
        self.__init_legacy_support()
    
    def _load_and_process_data(self):
        """Load MP4 and TXT data, then preprocess all frames"""
        print(f"Loading MP4: {self.mp4_path}")
        print(f"Loading TXT: {self.txt_path}")
        
        # Load video
        self._load_mp4()
        
        # Load tracking data
        self._load_tracking_data()
        
        print(f"Loaded {len(self.video_frames)} video frames and {len(self.tracking_data)} tracking frames")
        
        # Preprocess all data
        self._preprocess_all_data()
        
        print(f"Preprocessed {len(self.processed_body_data)} frames")
    
    def _load_mp4(self):
        """Load MP4 video file"""
        cap = cv2.VideoCapture(self.mp4_path)
        if not cap.isOpened():
            raise ValueError(f"Could not open video file: {self.mp4_path}")
        
        frames = []
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            frames.append(frame)
        
        cap.release()
        self.video_frames = frames
    
    def _load_tracking_data(self):
        """Load and parse tracking data from TXT file"""
        if not os.path.exists(self.txt_path):
            raise FileNotFoundError(f"Tracking data file not found: {self.txt_path}")
        
        with open(self.txt_path, 'r') as f:
            lines = f.readlines()
        
        # First line contains camera parameters and initial timestamp
        if lines:
            try:
                self.camera_params = json.loads(lines[0].strip())
                # Extract initial timestamp for video frame alignment
                self.initial_timestamp = self.camera_params.get("timeStampNs", 0)
            except json.JSONDecodeError:
                print("Warning: Could not parse camera parameters from first line")
                self.camera_params = {}
                self.initial_timestamp = 0
        
        # Remaining lines contain frame tracking data
        for line_idx, line in enumerate(lines[1:], 1):
            line = line.strip()
            if line:
                try:
                    frame_data = json.loads(line)
                    self.tracking_data.append(frame_data)
                except json.JSONDecodeError as e:
                    print(f"Warning: Could not parse tracking data on line {line_idx + 1}: {e}")
    
    def _find_nearest_mocap_frame(self, video_frame_timestamp):
        """Find the nearest mocap frame for a given video frame timestamp"""
        if not self.tracking_data:
            return None
            
        min_diff = float('inf')
        nearest_frame = None
        
        for frame_data in self.tracking_data:
            frame_timestamp = frame_data.get("timeStampNs", 0)
            diff = abs(frame_timestamp - video_frame_timestamp)
            
            if diff < min_diff:
                min_diff = diff
                nearest_frame = frame_data
        
        return nearest_frame
    
    def _preprocess_all_data(self):
        """Preprocess all tracking data for all frames with timestamp alignment"""
        total_video_frames = len(self.video_frames)
        
        # Initialize preprocessed data lists
        self.processed_body_data = []
        self.processed_left_hand_data = []
        self.processed_right_hand_data = []
        self.processed_controller_data = []
        self.processed_headset_poses = []
        
        # Video frame duration in nanoseconds (30 fps = 33.33ms per frame)
        frame_duration_ns = int(1e9 / 30)  # 33,333,333 ns per frame
        
        # Process each video frame
        for video_frame_idx in range(total_video_frames):
            # Calculate expected timestamp for this video frame
            video_frame_timestamp = self.initial_timestamp + (video_frame_idx * frame_duration_ns)
            
            # Find the nearest mocap frame for this video frame timestamp
            nearest_mocap_frame = self._find_nearest_mocap_frame(video_frame_timestamp)
            
            if nearest_mocap_frame is None:
                # If no mocap data available, use empty data
                self.processed_body_data.append({})
                self.processed_left_hand_data.append({})
                self.processed_right_hand_data.append({})
                self.processed_controller_data.append({
                    'LeftController': {
                        'index_trig': 0.0, 'grip': 0.0, 'key_one': False,
                        'axis': [0.0, 0.0], 'axis_click': False,
                    },
                    'RightController': {
                        'index_trig': 0.0, 'grip': 0.0, 'key_one': False,
                        'axis': [0.0, 0.0], 'axis_click': False,
                    },
                    'timestamp': video_frame_timestamp,
                })
                self.processed_headset_poses.append(None)
                continue
            
            # Process body data
            body_data = self._process_body_data(nearest_mocap_frame)
            self.processed_body_data.append(body_data)
            
            # Process hand data (with fallback logic)
            left_hand_data = self._process_left_hand_data(nearest_mocap_frame, video_frame_idx)
            right_hand_data = self._process_right_hand_data(nearest_mocap_frame, video_frame_idx)
            self.processed_left_hand_data.append(left_hand_data)
            self.processed_right_hand_data.append(right_hand_data)
            
            # Process controller data
            controller_data = self._process_controller_data(nearest_mocap_frame)
            self.processed_controller_data.append(controller_data)
            
            # Process headset pose
            headset_pose = self._process_headset_pose(nearest_mocap_frame)
            self.processed_headset_poses.append(headset_pose)
    
    def get_total_frames(self):
        """Get total number of frames (based on video frames since we align to video timing)"""
        return len(self.video_frames)
    
    def get_video_frame(self, idx):
        """Get video frame at specific index"""
        if 0 <= idx < len(self.video_frames):
            return self.video_frames[idx]
        return None
    
    def _process_body_data(self, frame_data):
        """Process body data for a single frame"""
        body_pose_dict = {}
        
        # Extract body joint data
        if "Body" in frame_data:
            body_data = frame_data["Body"]
            if "joints" in body_data:
                joints = body_data["joints"]
                
                # Parse each joint
                for i, joint_name in enumerate(self.body_joint_names):
                    if i < len(joints):
                        joint_data = joints[i]
                        
                        # Extract position and rotation from 'p' field
                        if "p" in joint_data:
                            p_str = joint_data["p"]
                            # Format: "px,py,pz,qx,qy,qz,qw" (position first, then rotation)
                            try:
                                values = [float(x) for x in p_str.split(',')]
                                if len(values) >= 7:
                                    px, py, pz, qx, qy, qz, qw = values[:7]
                                    pos = [px, py, pz]
                                    rot = [qw, qx, qy, qz]  # scalar first
                                    
                                    body_pose_dict[joint_name] = [pos, rot]
                            except (ValueError, IndexError) as e:
                                print(f"Warning: Could not parse joint {i} ({joint_name}): {e}")
                                continue
        
        # Apply coordinate transformation
        body_pose_dict = self.coordinate_transform_unity_data(body_pose_dict).copy()
        return body_pose_dict
    
    def get_processed_body_data(self, idx, use_hands=False):
        """Get processed body data for specific frame index"""
        if not (0 <= idx < len(self.processed_body_data)):
            return {}
        
        body_data = self.processed_body_data[idx].copy()
        
        # Add hand data if requested
        if use_hands:
            left_hand_data = self.get_left_hand_data(idx)
            right_hand_data = self.get_right_hand_data(idx)
            body_data.update(left_hand_data)
            body_data.update(right_hand_data)
        else:
            left_hand_data = {}
            right_hand_data = {}
        
        return body_data, left_hand_data, right_hand_data
    
    def _process_left_hand_data(self, frame_data, frame_idx):
        """Process left hand data for a single frame with fallback to previous frame"""
        hand_data_dict = {}
        
        if "Hand" not in frame_data:
            # Use previous frame's data if available
            if frame_idx > 0 and frame_idx - 1 < len(self.processed_left_hand_data):
                return self.processed_left_hand_data[frame_idx - 1].copy()
            return {}
        
        hand_data = frame_data["Hand"]
        
        if "leftHand" in hand_data and "HandJointLocations" in hand_data["leftHand"]:
            joint_locations = hand_data["leftHand"]["HandJointLocations"]
            is_active = hand_data["leftHand"].get("isActive", True)
            
            # If hand is not active, use previous frame's data
            if not is_active:
                if frame_idx > 0 and frame_idx - 1 < len(self.processed_left_hand_data):
                    return self.processed_left_hand_data[frame_idx - 1].copy()
                # If no previous frame, continue with empty data
                return {}
            
            for i, joint_name in enumerate(self.hand_joint_names):
                if i < len(joint_locations):
                    joint_data = joint_locations[i]
                    
                    if "p" in joint_data:
                        p_str = joint_data["p"]
                        # Format: "px,py,pz,qx,qy,qz,qw" (position first, then rotation)
                        try:
                            values = [float(x) for x in p_str.split(',')]
                            if len(values) >= 7:
                                px, py, pz, qx, qy, qz, qw = values[:7]
                                pos = [px, py, pz]
                                rot = [qw, qx, qy, qz]  # scalar first
                                
                                hand_data_dict["LeftHand" + joint_name] = [pos, rot]
                        except (ValueError, IndexError) as e:
                            print(f"Warning: Could not parse left hand joint {i} ({joint_name}): {e}")
                            continue
        
        hand_data_dict = self.coordinate_transform_unity_data(hand_data_dict).copy()
        return hand_data_dict
    
    def get_left_hand_data(self, idx):
        """Get left hand tracking data for specific frame index"""
        if 0 <= idx < len(self.processed_left_hand_data):
            return self.processed_left_hand_data[idx].copy()
        return {}
    
    def _process_right_hand_data(self, frame_data, frame_idx):
        """Process right hand data for a single frame with fallback to previous frame"""
        hand_data_dict = {}
        
        if "Hand" not in frame_data:
            # Use previous frame's data if available
            if frame_idx > 0 and frame_idx - 1 < len(self.processed_right_hand_data):
                return self.processed_right_hand_data[frame_idx - 1].copy()
            return {}
        
        hand_data = frame_data["Hand"]
        
        if "rightHand" in hand_data and "HandJointLocations" in hand_data["rightHand"]:
            joint_locations = hand_data["rightHand"]["HandJointLocations"]
            is_active = hand_data["rightHand"].get("isActive", True)
            
            # If hand is not active, use previous frame's data
            if not is_active:
                if frame_idx > 0 and frame_idx - 1 < len(self.processed_right_hand_data):
                    return self.processed_right_hand_data[frame_idx - 1].copy()
                # If no previous frame, continue with empty data
                return {}
            
            for i, joint_name in enumerate(self.hand_joint_names):
                if i < len(joint_locations):
                    joint_data = joint_locations[i]
                    
                    if "p" in joint_data:
                        p_str = joint_data["p"]
                        # Format: "px,py,pz,qx,qy,qz,qw" (position first, then rotation)
                        try:
                            values = [float(x) for x in p_str.split(',')]
                            if len(values) >= 7:
                                px, py, pz, qx, qy, qz, qw = values[:7]
                                pos = [px, py, pz]
                                rot = [qw, qx, qy, qz]  # scalar first
                                
                                hand_data_dict["RightHand" + joint_name] = [pos, rot]
                        except (ValueError, IndexError) as e:
                            print(f"Warning: Could not parse right hand joint {i} ({joint_name}): {e}")
                            continue
        
        hand_data_dict = self.coordinate_transform_unity_data(hand_data_dict).copy()
        return hand_data_dict
    
    def get_right_hand_data(self, idx):
        """Get right hand tracking data for specific frame index"""
        if 0 <= idx < len(self.processed_right_hand_data):
            return self.processed_right_hand_data[idx].copy()
        return {}
    
    def _process_controller_data(self, frame_data):
        """Process controller data for a single frame"""
        if "Controller" not in frame_data:
            return {
                'LeftController': {
                    'index_trig': 0.0,
                    'grip': 0.0,
                    'key_one': False,
                    'axis': [0.0, 0.0],
                    'axis_click': False,
                },
                'RightController': {
                    'index_trig': 0.0,
                    'grip': 0.0,
                    'key_one': False,
                    'axis': [0.0, 0.0],
                    'axis_click': False,
                },
                'timestamp': 0,
            }
        
        controller_data = frame_data["Controller"]
        
        # Parse controller data structure
        result = {
            'LeftController': {
                'index_trig': 0.0,
                'grip': 0.0,
                'key_one': False,
                'axis': [0.0, 0.0],
                'axis_click': False,
            },
            'RightController': {
                'index_trig': 0.0,
                'grip': 0.0,
                'key_one': False,
                'axis': [0.0, 0.0],
                'axis_click': False,
            },
            'timestamp': frame_data.get("timeStampNs", 0),
        }
        
        # Parse left controller
        if "leftController" in controller_data:
            left_ctrl = controller_data["leftController"]
            if "inputState" in left_ctrl:
                input_state = left_ctrl["inputState"]
                result['LeftController']['index_trig'] = input_state.get("indexTrigger", 0.0)
                result['LeftController']['grip'] = input_state.get("handTrigger", 0.0)
                result['LeftController']['key_one'] = input_state.get("menuButton", False)
                thumbstick = input_state.get("thumbstick", {})
                result['LeftController']['axis'] = [thumbstick.get("x", 0.0), thumbstick.get("y", 0.0)]
                result['LeftController']['axis_click'] = input_state.get("thumbstickClick", False)
        
        # Parse right controller
        if "rightController" in controller_data:
            right_ctrl = controller_data["rightController"]
            if "inputState" in right_ctrl:
                input_state = right_ctrl["inputState"]
                result['RightController']['index_trig'] = input_state.get("indexTrigger", 0.0)
                result['RightController']['grip'] = input_state.get("handTrigger", 0.0)
                result['RightController']['key_one'] = input_state.get("menuButton", False)
                thumbstick = input_state.get("thumbstick", {})
                result['RightController']['axis'] = [thumbstick.get("x", 0.0), thumbstick.get("y", 0.0)]
                result['RightController']['axis_click'] = input_state.get("thumbstickClick", False)
        
        return result
    
    def get_controller_data(self, idx):
        """Get controller data for specific frame index"""
        if 0 <= idx < len(self.processed_controller_data):
            return self.processed_controller_data[idx].copy()
        return {
            'LeftController': {
                'index_trig': 0.0,
                'grip': 0.0,
                'key_one': False,
                'axis': [0.0, 0.0],
                'axis_click': False,
            },
            'RightController': {
                'index_trig': 0.0,
                'grip': 0.0,
                'key_one': False,
                'axis': [0.0, 0.0],
                'axis_click': False,
            },
            'timestamp': 0,
        }
    
    def _process_headset_pose(self, frame_data):
        """Process headset pose for a single frame"""
        if "Head" not in frame_data:
            return None
        
        head_data = frame_data["Head"]
        if "pose" in head_data:
            pose_str = head_data["pose"]
            # Parse pose string format: "pos:(x,y,z) rot:(x,y,z,w)"
            try:
                pos_part, rot_part = pose_str.split(" rot:")
                pos_str = pos_part.replace("pos:(", "").replace(")", "")
                rot_str = rot_part.replace("(", "").replace(")", "")
                
                pos = [float(x.strip()) for x in pos_str.split(",")]
                rot = [float(x.strip()) for x in rot_str.split(",")]
                
                return {"position": pos, "rotation": rot}
            except:
                return None
        
        return None
    
    def get_headset_pose(self, idx):
        """Get headset pose for specific frame index"""
        if 0 <= idx < len(self.processed_headset_poses):
            return self.processed_headset_poses[idx]
        return None
    
    def coordinate_transform_unity_data(self, body_pose_dict):
        """
        Transform coordinates from Unity to right-hand coordinate system.
        Same as XRobotStreamer.coordinate_transform_unity_data()
        """
        for body_name, value in body_pose_dict.items():
            x, y, z = value[0]
            qw, qx, qy, qz = value[1]

            # from unity coordinate to right-hand coordinate
            rotation_matrix = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])
            rotation_quat = R.from_matrix(rotation_matrix).as_quat(scalar_first=True)
            orientation = quat_mul_np(rotation_quat, np.array([qw, qx, qy, qz]), scalar_first=True)
            position = np.array([x, y, z]) @ rotation_matrix.T

            body_pose_dict[body_name][0] = position.tolist()
            body_pose_dict[body_name][1] = orientation.tolist()

        return body_pose_dict
    
    def get_frame_data(self, idx):
        """Get all data for specific frame index"""
        if not (0 <= idx < self.get_total_frames()):
            return None
        
        body_pose_dict = self.get_processed_body_data(idx, use_hands=True)
        left_hand_data = self.get_left_hand_data(idx)
        right_hand_data = self.get_right_hand_data(idx)
        controller_data = self.get_controller_data(idx)
        video_frame = self.get_video_frame(idx)
        headset_pose = self.get_headset_pose(idx)
        
        return {
            'body_data': body_pose_dict,
            'left_hand_data': left_hand_data,
            'right_hand_data': right_hand_data,
            'controller_data': controller_data,
            'video_frame': video_frame,
            'headset_pose': headset_pose,
            'frame_index': idx
        }
    
    # Legacy methods for backwards compatibility
    def __init_legacy_support(self):
        """Initialize legacy support for current frame index"""
        self.current_frame_index = 0
    
    def set_frame_index(self, index):
        """Set current frame index (legacy method)"""
        if 0 <= index < self.get_total_frames():
            self.current_frame_index = index
        else:
            raise IndexError(f"Frame index {index} out of range [0, {self.get_total_frames()-1}]")
    
    def get_current_frame_data(self):
        """Get current frame's tracking data (legacy method)"""
        if hasattr(self, 'current_frame_index'):
            return self.get_frame_data(self.current_frame_index)
        return self.get_frame_data(0)
    
    def get_current_video_frame(self):
        """Get current video frame (legacy method)"""
        if hasattr(self, 'current_frame_index'):
            return self.get_video_frame(self.current_frame_index)
        return self.get_video_frame(0)
    
    def get_current_frame(self):
        """Get all data for current frame (legacy method)"""
        if hasattr(self, 'current_frame_index'):
            return self.get_frame_data(self.current_frame_index)
        return self.get_frame_data(0)
    
    def next_frame(self):
        """Move to next frame (legacy method)"""
        if not hasattr(self, 'current_frame_index'):
            self.current_frame_index = 0
        if self.current_frame_index < self.get_total_frames() - 1:
            self.current_frame_index += 1
            return True
        return False
    
    def prev_frame(self):
        """Move to previous frame (legacy method)"""
        if not hasattr(self, 'current_frame_index'):
            self.current_frame_index = 0
        if self.current_frame_index > 0:
            self.current_frame_index -= 1
            return True
        return False
    
    def reset(self):
        """Reset to first frame (legacy method)"""
        self.current_frame_index = 0
    
    def get_human_height(self):
        """
        Estimate human height by analyzing all body frames.
        Calculates the max difference between highest and lowest body parts for each frame,
        then returns the maximum height found across all frames.
        
        Returns:
            float: Estimated human height in meters
        """
        if not self.processed_body_data:
            print("Warning: No body data available for height estimation")
            return 1.7  # Default height
        
        max_height = 0.0
        valid_frames = 0
        
        for body_data in self.processed_body_data:
            if not body_data:
                continue
                
            # Extract Y coordinates (height) from all body joints
            y_positions = []
            for joint_data in body_data.values():
                if joint_data and len(joint_data) >= 1 and len(joint_data[0]) >= 3:
                    # joint_data format: [position, rotation]
                    # position format: [x, y, z]
                    y_pos = joint_data[0][1]  # Y coordinate
                    y_positions.append(y_pos)
            
            if len(y_positions) >= 2:  # Need at least 2 joints for height calculation
                frame_height = max(y_positions) - min(y_positions)
                if frame_height > max_height:
                    max_height = frame_height
                valid_frames += 1
        
        if valid_frames == 0:
            print("Warning: No valid frames found for height estimation")
            return 1.7  # Default height
        
        # Add some tolerance since we might not capture the full span (e.g., feet to head)
        estimated_height = max_height * 1.1  # Add 10% buffer
        
        # Clamp to reasonable human height range (1.4m to 2.2m)
        estimated_height = max(1.4, min(2.2, estimated_height))
        
        print(f"Estimated human height: {estimated_height:.2f}m (from {valid_frames} valid frames)")
        return estimated_height
