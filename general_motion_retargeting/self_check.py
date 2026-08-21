"""Self-check module: per-frame synth-vs-PICO error monitoring + auto-optimization.

Integrated into the upper-body retargeting pipeline (doc: 肘部运动优化-多优先级QP框架与实施记录.md §九).

Per frame (real-time, O(joints) vector ops):
  1. align the synthesized skeleton onto the real PICO pelvis (core-axis alignment)
  2. joint position errors (cm) + bone axis errors (deg) + elbow-above-shoulder
     kinematic rule + L/R symmetry metrics
  3. track the error history
  4. when windowed errors exceed thresholds, the optimizer mutates the QP/synthesis
     parameters (lam_virtual / per-side P0 lateral / elbow lowering) and re-runs
  5. a converged run renders the acceptance video with both skeletons + error overlays
"""
import json
import numpy as np
from scipy.spatial.transform import Rotation as R

JOINTS = ('Left_Shoulder', 'Left_Elbow', 'Left_Wrist',
          'Right_Shoulder', 'Right_Elbow', 'Right_Wrist')
BONES = (('Left_Shoulder', 'Left_Elbow'), ('Left_Elbow', 'Left_Wrist'),
         ('Left_Shoulder', 'Left_Wrist'),
         ('Right_Shoulder', 'Right_Elbow'), ('Right_Elbow', 'Right_Wrist'),
         ('Right_Shoulder', 'Right_Wrist'))
SYM_PAIRS = ((('Left_Shoulder', 'Left_Elbow'), ('Right_Shoulder', 'Right_Elbow')),
             (('Left_Elbow', 'Left_Wrist'), ('Right_Elbow', 'Right_Wrist')))

DEFAULT_THRESHOLDS = dict(
    joint_cm={'Left_Shoulder': 8.0, 'Right_Shoulder': 8.0,
              'Left_Elbow': 9.0, 'Right_Elbow': 9.0,
              'Left_Wrist': 14.0, 'Right_Wrist': 14.0},   # wrist is deliberately P0-corrected
    bone_deg=30.0, elbow_high=0.0, symmetry_deg=22.0, lat_skew_cm=8.0)


def _yaw(quat):
    """Yaw (rad) about the vertical (human frame, y-up) from a scalar-first quat."""
    r = R.from_quat([quat[1], quat[2], quat[3], quat[0]])
    fwd = r.apply([0.0, 0.0, 1.0])
    return float(np.arctan2(fwd[0], fwd[2]))


def align_to_reference(synth, real):
    """Translate + yaw-rotate the synth skeleton onto the real pelvis (core axes)."""
    sp = np.asarray(real['Pelvis'][0], float)
    ss = np.asarray(synth['Pelvis'][0], float)
    rot = R.from_rotvec([0.0, _yaw(real['Pelvis'][1]) - _yaw(synth['Pelvis'][1]), 0.0])
    out = {}
    for k, (pos, quat) in synth.items():
        p = rot.apply(np.asarray(pos, float) - ss) + sp
        q = (rot * R.from_quat([quat[1], quat[2], quat[3], quat[0]])).as_quat(scalar_first=True)
        out[k] = [np.asarray(p), np.asarray(q)]
    return out


def _bone_angle(a1, a2, b1, b2):
    u = np.asarray(a2, float) - np.asarray(a1, float)
    v = np.asarray(b2, float) - np.asarray(b1, float)
    nu, nv = float(np.linalg.norm(u)), float(np.linalg.norm(v))
    if nu < 1e-6 or nv < 1e-6:
        return 0.0
    return float(np.degrees(np.arccos(np.clip(np.dot(u / nu, v / nv), -1.0, 1.0))))


def compute_errors(synth, real):
    """Per-frame error dict (both bodies in the same frame, aligned beforehand)."""
    e = {}
    e['joint_cm'] = {j: float(np.linalg.norm(
        np.asarray(synth[j][0], float) - np.asarray(real[j][0], float)) * 100.0)
        for j in JOINTS if j in synth and j in real}
    e['bone_deg'] = {f'{a}->{b}': _bone_angle(real[a][0], real[b][0], synth[a][0], synth[b][0])
                     for a, b in BONES if a in synth and b in synth and a in real and b in real}
    # kinematic rule: when the SYNTH wrist is below the shoulder, the synth elbow must
    # stay at/below the shoulder line (the recovery-phase "elbow hovering above the
    # shoulder" issue).  The condition uses the synth's own wrist: during fast
    # transitions the controller (hence the synth wrist) legitimately lags above the
    # shoulder, and an elbow above the shoulder is then correct kinematics.
    for s in ('Left', 'Right'):
        S = np.asarray(synth[s + '_Shoulder'][0], float)
        Ws = np.asarray(synth[s + '_Wrist'][0], float)
        Es = np.asarray(synth[s + '_Elbow'][0], float)
        below = float(Ws[1] - S[1]) < 0.0          # human frame y-up
        e[f'elbow_high_{s}'] = float(max(0.0, Es[1] - S[1] - 0.02)) if below else 0.0
    # symmetry: mirror the left arm through the sagittal plane (x=0) vs the right arm
    e['symmetry_deg'] = {}
    for (la, lb), (ra, rb) in SYM_PAIRS:
        u = np.asarray(synth[lb][0], float) - np.asarray(synth[la][0], float)
        v = np.asarray(synth[rb][0], float) - np.asarray(synth[ra][0], float)
        nu, nv = float(np.linalg.norm(u)), float(np.linalg.norm(v))
        if nu > 1e-6 and nv > 1e-6:
            um = np.array([-u[0], u[1], u[2]]) / nu
            e['symmetry_deg'][f'{la}->{lb}'] = float(
                np.degrees(np.arccos(np.clip(np.dot(um, v / nv), -1.0, 1.0))))
    # lateral skew: L elbow lateral + R elbow lateral (0 = symmetric, + = R-dominant)
    try:
        el = np.asarray(synth['Left_Elbow'][0], float)[0] - np.asarray(synth['Left_Shoulder'][0], float)[0]
        er = np.asarray(synth['Right_Elbow'][0], float)[0] - np.asarray(synth['Right_Shoulder'][0], float)[0]
        e['lat_skew_cm'] = float((el + er) * 100.0)
    except KeyError:
        e['lat_skew_cm'] = 0.0
    return e


class SelfCheckMonitor:
    """Per-frame tracker + threshold checks + windowed auto-optimization records."""

    def __init__(self, thresholds=None):
        self.thr = dict(DEFAULT_THRESHOLDS)
        if thresholds:
            self.thr.update(thresholds)
        self.history = []
        self.violations = []
        self.adjust_log = []

    def update(self, frame_idx, errors):
        self.history.append({'frame': frame_idx, **errors})
        v = self.check(frame_idx, errors)
        if v:
            self.violations.extend(v)
        return v

    def check(self, frame_idx, errors):
        out = []
        jthr = self.thr['joint_cm']
        jthr = jthr if isinstance(jthr, dict) else {j: jthr for j in errors['joint_cm']}
        for j, d in errors['joint_cm'].items():
            if d > jthr.get(j, 99.0):
                out.append((frame_idx, f'joint {j}', round(d, 1), jthr.get(j, 99.0)))
        for b, d in errors['bone_deg'].items():
            if d > self.thr['bone_deg']:
                out.append((frame_idx, f'bone {b}', round(d, 1), self.thr['bone_deg']))
        for s in ('Left', 'Right'):
            d = errors.get(f'elbow_high_{s}', 0.0)
            if d > self.thr['elbow_high']:
                out.append((frame_idx, f'elbow_high {s}', round(d, 2), self.thr['elbow_high']))
        for b, d in errors['symmetry_deg'].items():
            if d > self.thr['symmetry_deg']:
                out.append((frame_idx, f'symmetry {b}', round(d, 1), self.thr['symmetry_deg']))
        return out

    def converged(self):
        return len(self.violations) == 0

    def window_stats(self, lo=0, hi=None):
        h = self.history[lo:hi]
        if not h:
            return {}
        j = {k: float(np.mean([f['joint_cm'][k] for f in h])) for k in h[0]['joint_cm']}
        b = {k: float(np.mean([f['bone_deg'][k] for f in h])) for k in h[0]['bone_deg']}
        eh = {s: float(max(f[f'elbow_high_{s}'] for f in h)) for s in ('Left', 'Right')}
        sy = {k: float(np.mean([f['symmetry_deg'][k] for f in h])) for k in h[0]['symmetry_deg']}
        sk = float(np.mean([f['lat_skew_cm'] for f in h]))
        return dict(joint_cm=j, bone_deg=b, elbow_high=eh, symmetry_deg=sy, lat_skew_cm=sk)

    def record_adjustment(self, frame_idx, rule, detail):
        rec = dict(frame=frame_idx, rule=rule, detail=detail)
        self.adjust_log.append(rec)
        return rec

    def save(self, path):
        json.dump(dict(thresholds=self.thr, history=self.history,
                       violations=self.violations, adjust_log=self.adjust_log),
                  open(path, 'w'), indent=1, default=float)


class QpParamSet:
    """Mutable QP/synthesis parameters the optimizer iterates (applied per pass)."""

    def __init__(self, lam_virtual=1.5, lam_physio=12.0, lam_medial=4.0,
                 lam_smooth=1.0, lam_posture=2.0, lam_rhythm=6.0,
                 q_yaw_rest_L=-0.374, q_yaw_rest_R=0.281,
                 p0_lat_L=0.04, p0_lat_R=0.04, p0_dy=0.05, p0_fwd=0.04):
        self.lam_virtual = lam_virtual
        self.lam_physio = lam_physio
        self.lam_medial = lam_medial
        self.lam_smooth = lam_smooth
        self.lam_posture = lam_posture
        self.lam_rhythm = lam_rhythm
        self.q_yaw_rest_L = q_yaw_rest_L
        self.q_yaw_rest_R = q_yaw_rest_R
        self.p0_lat_L = p0_lat_L
        self.p0_lat_R = p0_lat_R
        self.p0_dy = p0_dy
        self.p0_fwd = p0_fwd

    def tasks_kwargs(self):
        return dict(lam_virtual=self.lam_virtual, lam_physio=self.lam_physio,
                    lam_medial=self.lam_medial, lam_smooth=self.lam_smooth,
                    lam_posture=self.lam_posture, lam_rhythm=self.lam_rhythm,
                    q_yaw_rest_L=self.q_yaw_rest_L, q_yaw_rest_R=self.q_yaw_rest_R)

    def apply_synthesis(self):
        import general_motion_retargeting.xrobot_utils as xu
        xu.XU_WRIST_P0_LAT = self.p0_fwd
        xu.XU_ELBOW_P0_LAT_L = self.p0_lat_L
        xu.XU_ELBOW_P0_LAT_R = self.p0_lat_R
        xu.XU_ELBOW_P0_FWD = self.p0_fwd
        xu.XU_ELBOW_P0_DY = self.p0_dy

    def as_dict(self):
        return self.__dict__.copy()


def optimize_step(params, monitor, frame_idx, rule_priority=('elbow_high', 'lat_skew',
                                                              'joint', 'bone', 'symmetry')):
    """One rule-based adjustment step from the windowed stats. Returns a message or None."""
    st = monitor.window_stats()
    if not st:
        return None
    from collections import Counter
    c = Counter(x[1] for x in monitor.violations)
    n_eh = sum(c[k] for k in c if k.startswith('elbow_high'))
    n_joint = sum(c[k] for k in c if k.startswith('joint'))
    n_bone = sum(c[k] for k in c if k.startswith('bone'))
    n_sym = sum(c[k] for k in c if k.startswith('symmetry'))
    max_eh = max(st['elbow_high'].values())
    skew = st['lat_skew_cm']
    mean_joint = float(np.mean(list(st['joint_cm'].values())))
    mean_sym = float(np.mean(list(st['symmetry_deg'].values()))) if st['symmetry_deg'] else 0.0

    if n_eh > 0:
        import general_motion_retargeting.xrobot_utils as xu
        params.p0_dy = min(0.10, params.p0_dy + 0.01)
        params.lam_virtual = min(6.0, params.lam_virtual + 0.5)
        xu.XU_ENV_W = min(60.0, xu.XU_ENV_W + 5.0)     # tighten the phi-grid height envelope
        return (f'elbow_high(n={n_eh}, max {max_eh:.2f}m) -> XU_ELBOW_P0_DY={params.p0_dy:.2f}, '
                f'lam_virtual={params.lam_virtual:.1f}, XU_ENV_W={xu.XU_ENV_W:.0f}')
    if abs(skew) > monitor.thr['lat_skew_cm']:
        if skew < 0:                       # left-dominant (L elbow too far out)
            params.p0_lat_L = max(0.0, params.p0_lat_L - 0.005)
            msg = f'lat_skew({skew:+.1f}cm, left-dominant) -> p0_lat_L={params.p0_lat_L:.3f}'
        else:
            params.p0_lat_R = max(0.0, params.p0_lat_R - 0.005)
            msg = f'lat_skew({skew:+.1f}cm, right-dominant) -> p0_lat_R={params.p0_lat_R:.3f}'
        return msg
    if n_joint > 0:
        params.lam_virtual = min(6.0, params.lam_virtual + 0.5)
        return f'joint(n={n_joint}, mean {mean_joint:.1f}cm) -> lam_virtual={params.lam_virtual:.1f}'
    if n_bone > 0:
        params.lam_virtual = min(6.0, params.lam_virtual + 0.5)
        return f'bone(n={n_bone}) -> lam_virtual={params.lam_virtual:.1f}'
    if n_sym > 0:
        params.lam_rhythm = min(12.0, params.lam_rhythm + 1.0)
        return f'symmetry(n={n_sym}, mean {mean_sym:.1f}deg) -> lam_rhythm={params.lam_rhythm:.1f}'
    return None
