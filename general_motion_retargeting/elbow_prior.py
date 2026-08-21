"""L3 learned elbow-azimuth prior (L3): predict the operator's natural elbow azimuth.

Trained offline on recorded ground-truth bundles (real body + controllers + headset).
Feature: [arm direction (shoulder->wrist, body frame), wrist rotation vector (body frame)]
         -- the wrist rotation encodes shoulder internal/external rotation, which drives
         whether the elbow tucks back or sticks out (e.g. wrist-in + raise -> elbow out).
Label  : real elbow azimuth phi on the 2-bone IK solution circle, measured from the
         body-frame pole direction (same reference the synthesis uses). Near-vertical
         hanging arms are labelled with phi=0 (the natural tuck): the body tracker's
         elbow azimuth is unreliable there (it pushes the elbow ~90 deg forward).

phi is used as `phi=` in _two_bone_ik so the synthesized elbow follows the operator's
real behavior instead of a fixed analytic pole.
"""
import json
import pickle
import numpy as np
from scipy.spatial.transform import Rotation as R
from general_motion_retargeting.xrobot_utils import (
    StandingTemplate, _arm_max_reach, _qrotate, _qconj, _qmul, synthesize_upper_body,
)

BODY_JOINT_NAMES = [
    "Pelvis", "Left_Hip", "Right_Hip", "Spine1", "Left_Knee", "Right_Knee",
    "Spine2", "Left_Ankle", "Right_Ankle", "Spine3", "Left_Foot", "Right_Foot",
    "Neck", "Left_Collar", "Right_Collar", "Head", "Left_Shoulder", "Right_Shoulder",
    "Left_Elbow", "Right_Elbow", "Left_Wrist", "Right_Wrist", "Left_Hand", "Right_Hand",
]


class ElbowPrior:
    def __init__(self):
        self.pairs = {"Left": [], "Right": []}  # side -> list of (d_body_unit(3), phi)
        self.arr = {}

    # ---- shared geometry with synthesize_upper_body ----
    @staticmethod
    def body_pole(side, pelvis_quat):
        back = _qrotate(pelvis_quat, np.array([0.0, 0.0, -1.0]))
        lat = _qrotate(pelvis_quat, np.array([-1.0, 0.0, 0.0]) if side == "Left"
                       else np.array([1.0, 0.0, 0.0]))
        p = np.array([0.0, -1.0, 0.0]) + 0.35 * back + 0.2 * lat
        return p / float(np.linalg.norm(p))

    @staticmethod
    def circle_params(S, W, l1, l2):
        d_vec = W - S
        raw_d = float(np.linalg.norm(d_vec))
        if raw_d < 1e-6:
            return None
        u0 = d_vec / raw_d
        d = float(np.clip(raw_d, abs(l1 - l2) + 1e-4, _arm_max_reach(l1, l2)))
        a = float(np.clip((l1 * l1 - l2 * l2 + d * d) / (2.0 * d), 0.0, l1 - 1e-6))
        h = float(np.sqrt(max(l1 * l1 - a * a, 0.0)))
        return u0, a, h

    @staticmethod
    def _phi(side, pelvis_quat, S, W, E, l1, l2):
        cp = ElbowPrior.circle_params(S, W, l1, l2)
        if cp is None:
            return 0.0
        u0, a, h = cp
        C = S + a * u0
        pole = ElbowPrior.body_pole(side, pelvis_quat)
        g = pole - u0 * float(np.dot(pole, u0))
        ng = float(np.linalg.norm(g))
        if ng < 1e-6:
            return 0.0
        pdir = g / ng
        perp = np.cross(u0, pdir)
        ep = np.asarray(E, dtype=float) - C
        return float(np.arctan2(np.dot(ep, perp), np.dot(ep, pdir)))

    # ---- training ----
    def add_sample(self, side, synth_body, real_elbow_pos, template):
        """Add a training sample using the SYNTHESIZED geometry (shoulder/wrist/pelvis),
        so the feature and label geometry match synthesis exactly.
        Feature: [arm direction (body frame, 3D), wrist rotation vector (body frame, 3D)].
        Label  : real elbow azimuth; near-vertical hanging arms use phi=0 (natural tuck)
                 because the body tracker pushes those elbows forward (~90 deg bias)."""
        pq = np.asarray(synth_body["Pelvis"][1], dtype=float)
        S = np.asarray(synth_body[side + "_Shoulder"][0], dtype=float)
        W = np.asarray(synth_body[side + "_Wrist"][0], dtype=float)
        wq = np.asarray(synth_body[side + "_Wrist"][1], dtype=float)
        E = np.asarray(real_elbow_pos, dtype=float)
        l1, l2 = template.arm_lengths()[side]
        d_vec = W - S
        db = _qrotate(_qconj(pq), d_vec)
        n = float(np.linalg.norm(db))
        if n < 1e-6:
            return
        arm_dir = db / n
        phi = ElbowPrior._phi(side, pq, S, W, E, l1, l2)
        if float(arm_dir[1]) < -0.8:
            phi = 0.0
        rv = R.from_quat(_qmul(_qconj(pq), wq), scalar_first=True).as_rotvec()
        self.pairs[side].append((np.concatenate([arm_dir, 0.3 * rv]), phi))

    def fit(self):
        for side, v in self.pairs.items():
            if v:
                self.arr[side] = (np.array([x[0] for x in v]), np.array([x[1] for x in v]))

    # ---- inference (used by synthesize_upper_body) ----
    def predict(self, side, pelvis_quat, S, W, wrist_quat):
        arr = self.arr.get(side)
        if arr is None or len(arr[1]) == 0:
            return None
        pq = np.asarray(pelvis_quat, dtype=float)
        d_vec = np.asarray(W, dtype=float) - np.asarray(S, dtype=float)
        db = _qrotate(_qconj(pq), d_vec)
        n = float(np.linalg.norm(db))
        if n < 1e-6:
            return None
        arm_dir = db / n
        rv = R.from_quat(_qmul(_qconj(pq), np.asarray(wrist_quat, dtype=float)),
                         scalar_first=True).as_rotvec()
        q = np.concatenate([arm_dir, 0.3 * rv])
        D, P = arr
        k = min(8, len(P))
        dist = np.linalg.norm(D - q, axis=1)
        idx = np.argsort(dist)[:k]
        w = 1.0 / (dist[idx] + 1e-3)
        w = w / w.sum()
        phis = P[idx]
        return float(np.arctan2(np.sum(w * np.sin(phis)), np.sum(w * np.cos(phis))))

    def save(self, path):
        with open(path, "wb") as f:
            pickle.dump({"pairs": self.pairs}, f)

    @staticmethod
    def load(path):
        with open(path, "rb") as f:
            data = pickle.load(f)
        p = ElbowPrior()
        p.pairs = data["pairs"]
        p.fit()
        return p


def train(frames_json, template_path, out_pkl):
    T = StandingTemplate.load(template_path)
    frames = json.load(open(frames_json))
    prior = ElbowPrior()

    def nz(p7):
        return p7 is not None and float(np.linalg.norm(np.asarray(p7[:3], dtype=float))) > 1e-3

    B = [f for f in frames[30:] if f["body"] is not None and nz(f["headset"])
         and nz(f["ctrl_left"]) and nz(f["ctrl_right"])]
    n_frames = 0
    for f in B:
        body = f["body"]
        # synthesize with the SAME pipeline used at runtime (features/geometry consistent)
        synth, _ = synthesize_upper_body(f["headset"], f["ctrl_left"], f["ctrl_right"], T, None)
        for side in ("Left", "Right"):
            real_elbow = np.asarray(body[side + "_Elbow"][0], dtype=float)
            prior.add_sample(side, synth, real_elbow, T)
        n_frames += 1
    prior.fit()
    prior.save(out_pkl)
    print(f"[elbow_prior] trained on {n_frames} frames")
    for side in ("Left", "Right"):
        print(f"[elbow_prior] {side}: {len(prior.pairs[side])} samples")
    return prior


if __name__ == "__main__":
    import sys
    data = sys.argv[1] if len(sys.argv) > 1 else "/tmp/kilo/rec_elbow_prior.npz.json"
    out = sys.argv[2] if len(sys.argv) > 2 else "/tmp/kilo/elbow_prior.pkl"
    train(data, "/home/user/.xrobotkit/standing_template.json", out)
