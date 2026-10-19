"""
Inverse model for the 3-motor soft robot:  desired pose  ->  motor positions.

    pose   = [z (mm, platform height rel. to rest), roll (deg), pitch (deg)]
    rate   = d(pose)/dt of the *desired* trajectory (known from the MPC reference)
    output = [m1, m2, m3]  motor position (deg, absolute)

Pipeline
  1. Fuse the two logs: the mocap run gives platform height, the motor run gives
     motor positions + IMU roll/pitch. Mocap is aligned to the motor log with ONE
     constant time offset and ONE yaw angle between marker frame and IMU frame
     (both fitted by least squares against the IMU roll/pitch).
  2. Inverse network with 3-fold symmetry: ONE small MLP shared by the three motors.
     Motor i sees  [z, a_i, |b_i|, flags]  where a_i / b_i are the tilt components
     along / across tendon i. This triples the effective data and lets every motor
     see all relative directions that any motor sees.
  3. Direction-of-motion flags (tanh of the desired-pose rate) let the net handle
     the tendon hysteresis without ever seeing motor measurements.
  4. Leave-one-segment-out CV, final fit, numpy predictor, CasADi export for MPC.

Run:  python softri_inverse_model.py  [--mocap Data_Col.csv --motors softri_ml_dataset_*.csv]
Needs: numpy pandas scipy scikit-learn matplotlib  (+ casadi for the export check)
"""
import argparse, json, warnings
import numpy as np, pandas as pd
from scipy.optimize import minimize
from scipy.signal import savgol_filter
from sklearn.neural_network import MLPRegressor
from sklearn.preprocessing import StandardScaler
warnings.filterwarnings("ignore")

DT = 1 / 30.0                       # motor-log sample time
# Tendon directions in the (roll, pitch) plane (read off the single-motor sweeps:
# m1 -> pitch-, m2 -> roll-/pitch+, m3 -> roll+/pitch+ ; 120 deg apart)
U = np.array([[0, -1.0], [-np.sqrt(3) / 2, 0.5], [np.sqrt(3) / 2, 0.5]])
V_Z, V_A = 15.0, 4.0                # scales of the motion flags (mm/s, deg/s)
MOTOR_SPAN = 605.0                  # largest relative motor travel seen in the data (deg)


# ----------------------------------------------------------------------------- data
def mocap_plate_frame(m, yaw):
    """Centroid + world-'up' expressed in the plate frame, from the 3 markers."""
    P = [m[[f"x{i}", f"y{i}", f"z{i}"]].values for i in (1, 2, 3)]
    c = np.mean(P, 0)
    ex = P[1] - P[0]; ex /= np.linalg.norm(ex, axis=1)[:, None]
    ez = np.cross(P[1] - P[0], P[2] - P[0]); ez /= np.linalg.norm(ez, axis=1)[:, None]
    ez *= np.sign(ez[:, 1:2])
    ey = np.cross(ez, ex)
    up = np.array([0, 1.0, 0])                      # Motive: Y is up
    ux, uy, uz = ex @ up, ey @ up, ez @ up
    cy, sy = np.cos(yaw), np.sin(yaw)
    return c, cy * ux - sy * uy, sy * ux + cy * uy, uz


def mocap_roll_pitch(ux, uy, uz):
    # axis naming/sign found by exhaustive search against the IMU (roll<->pitch swapped, both inverted)
    p = -np.degrees(np.arctan2(uy, uz))
    r = -np.degrees(np.arctan2(-ux, np.hypot(uy, uz)))
    return r, p


def load(mocap_csv, motors_csv):
    m = pd.read_csv(mocap_csv, skiprows=7, header=None).astype(float)
    m.columns = ["f", "t", "x1", "y1", "z1", "x2", "y2", "z2", "x3", "y3", "z3"]
    d = pd.read_csv(motors_csv)
    return m, d


def fit_alignment(m, d, x0=(np.radians(59.0), 7.86)):
    r0, p0 = d.roll_deg.iloc[:20].mean(), d.pitch_deg.iloc[:20].mean()

    def cost(x):
        yaw, off = x
        _, ux, uy, uz = mocap_plate_frame(m, yaw)
        r, p = mocap_roll_pitch(ux, uy, uz)
        r, p = r - r[:600].mean(), p - p[:600].mean()
        tm = d.time_s.values + off
        return np.mean((np.interp(tm, m.t, r) - (d.roll_deg - r0)) ** 2 +
                       (np.interp(tm, m.t, p) - (d.pitch_deg - p0)) ** 2)

    res = minimize(cost, x0, method="Nelder-Mead", options=dict(xatol=1e-4, fatol=1e-7))
    return res.x, np.sqrt(res.fun / 2)           # (yaw, offset), tilt RMSE per axis (deg)


def build_dataset(mocap_csv, motors_csv):
    m, d = load(mocap_csv, motors_csv)
    (yaw, off), rmse = fit_alignment(m, d)
    print(f"alignment: yaw={np.degrees(yaw):.1f} deg, time offset={off:.2f} s, "
          f"mocap-vs-IMU tilt RMSE={rmse:.2f} deg")
    c, *_ = mocap_plate_frame(m, yaw)
    z = (c[:, 1] - c[:600, 1].mean()) * 1000.0            # mm, rest = 0, negative = lowered
    t = d.time_s.values
    rest = np.array([d[f"m{i}_position_deg"].iloc[:20].mean() for i in (1, 2, 3)])
    df = pd.DataFrame({"t": t})
    for i in (1, 2, 3):
        df[f"m{i}"] = d[f"m{i}_position_deg"].values - rest[i - 1]   # relative to rest
        df[f"cur{i}"] = d[f"m{i}_current_mA"].values
    df["roll"] = d.roll_deg.values - d.roll_deg.iloc[:20].mean()
    df["pitch"] = d.pitch_deg.values - d.pitch_deg.iloc[:20].mean()
    df["z"] = np.interp(t + off, m.t, z)
    # motion segments of the protocol: all3, m1, m2, m3, then three pairs
    df["seg"] = np.digitize(t, [12.5, 20.5, 28.5, 36.5, 44.5, 52.5])
    return df, rest


def pose_rate(pose, dt=DT, win=9):
    """Smoothed d(pose)/dt of a trajectory sampled at dt (use the reference trajectory in MPC)."""
    return np.c_[[savgol_filter(pose[:, k], win, 2, deriv=1, delta=dt) for k in range(3)]].T


# ----------------------------------------------------------------------------- model
def stack_features(pose, rate):
    """Rows for motor 1, then motor 2, then motor 3 (shared network)."""
    rows = []
    for i in range(3):
        a = pose[:, 1:3] @ U[i]                                   # tilt along tendon i
        b = pose[:, 1:3] @ np.array([-U[i, 1], U[i, 0]])          # tilt across tendon i
        va = rate[:, 1:3] @ U[i]
        rows.append(np.c_[pose[:, 0], a, np.abs(b), np.tanh(rate[:, 0] / V_Z), np.tanh(va / V_A)])
    return np.vstack(rows)


class InverseModel:
    def __init__(self, hidden=(24, 24), alpha=1e-3, seeds=3):
        self.hidden, self.alpha, self.seeds = hidden, alpha, seeds

    def fit(self, pose, rate, motors_rel):
        F = stack_features(pose, rate)
        y = np.concatenate([motors_rel[:, i] for i in range(3)])
        self.sx = StandardScaler().fit(F)
        self.ym, self.ys = y.mean(), y.std()
        self.nets = [MLPRegressor(hidden_layer_sizes=self.hidden, activation="tanh", alpha=self.alpha,
                                  max_iter=4000, tol=1e-8, random_state=k)
                     .fit(self.sx.transform(F), (y - self.ym) / self.ys) for k in range(self.seeds)]
        return self

    def predict_rel(self, pose, rate):
        F = self.sx.transform(stack_features(pose, rate))
        p = np.mean([n.predict(F) for n in self.nets], 0) * self.ys + self.ym
        n = len(pose)
        return np.clip(np.c_[p[:n], p[n:2 * n], p[2 * n:]], 0.0, MOTOR_SPAN)  # tendons only pull

    # ---- export -----------------------------------------------------------
    def save(self, path, rest):
        np.savez(path, rest=rest, mean=self.sx.mean_, scale=self.sx.scale_, ym=self.ym, ys=self.ys,
                 n_nets=len(self.nets), U=U, V_Z=V_Z, V_A=V_A, span=MOTOR_SPAN,
                 **{f"W{k}_{j}": w for k, n in enumerate(self.nets) for j, w in enumerate(n.coefs_)},
                 **{f"b{k}_{j}": b for k, n in enumerate(self.nets) for j, b in enumerate(n.intercepts_)})

    def to_casadi(self, rest, absolute=True):
        """CasADi Function  f(pose[3], rate[3]) -> motor[3]  (deg), usable inside an NLP/MPC."""
        import casadi as ca
        pose, rate = ca.SX.sym("pose", 3), ca.SX.sym("rate", 3)
        outs = []
        for i in range(3):
            a = pose[1] * U[i, 0] + pose[2] * U[i, 1]
            b = pose[1] * (-U[i, 1]) + pose[2] * U[i, 0]
            va = rate[1] * U[i, 0] + rate[2] * U[i, 1]
            f = ca.vertcat(pose[0], a, ca.sqrt(b * b + 1e-6), ca.tanh(rate[0] / V_Z), ca.tanh(va / V_A))
            f = (f - self.sx.mean_) / self.sx.scale_            # smooth |b| (sqrt) keeps NLP gradients finite
            acc = 0
            for n in self.nets:
                h = f
                for j, (W, bb) in enumerate(zip(n.coefs_, n.intercepts_)):
                    h = ca.mtimes(W.T, h) + bb
                    if j < len(n.coefs_) - 1:
                        h = ca.tanh(h)
                acc += h / len(self.nets)
            outs.append(acc * self.ys + self.ym)
        m = ca.vertcat(*outs)
        m = ca.fmin(ca.fmax(m, 0.0), MOTOR_SPAN) + (rest if absolute else 0)
        return ca.Function("softri_inverse", [pose, rate], [m], ["pose", "rate"], ["motors_deg"])


# ----------------------------------------------------------------------------- evaluation
def loso_cv(df, segs_to_test=range(1, 7)):
    pose = df[["z", "roll", "pitch"]].values
    rate = pose_rate(pose)
    Y = df[["m1", "m2", "m3"]].values
    out, pred = {}, np.full_like(Y, np.nan)
    for s in segs_to_test:
        tr, te = df.seg.values != s, df.seg.values == s
        P = InverseModel().fit(pose[tr], rate[tr], Y[tr]).predict_rel(pose[te], rate[te])
        pred[te] = P
        out[s] = np.sqrt(np.mean((P - Y[te]) ** 2))
    return out, pred


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mocap", default="/mnt/user-data/uploads/Data_Col.csv")
    ap.add_argument("--motors", default="/mnt/user-data/uploads/softri_ml_dataset_20261008_111120.csv")
    ap.add_argument("--out", default="/mnt/user-data/outputs")
    a = ap.parse_args()

    df, rest = build_dataset(a.mocap, a.motors)
    df.to_csv(f"{a.out}/softri_fused_dataset.csv", index=False)
    print("rest motor positions (deg):", rest.round(2))

    cv, pred = loso_cv(df)
    print("leave-one-segment-out motor RMSE (deg) per held-out segment:",
          {k: round(v, 1) for k, v in cv.items()}, "| mean %.1f" % np.mean(list(cv.values())))

    pose = df[["z", "roll", "pitch"]].values
    rate = pose_rate(pose)
    Y = df[["m1", "m2", "m3"]].values
    model = InverseModel(seeds=5).fit(pose, rate, Y)
    fit_rmse = np.sqrt(np.mean((model.predict_rel(pose, rate) - Y) ** 2))
    print("final model (all data) in-sample RMSE: %.1f deg" % fit_rmse)
    model.save(f"{a.out}/softri_inverse_weights.npz", rest)

    # CasADi export check
    try:
        import casadi as ca
        f = model.to_casadi(rest)
        k = 700
        ref = model.predict_rel(pose[k:k + 1], rate[k:k + 1])[0] + rest
        got = np.array(f(pose[k], rate[k])).ravel()
        print("CasADi vs numpy max abs diff (deg): %.2e" % np.abs(ref - got).max())
    except ImportError:
        print("casadi not installed - skipping export check")

    # plot
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    fig, ax = plt.subplots(3, 1, figsize=(13, 8), sharex=True)
    for i in range(3):
        ax[i].plot(df.t, Y[:, i] + rest[i], "k", lw=1.5, label="measured")
        ax[i].plot(df.t, pred[:, i] + rest[i], "r", lw=1, label="predicted (segment held out)")
        ax[i].set_ylabel(f"motor {i + 1} (deg)")
    for s in range(7):
        ax[0].axvline(df.t[df.seg == s].min(), color="gray", ls=":", lw=0.7)
    ax[0].legend(loc="upper right"); ax[-1].set_xlabel("time (s)")
    ax[0].set_title("Inverse model: every segment predicted by a model that never saw it (segment 0 = out of range, not tested)")
    plt.tight_layout(); plt.savefig(f"{a.out}/softri_inverse_loso.png", dpi=110)


if __name__ == "__main__":
    main()
