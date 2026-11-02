"""
Behavior-cloning trainer for the Junior Bot decision model.

Learns to copy the rule-based "teacher" in DecisionModelTeleOp:
    observation history (5 frames)  ->  fwd, strafe, turn  (tanh, MSE)
                                        fire               (logit, weighted BCE)

Usage
    adb pull /sdcard/FIRST/decision_logs logs
    python train_policy.py --data logs/ --out policy.pt
    python train_policy.py --data logs/ --out policy_fast.pt --fast-only 50   # best half of cycles

Notes
    * Validation holds out WHOLE log files, never random rows (rows next to each other
      are nearly identical, so a random split would look far better than reality).
    * The previous action is NOT an input (the "copycat" problem: the model would just
      repeat its last command instead of looking at the world).
    * Only rows where the teacher was active are used. Rows recorded while the model
      was driving still carry the teacher's label (act_*), which is what DAgger needs.
    * The real metric is balls scored per minute on the robot, not this loss.

PolicyRunner (bottom of the file) is what policy_server.py uses at run time.
"""

import argparse
import glob
import math
import os
import random

import numpy as np
import pandas as pd
import torch
import torch.nn as nn

# ---------------------------------------------------------------------------
# Feature definition — must match the Hub's observation packet
# ---------------------------------------------------------------------------
NUMERIC_COLS = [
    "have_target", "tx", "ty", "area", "conf",
    "shoot_fwd_err", "shoot_left_err", "shoot_head_err",
    "vx", "vy", "balls",
]
STATE_COL = "state"
N_STATES = 6          # IDLE, SEARCH, COLLECT, GO_TO_SHOOT, SETTLE, FIRE (Java enum order)
ACTION_COLS = ["act_fwd", "act_strafe", "act_turn"]
FIRE_COL = "act_fire"
BALLS_FULL = 4
DEFAULT_HISTORY = 5


def frame_features(numeric: np.ndarray, state: np.ndarray, mean, std) -> np.ndarray:
    """(N, 11) numeric + (N,) state -> (N, 11 + N_STATES) normalized features."""
    num = (numeric - mean) / std
    onehot = np.zeros((len(state), N_STATES), dtype=np.float32)
    idx = np.clip(state.astype(int), 0, N_STATES - 1)
    onehot[np.arange(len(state)), idx] = 1.0
    return np.concatenate([num, onehot], axis=1).astype(np.float32)


def stack_history(frames: np.ndarray, history: int) -> np.ndarray:
    """(T, F) -> (T, history*F). Oldest first; the start is padded with the first frame."""
    T = len(frames)
    out = []
    for k in range(history - 1, -1, -1):
        idx = np.maximum(np.arange(T) - k, 0)
        out.append(frames[idx])
    return np.concatenate(out, axis=1)


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------
class Policy(nn.Module):
    def __init__(self, in_dim: int, hidden: int = 128):
        super().__init__()
        self.body = nn.Sequential(
            nn.Linear(in_dim, hidden), nn.ReLU(),
            nn.Linear(hidden, hidden), nn.ReLU(),
        )
        self.drive = nn.Linear(hidden, 3)   # fwd, strafe, turn (tanh)
        self.fire = nn.Linear(hidden, 1)    # fire logit

    def forward(self, x):
        h = self.body(x)
        return torch.tanh(self.drive(h)), self.fire(h).squeeze(-1)


# ---------------------------------------------------------------------------
# Data loading
# ---------------------------------------------------------------------------
def load_file(path: str) -> pd.DataFrame:
    df = pd.read_csv(path)
    missing = [c for c in NUMERIC_COLS + [STATE_COL, "episode", "obs_seq", "teacher_active"]
               + ACTION_COLS + [FIRE_COL] if c not in df.columns]
    if missing:
        raise ValueError(f"{path}: missing columns {missing}")
    df["event"] = df["event"].fillna("") if "event" in df.columns else ""
    df["file"] = os.path.basename(path)
    return df


def fast_episodes(df: pd.DataFrame, pct: float) -> set:
    """Episodes that ended in a shot with a full load, faster than the pct percentile."""
    stats = []
    for ep, g in df.groupby("episode"):
        shots = g.index[g["event"] == "shot"]
        if len(shots) == 0:
            continue
        before = g.loc[:shots[0]]
        if before["balls"].max() < BALLS_FULL:
            continue
        stats.append((ep, g.loc[shots[0], "t_ms"] - g["t_ms"].iloc[0]))
    if not stats:
        return set()
    cutoff = np.percentile([d for _, d in stats], pct)
    return {ep for ep, d in stats if d <= cutoff}


def build_arrays(dfs, history, mean, std, fast_pct=None):
    """Turns log files into (X, y_drive, y_fire). Windows never cross episodes or files."""
    xs, yd, yf = [], [], []
    for df in dfs:
        keep_eps = fast_episodes(df, fast_pct) if fast_pct is not None else None
        for ep, g in df.groupby("episode", sort=False):
            if keep_eps is not None and ep not in keep_eps:
                continue
            # One row per observation actually sent (~30 Hz), so the history spacing
            # matches what policy_server.py sees live.
            g = g[g["obs_seq"].diff().fillna(1) != 0]
            g = g[g["teacher_active"] == 1]
            if len(g) == 0:
                continue
            frames = frame_features(g[NUMERIC_COLS].to_numpy(np.float32),
                                    g[STATE_COL].to_numpy(), mean, std)
            xs.append(stack_history(frames, history))
            yd.append(np.clip(g[ACTION_COLS].to_numpy(np.float32), -1, 1))
            yf.append(g[FIRE_COL].to_numpy(np.float32))
    if not xs:
        return None
    return np.concatenate(xs), np.concatenate(yd), np.concatenate(yf)


# ---------------------------------------------------------------------------
# Training
# ---------------------------------------------------------------------------
def evaluate(model, X, yd, yf):
    model.eval()
    with torch.no_grad():
        drive, logit = model(torch.from_numpy(X))
        mse = ((drive - torch.from_numpy(yd)) ** 2).mean().item()
        pred = (torch.sigmoid(logit) > 0.5).numpy()
    truth = yf > 0.5
    tp = int((pred & truth).sum())
    prec = tp / max(1, int(pred.sum()))
    rec = tp / max(1, int(truth.sum()))
    return mse, prec, rec


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--data", default="logs/", help="folder with run_*.csv files")
    ap.add_argument("--out", default="policy.pt")
    ap.add_argument("--history", type=int, default=DEFAULT_HISTORY)
    ap.add_argument("--epochs", type=int, default=40)
    ap.add_argument("--batch", type=int, default=256)
    ap.add_argument("--lr", type=float, default=1e-3)
    ap.add_argument("--val-frac", type=float, default=0.2, help="fraction of FILES held out")
    ap.add_argument("--fast-only", type=float, default=None, metavar="PCT",
                    help="train only on full-load cycles faster than this percentile (e.g. 50)")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    random.seed(args.seed)
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)

    paths = sorted(glob.glob(os.path.join(args.data, "**", "*.csv"), recursive=True))
    if not paths:
        raise SystemExit(f"No CSV files in {args.data}")
    dfs = [load_file(p) for p in paths]

    # ---- Split by whole files ----
    order = list(range(len(dfs)))
    random.shuffle(order)
    n_val = int(round(len(dfs) * args.val_frac))
    if len(dfs) >= 2:
        n_val = max(1, n_val)
    else:
        n_val = 0
        print("WARNING: only one log file, so no validation set. Record more runs.")
    val_idx = set(order[:n_val])
    train_dfs = [d for i, d in enumerate(dfs) if i not in val_idx]
    val_dfs = [d for i, d in enumerate(dfs) if i in val_idx]
    print(f"{len(train_dfs)} training files, {len(val_dfs)} validation files")

    # ---- Normalization from training rows only ----
    active = pd.concat([d[d["teacher_active"] == 1] for d in train_dfs])
    if len(active) == 0:
        raise SystemExit("No rows with teacher_active == 1. Run the A / X / Y modes to record data.")
    mean = active[NUMERIC_COLS].to_numpy(np.float32).mean(axis=0)
    std = active[NUMERIC_COLS].to_numpy(np.float32).std(axis=0) + 1e-3

    train = build_arrays(train_dfs, args.history, mean, std, args.fast_only)
    if train is None:
        raise SystemExit("No training samples after filtering.")
    val = build_arrays(val_dfs, args.history, mean, std, args.fast_only) if val_dfs else None
    Xtr, ydtr, yftr = train
    print(f"{len(Xtr)} training samples ({int(yftr.sum())} fire), "
          f"{0 if val is None else len(val[0])} validation samples")

    # ---- Model + losses ----
    model = Policy(Xtr.shape[1])
    opt = torch.optim.Adam(model.parameters(), lr=args.lr, weight_decay=1e-5)
    n_pos = max(1.0, float(yftr.sum()))
    pos_weight = torch.tensor((len(yftr) - n_pos) / n_pos)   # fire is rare -> weight it up
    bce = nn.BCEWithLogitsLoss(pos_weight=pos_weight)

    Xt, ydt, yft = map(torch.from_numpy, (Xtr, ydtr, yftr))
    best = math.inf
    best_state = None
    for epoch in range(1, args.epochs + 1):
        model.train()
        perm = torch.randperm(len(Xt))
        total = 0.0
        for i in range(0, len(Xt), args.batch):
            b = perm[i:i + args.batch]
            drive, logit = model(Xt[b])
            loss = ((drive - ydt[b]) ** 2).mean() + bce(logit, yft[b])
            opt.zero_grad()
            loss.backward()
            opt.step()
            total += loss.item() * len(b)

        msg = f"epoch {epoch:3d}  train loss {total / len(Xt):.4f}"
        if val is not None and len(val[0]):
            mse, prec, rec = evaluate(model, *val)
            msg += f"  val drive MSE {mse:.4f}  fire precision {prec:.2f} recall {rec:.2f}"
            score = mse + (1 - rec) * 0.1
        else:
            score = total / len(Xt)
        if score < best:
            best = score
            best_state = {k: v.clone() for k, v in model.state_dict().items()}
            msg += "  *"
        print(msg)

    torch.save({
        "state_dict": best_state,
        "in_dim": Xtr.shape[1],
        "history": args.history,
        "mean": mean.tolist(),
        "std": std.tolist(),
        "numeric_cols": NUMERIC_COLS,
        "n_states": N_STATES,
    }, args.out)
    print(f"saved {args.out}")


# ---------------------------------------------------------------------------
# Run-time wrapper (used by policy_server.py)
# ---------------------------------------------------------------------------
class PolicyRunner:
    """Loads a checkpoint and turns an observation history into an action."""

    def __init__(self, path: str, fire_threshold: float = 0.5):
        ckpt = torch.load(path, map_location="cpu")
        if ckpt["numeric_cols"] != NUMERIC_COLS or ckpt["n_states"] != N_STATES:
            raise ValueError("checkpoint was trained with different features")
        self.history = ckpt["history"]
        self.mean = np.array(ckpt["mean"], dtype=np.float32)
        self.std = np.array(ckpt["std"], dtype=np.float32)
        self.model = Policy(ckpt["in_dim"])
        self.model.load_state_dict(ckpt["state_dict"])
        self.model.eval()
        self.fire_threshold = fire_threshold

    def act(self, frames):
        """frames: list of dicts (oldest first) with NUMERIC_COLS + 'state'.
        Returns (fwd, strafe, turn, fire, fire_prob)."""
        if not frames:
            return 0.0, 0.0, 0.0, False, 0.0
        frames = list(frames)[-self.history:]
        frames = [frames[0]] * (self.history - len(frames)) + frames   # pad like training
        numeric = np.array([[f[c] for c in NUMERIC_COLS] for f in frames], dtype=np.float32)
        state = np.array([f["state"] for f in frames])
        x = frame_features(numeric, state, self.mean, self.std).reshape(1, -1)
        with torch.no_grad():
            drive, logit = self.model(torch.from_numpy(x))
        fwd, strafe, turn = (float(v) for v in drive[0])
        prob = float(torch.sigmoid(logit[0]))
        return fwd, strafe, turn, prob > self.fire_threshold, prob


if __name__ == "__main__":
    main()
