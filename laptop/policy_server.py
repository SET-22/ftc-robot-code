"""
Laptop side of AI mode: receives observations from the Control Hub over UDP,
runs the trained policy, and sends back an action.

    python policy_server.py --model policy.pt
    python policy_server.py --zero          # no model: always send 0,0,0,no-fire (link test)

Connect the laptop to the robot's WiFi first. Then on the Driver Station start
"Decision model teleop" and press Y on gamepad1 to give the laptop control.

Packets (plain text):
    laptop -> Hub   "H"                               hello (until observations arrive)
    Hub -> laptop   "O,seq,t_ms,have_target,tx,ty,area,conf,
                     shoot_fwd_err,shoot_left_err,shoot_head_err,vx,vy,balls,state"
    laptop -> Hub   "A,seq,fwd,strafe,turn,fire"      seq = the observation answered

The Hub still enforces every safety rule (speed cap, fire only when ready + stopped,
200 ms timeout, B/stick cancel). This script never sends motor powers.
"""

import argparse
import collections
import csv
import os
import socket
import time

OBS_FIELDS = ["seq", "t_ms", "have_target", "tx", "ty", "area", "conf",
              "shoot_fwd_err", "shoot_left_err", "shoot_head_err", "vx", "vy", "balls", "state"]
HELLO_PERIOD_S = 0.5
HISTORY_RESET_MS = 300   # gap in observations that starts a fresh history


def parse_obs(text: str):
    parts = text.split(",")
    if parts[0] != "O" or len(parts) != len(OBS_FIELDS) + 1:
        return None
    try:
        vals = [float(v) for v in parts[1:]]
    except ValueError:
        return None
    obs = dict(zip(OBS_FIELDS, vals))
    obs["seq"] = int(obs["seq"])
    obs["state"] = int(obs["state"])
    return obs


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--model", default="policy.pt")
    ap.add_argument("--zero", action="store_true", help="send zero actions (no model needed)")
    ap.add_argument("--hub-ip", default="192.168.43.1")
    ap.add_argument("--hub-port", type=int, default=6100)
    ap.add_argument("--port", type=int, default=6101, help="local UDP port")
    ap.add_argument("--fire-threshold", type=float, default=0.5)
    ap.add_argument("--log-dir", default="server_logs")
    args = ap.parse_args()

    runner = None
    history = 5
    if not args.zero:
        from train_policy import PolicyRunner   # imports torch only when needed
        runner = PolicyRunner(args.model, args.fire_threshold)
        history = runner.history

    os.makedirs(args.log_dir, exist_ok=True)
    log_path = os.path.join(args.log_dir, time.strftime("server_%Y%m%d_%H%M%S.csv"))
    log_file = open(log_path, "w", newline="", buffering=1)  # line-buffered: survives a kill
    log = csv.writer(log_file)
    log.writerow(["recv_time"] + OBS_FIELDS + ["fwd", "strafe", "turn", "fire", "fire_prob", "infer_ms"])

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("", args.port))
    sock.settimeout(0.1)
    hub = (args.hub_ip, args.hub_port)

    frames = collections.deque(maxlen=history)
    last_seq = -1
    last_t_ms = None
    last_obs_time = 0.0
    last_hello = 0.0
    last_print = time.time()
    count = 0
    print(f"policy server on port {args.port}, hub {hub}, logging to {log_path}")

    try:
        while True:
            now = time.time()
            # Say hello until the Hub is sending to us
            if now - last_obs_time > 1.0 and now - last_hello > HELLO_PERIOD_S:
                sock.sendto(b"H", hub)
                last_hello = now

            try:
                data, addr = sock.recvfrom(512)
            except socket.timeout:
                continue

            obs = parse_obs(data.decode("ascii", "ignore").strip())
            if obs is None:
                continue
            if obs["seq"] <= last_seq:
                # Out of order, or the OpMode restarted (seq starts again at 1)
                if obs["seq"] < last_seq - 100:
                    last_seq = -1
                    frames.clear()
                else:
                    continue
            if last_t_ms is not None and obs["t_ms"] - last_t_ms > HISTORY_RESET_MS:
                frames.clear()
            last_seq, last_t_ms, last_obs_time = obs["seq"], obs["t_ms"], now
            frames.append(obs)

            t0 = time.perf_counter()
            if runner is None:
                fwd = strafe = turn = 0.0
                fire, prob = False, 0.0
            else:
                fwd, strafe, turn, fire, prob = runner.act(frames)
            infer_ms = (time.perf_counter() - t0) * 1000

            reply = f"A,{obs['seq']},{fwd:.4f},{strafe:.4f},{turn:.4f},{1 if fire else 0}"
            sock.sendto(reply.encode("ascii"), addr)

            log.writerow([f"{now:.3f}"] + [obs[k] for k in OBS_FIELDS]
                         + [f"{fwd:.4f}", f"{strafe:.4f}", f"{turn:.4f}", int(fire), f"{prob:.3f}",
                            f"{infer_ms:.2f}"])
            count += 1
            if now - last_print > 2.0:
                log_file.flush()
                print(f"{count / (now - last_print):5.1f} obs/s  state {obs['state']}  balls {int(obs['balls'])}"
                      f"  -> fwd {fwd:+.2f} strafe {strafe:+.2f} turn {turn:+.2f} fire {int(fire)}"
                      f"  ({infer_ms:.1f} ms)")
                count = 0
                last_print = now
    except KeyboardInterrupt:
        pass
    finally:
        log_file.close()
        sock.close()


if __name__ == "__main__":
    main()
