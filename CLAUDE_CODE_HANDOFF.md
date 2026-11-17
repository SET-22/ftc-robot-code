# Handoff: Pollen auto-cycle + learned drive policy (FTC Junior Bot)

## 0. Status (2026-11-02)

The two missing files were rebuilt from this spec (the originals were not available):

| File | What it is |
|---|---|
| `TeamCode/.../opmodes/DecisionModelTeleOp.java` | Tasks A, B, C: auto cycle, training log, laptop link, shadow + DAgger logging |
| `laptop/train_policy.py` | Behavior-cloning trainer + `PolicyRunner` |
| `laptop/policy_server.py` | Task D: UDP policy server (`--zero` mode for link tests without a model) |

Decisions made with the student:
- **Motor directions: `MyTeleOp` is correct** (fl/bl FORWARD, fr/br REVERSE). The Map's directions are wrong; `Constants` already matches. Open issue 1 is closed.
- **Transfer idle = 20%** (`IDLE_POWER = 0.2`, matches the latest `MyTeleOp` and the Map). Open issue 2 is closed.

Changes from the plan:
- Odometry reads the Pinpoint localizer directly (`new PinpointLocalizer(hardwareMap, Constants.localizerConfig)`) instead of `Constants.create(...)`. No follower is created, so the stutter in open issue 4 cannot happen.
- Velocity (`vx`, `vy`, `omega`) is computed from pose changes and rotated into the robot frame, not read from Pedro, so the frame is known for sure.
- The CSV logger writes every loop, but `train_policy.py` keeps only rows where `obs_seq` changed (~30 Hz). This makes the 5-frame history span the same time in training as it does live.

Verified off-robot:
- `DecisionModelTeleOp.java` compiles against the real FTC SDK 12.0.0 `RobotCore` + `Hardware` jars. The Limelight calls (`getConfidence`, `getClassName`, `getStaleness`, ...) exist in 12.0.0. The Pedro calls (`localizer.update()`, `pose()`, `setPose(Pose.zero())`, `Pose.x()/y()/heading()`) were checked only against stubs, but they are the same calls the repo's tuners use.
- The Java `PolicyLink` was run against `policy_server.py` over localhost: both directions work, and the action goes stale (null) within 200 ms after the server stops.
- `train_policy.py` trains on synthetic logs with the exact TeleOp CSV header, `--fast-only` works, and `PolicyRunner` drives to the spot and fires only there.

NOT verified (needs the robot): everything in task A's checklist, the signs (`HEADING_TURN_SIGN`, `STRAFE_SIGN`, `VISION_TURN_SIGN`), the gains, ball-count thresholds (`BALL_CURRENT_AMPS`, `NEAR_TY_DEG`, `NEAR_AREA`), the Limelight config name (`LIMELIGHT_NAME = "limelight"`, check it), and whether port 6100 is reachable over the Hub's WiFi.

### How to run
```
# Robot: "Decision model teleop". gamepad1 left bumper at the shooting spot, then:
#   gamepad1 X = drive there + fire once, A = full cycle (needs Limelight), Y = AI mode, B/stick = cancel
adb pull /sdcard/FIRST/decision_logs laptop/logs
cd laptop
pip install -r requirements.txt
python train_policy.py --data logs/ --out policy.pt
python policy_server.py --zero          # first: link test on blocks (robot should sit still)
python policy_server.py --model policy.pt
```

---

Paste or save this at the repo root so Claude Code has the full context. Nothing below has been compiled or run on the robot yet.

## 1. Goal

A portfolio project (not for competition). The robot should:
1. Find pollen (game balls) with a Limelight 3A running a pretrained neural detector.
2. Drive over pollen with the intake until it holds 4 balls.
3. Drive to a fixed shooting spot using Pinpoint odometry (through Pedro 3).
4. Shoot the balls, then repeat.

Final stage: a decision model trained on the laptop takes **full control of driving and the fire decision**, over WiFi. The Control Hub keeps the shooter, gate, intake, and safety. The robot does not need RPM or any shooter details in the model.

## 2. Files

| File | Where | Status |
|---|---|---|
| `MyTeleOp.java` | `teamcode/opmodes` | Student's existing, tuned TeleOp. **Do not modify.** |
| `DecisionModelTeleOp.java` | `teamcode/opmodes` | New. `MyTeleOp` plus an auto-cycle state machine, logging, optional Limelight. Compiles except unverified Limelight calls. |
| `Constants.java`, `Tuning.java` | `teamcode/pedro` | Existing Pedro 3 config. Only `Constants.create(hardwareMap)` is used. |
| `train_policy.py` | laptop | New. Behavior-cloning trainer. Not run yet (no data). |

TeleOp name on the Driver Station: **"Decision model teleop"**.

**Version warning:** the copy of `DecisionModelTeleOp.java` in the student's project may still call `follower.getPose().getX()/getY()/getHeading()`, which does not exist in Pedro 3. The fixed `readPose()` is:
`Pose p = follower.pose(); return new double[]{p.x(), p.y(), p.heading()};` with `import com.pedropathing.math.Pose;`. Also wrap `String.format` calls in `Locale.US` so CSV decimals stay dots.

## 3. Stack facts (verified from the build file / Pedro docs)

- FTC SDK 12.0.0, Pedro `com.pedropathing:revhub:3.0.2`, tuning 1.0.1, Ivy 1.1.1, FTC Dashboard 0.6.0.
- **Pedro 3 API** (not 2.x): `follower.pose()` returns `com.pedropathing.math.Pose` with `x()`, `y()`, `heading()` (inches, radians). `follower.update()` each loop. `Constants.create(hardwareMap)` builds the follower.
- Limelight: `Limelight3A`, `LLResult`, `LLResultTypes.DetectorResult` (`getTargetXDegrees/YDegrees/Area`, `getConfidence`, `getClassName`). The confidence/class calls are unverified against SDK 12.0.0.
- Limelight is **optional** in code (`hardwareMap.tryGet`); it is not mounted yet.

## 4. Hardware (Junior Bot map)

- Control Hub motors: `bl`=0, `fr`=1, `br`=2, `fl`=3. Expansion Hub: `intake`=0, `transfer`=1, `shooter`=2, `kickwheel`=3. Servo: `gate`=0. Odometry: goBILDA Pinpoint, config name `pinpoint`.
- Behavior (unchanged from `MyTeleOp`): shooter always on at `targetRPM`=4100 (bang-bang + PIDF, D-pad up/down ±50); intake always 100%; transfer idle 15% (100% while feeding); kickwheel constant 50%; gate closed unless feeding (right trigger).
- **No ball-detecting sensor exists.** Counting uses intake-current spikes, gated by vision (only count when the camera saw pollen near the intake recently). A beam break named `intakeBeam` is supported if added.

## 5. Open issues to resolve or ask the student about

1. **Motor direction conflict.** The Map says `fl`/`bl` REVERSE and `fr`/`br` FORWARD. `MyTeleOp` and Pedro `Constants` have the opposite. `DecisionModelTeleOp` currently follows the **Map**. Test on blocks; flip all four if wrong. If the Map is right, also flip the four lines in `Constants.drivetrainConfig`.
2. Transfer idle: 15% in code, 20% in the Map. Code left as-is.
3. The comment in `Constants` says the Foresight values are examples, but they look tuned. Irrelevant to the TeleOp (pose only) but matters for path following.
4. **Possible stutter:** the TeleOp calls `follower.update()` but never `follower.manual(...)`. In Pedro 3 an idle follower may write zero power to the drivetrain each loop. If manual driving is jittery, read the Pinpoint localizer directly instead of using the follower.
5. Pose frame depends on where the robot is at INIT. The shooting spot is saved live (gamepad2 D-pad up) or typed into `SHOOT_*_DEFAULT`. The robot must start on the same marked spot every run.

## 6. Tasks, in order

### A. Make it compile and run (no Limelight)
- Fix any compile errors (most likely the Limelight detector methods).
- Verify: manual driving matches `MyTeleOp`; pose x increases pushing forward and heading increases CCW; gamepad2 D-pad up saves the shooting spot; **X** drives to it, waits for `ready` + stopped, fires.
- Tune `HEADING_TURN_SIGN` and the strafe sign in `GO_TO_SHOOT` if the motion is wrong.

### B. Extend the CSV logger for training
The current log lacks the columns `train_policy.py` needs. Add (one row per loop, ~30 Hz):

`have_target, tx, ty, area, conf` (best pollen detection, zeros if none),
`shoot_fwd_err, shoot_left_err, shoot_head_err` (error to the shooting spot in the **robot frame**: these are `fwdErr`, `leftErr`, `eh` already computed in `GO_TO_SHOOT`; log zeros or NaN-safe values if no spot is saved),
`vx, vy` (robot-frame velocity from odometry),
`balls`,
`act_fwd, act_strafe, act_turn, act_fire` (`autoFwd/autoStrafe/autoTurn` and `autoFeed` as the **teacher's** command, whether or not it is applied).
Keep the existing columns too. Also write an `episode` marker when a cycle starts/ends if cheap.

### C. Hub-side laptop link ("AI mode")
- A background thread on the Hub; UDP, Hub IP `192.168.43.1`; send observations ~30 Hz, receive actions.
- Observation packet: `seq, t_ms, have_target, tx, ty, area, conf, shoot_fwd_err, shoot_left_err, shoot_head_err, vx, vy, balls, state`. Action packet: `seq, fwd, strafe, turn, fire`.
- **The laptop never sends motor powers.** Hub maps `fwd/strafe/turn` through the existing mecanum mix (keep `mult`, clamp to [-1, 1], add a rate limit and a speed cap).
- `fire=true` only *requests* a shot: the Hub still waits for `ready` and a stopped robot, then runs the existing feed path (`autoFeed`). Never fire when `ready` is false.
- Safety: if no fresh packet in ~200 ms, fall back to the rule-based state machine (or stop). Ignore stale or out-of-order `seq`. **B or any stick cancels instantly.**
- A toggle button to enter/leave AI mode; the existing controls must keep working.
- Shadow mode: the rules stay in control while the Hub logs what the model *would* have commanded next to the teacher command, plus an agreement rate.
- DAgger logging: when the model is in control, still compute the teacher's command each loop and log it as `act_*` so the next retrain uses it.

### D. Laptop policy server (Python)
- UDP server that receives observations, keeps the last 5 frames, runs `PolicyRunner.act(...)` from `train_policy.py`, and replies with the action.
- Log every received observation and the model's output to CSV for later analysis.
- Dependencies: `torch`, `pandas`, `numpy`.

### E. Optional improvements
- Fire one ball at a time when `ready` is true (currently feeds for a fixed `FIRE_MS`).
- Voltage-compensate `kF`.
- Relocalization between cycles (AprilTag pipeline or a wall-reset button) to limit odometry drift.

## 7. Training plan (`train_policy.py`)

- Behavior cloning: MLP (2x128) on a 5-frame history of observations to `fwd, strafe, turn` (MSE on tanh outputs) and `fire` (weighted BCE).
- Hold out **whole log files** for validation, not random rows. Do not feed the previous action as an input (copycat problem).
- Expect a few hundred teacher cycles to start. Then run the model live with the rules as fallback, apply DAgger (relabel states the model visits with the teacher's command, retrain), and finally retrain on only the fast, all-balls-scored cycles to beat the teacher.
- **Real metric:** balls scored per minute (model vs rule-based vs the student driving), not offline loss.

## 8. Constraints

- Do not change `MyTeleOp.java`, the shooter PIDF/bang-bang code, or any tuned constant (`kP`, `kF`, RPM, gate positions).
- Keep the student's code style: section banner comments, constants grouped at the top, explanatory comments.
- Keep edits small and targeted; do not rewrite files wholesale.
- Do not invent hardware config names; ask or use the names in section 4.
- Everything must still work with the Limelight absent.
- Detector models come from community releases (Hive Vision v2, RepLiteDet) or a model trained from the robot's own camera. Check licenses and credit authors in the write-up.

## 9. Done means

1. `DecisionModelTeleOp` compiles and runs with no Limelight; X-cycle fires from the saved spot.
2. With the Limelight and a detector model: A runs the full cycle collecting 4 and shooting.
3. Logs contain every column in task B; `python train_policy.py --data logs/` runs on them.
4. AI mode drives the robot from the laptop with the 200 ms fallback and cancel verified on blocks first.
