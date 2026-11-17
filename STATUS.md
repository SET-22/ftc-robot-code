# Project status

Last updated 2026-11-16. Both old Claude branches (`claude/sharp-bohr-k64sgd` and `claude/youthful-bell-oakzma`) are now merged into `main`.

Legend: ✅ finished · 🟡 written but not tested on real hardware · ❌ not started

## FTC robot (TeamCode + laptop)

| Item | Status | Notes |
|---|---|---|
| `MyTeleOp` (driver TeleOp) | ✅ | Tuned and working. Comments fixed to say 20% transfer idle. |
| Gamepad1 controls for shooting spot, ball count, odometry reset | ✅ code / 🟡 robot | Left bumper saves spot, D-pad L/R adjusts count, right bumper zeroes it, Back resets odometry. Gamepad2 buttons still work too. |
| `nurse` gate positions | ✅ | Now match MyTeleOp (closed 0.7, open 0.0). |
| Cleanup: removed copied FTC Dashboard source, saved GitHub page, node_modules, yarn.lock | ✅ | Dashboard is now a Gradle dependency (`0.4.16` from maven.brott.dev). |
| Robot map moved to `docs/junior-bot-map.txt` | ✅ | Motor directions corrected to match MyTeleOp. |
| `DecisionModelTeleOp` auto cycle (search → collect → drive to spot → fire) | 🟡 | Compiles against FTC SDK 12.0.0. Never run on the robot. |
| Per-loop CSV training log | 🟡 | Works off-robot; needs a real driving session to produce data. |
| Laptop link (`PolicyLink` ↔ `laptop/policy_server.py`) | 🟡 | Tested over localhost only. Port 6100 over the Hub WiFi is unchecked. |
| `laptop/train_policy.py` (behavior cloning) | 🟡 | Trains on synthetic logs. No real data yet. |
| Limelight 3A pollen detection | ❌ | Camera not mounted yet. Config name `limelight` needs checking. |

### Still to check on the robot
- Signs: `HEADING_TURN_SIGN`, `STRAFE_SIGN`, `VISION_TURN_SIGN`
- Drive gains
- Ball-count thresholds: `BALL_CURRENT_AMPS`, `NEAR_TY_DEG`, `NEAR_AREA`
- Gradle sync with the new Dashboard dependency (the handoff doc mentions Dashboard 0.6.0, but the build file uses 0.4.16)
- First AI test: `python policy_server.py --zero` with the robot on blocks

See `CLAUDE_CODE_HANDOFF.md` for the full plan and run steps.

## Animatronic face (`animatronic-face/`)

| Item | Status | Notes |
|---|---|---|
| v0.1 CAD model (CadQuery, 15 servos) | ✅ | Eyes, eyelids, brows, jaw, mouth corners. Interference-checked across 7 poses. |
| STEP/STL exports, preview renders, BOM, research notes | ✅ | Assembly STEP imports into Onshape. |
| Print and test one eye module | ❌ | |
| Pi controller (expressions, blinking, saccades, jaw sync, face tracking) | ❌ | |
| AI hookup (speech-to-text → LLM → text-to-speech + emotion) | ❌ | |
| Phase 2: silicone skin, skull, neck, body | ❌ | |
