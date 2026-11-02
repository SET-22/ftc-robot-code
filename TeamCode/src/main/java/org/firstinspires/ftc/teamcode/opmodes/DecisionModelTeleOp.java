// package here — must match your folder
package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetSocketAddress;
import java.net.SocketTimeoutException;
import java.nio.charset.StandardCharsets;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.List;
import java.util.Locale;

/*
 * ============================== OVERVIEW ==============================
 *
 * MyTeleOp + an automatic "pollen cycle" + training logs + a laptop link.
 * Everything MyTeleOp does still works exactly the same way.
 *
 * THE CYCLE
 *   SEARCH ....... turn slowly until the Limelight sees pollen
 *   COLLECT ...... aim at the pollen and drive over it (intake always on)
 *                  until BALLS_FULL balls are counted
 *   GO_TO_SHOOT .. drive to the saved shooting spot (Pinpoint odometry)
 *   SETTLE ....... hold still until the robot is stopped and the shooter is ready
 *   FIRE ......... open the gate and feed for FIRE_MS, then start again
 *
 * WHO DRIVES (mode)
 *   MANUAL ....... the driver (exactly like MyTeleOp)
 *   RULES ........ the state machine above (the "teacher")
 *   AI ........... the model on the laptop, over WiFi. The teacher still runs every
 *                  loop so its command can be logged (DAgger) and used as the
 *                  fallback if the laptop goes quiet for LINK_TIMEOUT_MS.
 *
 * CONTROLS (gamepad1) — MyTeleOp controls are unchanged
 *   Left stick ........ drive (forward/back + strafe)
 *   Right stick X ..... turn
 *   Right trigger ..... hold to FEED (gate open, transfer 100%)
 *   D-pad up/down ..... shooter target RPM +50 / -50 per press
 *   A ................. start the full auto cycle (needs the Limelight)
 *   X ................. drive to the shooting spot and fire once
 *   Y ................. toggle AI mode (laptop drives)
 *   B ................. CANCEL back to manual (moving any stick also cancels)
 *   Left bumper ....... save the current position as the shooting spot
 *   D-pad right/left .. ball count +1 / -1 (fix a miscount)
 *   Right bumper ...... ball count = 0
 *   Back .............. reset odometry to (0, 0, 0) — robot must be on the start mark
 *
 * CONTROLS (gamepad2) — optional backup, same jobs as above
 *   D-pad up = save spot, D-pad right/left = balls +1/-1, A = balls 0, Back = reset odometry
 *
 * SAFETY
 *   - The laptop only sends fwd/strafe/turn/fire. It never sets motor powers.
 *   - fire only REQUESTS a shot: we still wait for ready + stopped.
 *   - No fresh laptop packet for LINK_TIMEOUT_MS -> rules take over (or stop).
 *   - B or any stick cancels instantly.
 *
 * FILE LAYOUT
 *   1. Tunable constants
 *   2. Hardware objects
 *   3. Controller state
 *   4. runOpMode: init, then the main loop
 *        - Buttons (mode changes, cancel)
 *        - Odometry + velocity
 *        - Vision (Limelight)
 *        - Ball counting
 *        - Teacher (rule-based state machine)
 *        - Laptop link (send observation, read action)
 *        - Choose + apply drive command
 *        - Shooter (bang-bang + PIDF)   <- copied unchanged from MyTeleOp
 *        - Ready check / gate / intake / transfer
 *        - Logging + telemetry
 *   5. Helpers (math, CSV logger, laptop link thread)
 */
@TeleOp(name = "Decision model teleop", group = "Practice")
public class DecisionModelTeleOp extends LinearOpMode {

    // =====================================================================
    // 1. TUNABLE CONSTANTS
    // =====================================================================

    // ---------- Drive ----------
    private double mult = 0.9; // overall drive speed multiplier (0 to 1)

    // ---------- Gate ----------
    private static final double GATE_CLOSED = 0.7; // tune
    private static final double GATE_OPEN = 0.0;   // tune
    private static final double FEED_DELAY_MS = 250; // tune

    // ---------- Intake + transfer ----------
    private static final double INTAKE_POWER = 1.0; // intake, always on (tune)
    private static final double IDLE_POWER = 0.2;   // transfer idle (tune)
    private static final double FEED_POWER = 1.0;   // transfer while feeding (tune)

    // ---------- Controls ----------
    private static final double TRIGGER_THRESHOLD = 0.5;
    private static final double STICK_CANCEL = 0.3; // any stick past this cancels auto/AI

    // ---------- Shooter: bang-bang ----------
    private static final double BANG_BANG_WINDOW = 100.0; // RPM
    private static final double BANG_HIGH_POWER = 1.0;    // power while too slow
    private static final double BANG_LOW_POWER = 0.0;     // power while too fast (tune)

    // ---------- Shooter: encoder ----------
    private static final double TICKS_PER_REV = 28.0;

    // ---------- Shooter: PIDF (same values as MyTeleOp — see the notes there) ----------
    private double kP = 0.0009;
    private double kI = 0;
    private double kD = 0;
    private double kF = 1.0 / 5800.0;
    private static final double MAX_I_POWER = 0.3;

    // ---------- Shooter: "ready" thresholds ----------
    private static final double AT_SPEED_TOLERANCE = 100.0;
    private static final double LOSE_SPEED_TOLERANCE = 150.0;

    // ---------- Auto drive: limits ----------
    // Every auto/AI command is capped to AUTO_SPEED_CAP and can only change by
    // AUTO_MAX_ACCEL per second (no sudden jerks if the model outputs jump).
    private static final double AUTO_SPEED_CAP = 0.6;  // tune
    private static final double AUTO_MAX_ACCEL = 3.0;  // command units per second (tune)

    // ---------- Auto drive: directions ----------
    // Flip these if the robot turns/strafes the wrong way during GO_TO_SHOOT/COLLECT.
    // Heading is CCW-positive. In the drive mix, a positive turn command turns right (CW),
    // so to fix a positive (CCW) heading error we need a negative turn -> sign = -1.
    private static final double HEADING_TURN_SIGN = -1.0; // tune on blocks
    // Positive leftErr = spot is to the robot's left. Positive strafe = right -> sign = -1.
    private static final double STRAFE_SIGN = -1.0;       // tune on blocks
    // Limelight tx is positive when the target is to the right -> turn right (positive).
    private static final double VISION_TURN_SIGN = 1.0;   // tune on blocks

    // ---------- Auto drive: GO_TO_SHOOT ----------
    private static final double K_POS = 0.05;          // power per inch of error (tune)
    private static final double K_HEAD = 1.2;          // power per radian of error (tune)
    private static final double POS_TOL_IN = 1.5;      // "at the spot" distance (inches)
    private static final double HEAD_TOL_RAD = Math.toRadians(3);
    private static final double SETTLE_SLACK = 2.0;    // leave SETTLE if error grows past tol * this

    // ---------- Shooting spot ----------
    // Save it live with gamepad1 left bumper, or type it here and set SHOOT_SPOT_DEFAULT_SET.
    // Coordinates are relative to where the robot was at INIT (start on the same mark!).
    private static final boolean SHOOT_SPOT_DEFAULT_SET = false;
    private static final double SHOOT_X_DEFAULT = 0.0;       // inches
    private static final double SHOOT_Y_DEFAULT = 0.0;       // inches
    private static final double SHOOT_HEADING_DEFAULT = 0.0; // radians

    // ---------- "Stopped" check (needed before firing) ----------
    private static final double STOP_SPEED_IN_S = 2.0;   // inches per second
    private static final double STOP_TURN_RAD_S = 0.15;  // radians per second
    private static final double VEL_FILTER = 0.7;        // 0 = no smoothing, closer to 1 = smoother

    // ---------- Firing ----------
    private static final double FIRE_MS = 2000; // how long to feed per volley (tune)

    // ---------- Collecting ----------
    private static final int BALLS_FULL = 4;
    private static final double SEARCH_TURN_POWER = 0.25;  // turn speed while searching (tune)
    private static final double COLLECT_FWD_POWER = 0.4;   // forward speed toward pollen (tune)
    private static final double K_TX = 0.015;              // turn power per degree of tx (tune)
    private static final double TX_SLOW_DEG = 20.0;        // full stop forward at this tx (deg)
    private static final double TARGET_LOST_MS = 600;      // keep rolling this long after losing it

    // ---------- Limelight ----------
    // Not mounted yet: everything works without it (hardwareMap.tryGet).
    private static final String LIMELIGHT_NAME = "limelight"; // CHECK the Robot Configuration name
    private static final int DETECTOR_PIPELINE = 0;           // pipeline slot with the detector
    private static final String POLLEN_CLASS = "";            // "" = accept any class name
    private static final double MIN_CONFIDENCE = 0.5;
    private static final long MAX_STALENESS_MS = 150;         // ignore old Limelight results

    // ---------- Ball counting ----------
    // No ball sensor exists. We count intake current spikes, but only if the camera saw
    // pollen close to the intake recently. A beam break named "intakeBeam" is used instead
    // if it is in the configuration.
    private static final double BALL_CURRENT_AMPS = 3.0;  // spike threshold (tune from logs)
    private static final double BALL_SPIKE_MIN_MS = 40;   // spike must last this long
    private static final double BALL_DEBOUNCE_MS = 300;   // min time between two balls
    private static final double NEAR_TY_DEG = -10.0;      // pollen below this ty = near intake (tune)
    private static final double NEAR_AREA = 1.0;          // and bigger than this (% of image, tune)
    private static final double VISION_GATE_MS = 800;     // how long "near intake" stays valid

    // ---------- Laptop link (AI mode) ----------
    private static final int HUB_PORT = 6100;              // laptop sends to 192.168.43.1:6100
    private static final double OBS_PERIOD_MS = 33;        // ~30 Hz observations
    private static final double LINK_TIMEOUT_MS = 200;     // older action = no model
    private static final long MAX_SEQ_LAG = 10;            // action for an obs older than this = stale
    private static final boolean AI_FALLBACK_TO_RULES = true; // false = stop instead
    private static final double AGREE_TOL = 0.15;          // shadow mode agreement tolerance

    // ---------- Logging ----------
    private static final String LOG_DIR = "/sdcard/FIRST/decision_logs";

    // =====================================================================
    // 2. HARDWARE
    // =====================================================================

    private DcMotor fr, fl, br, bl;

    private DcMotor transfer;
    private DcMotorEx intake;   // DcMotorEx so we can read its current
    private Servo gate;
    private DcMotor kickwheel;
    private DcMotorEx shooter;

    private Localizer odometry;        // Pinpoint, read directly (no Pedro follower)
    private Limelight3A limelight;     // null if not in the configuration
    private DigitalChannel intakeBeam; // null if not in the configuration

    // =====================================================================
    // 3. CONTROLLER STATE
    // =====================================================================

    private enum Mode { MANUAL, RULES, AI }
    private enum Source { MANUAL, RULES, MODEL, STOPPED }
    // Order matters: the ordinal is sent to the laptop and logged as "state".
    private enum State { IDLE, SEARCH, COLLECT, GO_TO_SHOOT, SETTLE, FIRE }

    private Mode mode = Mode.MANUAL;
    private State state = State.IDLE;
    private boolean cycling = false;   // true = keep repeating (A / AI), false = one shot (X)
    private String message = "";

    // ---------- Shooter (same as MyTeleOp) ----------
    private double targetRPM = 4100.0;
    private boolean dpadUpWas = false;
    private boolean dpadDownWas = false;
    private double integral = 0.0;
    private double previousRPM = 0.0;
    private double filteredRPM = 0.0;
    private boolean ready = false;

    // ---------- Feeding ----------
    private boolean feedingWas = false;
    private long feedStartTime = 0;
    private boolean firing = false;    // an auto/AI volley is in progress
    private double fireStartMs = 0;

    // ---------- Button press detection ----------
    private boolean aWas, xWas, yWas;
    private boolean saveSpotWas, ballsDownWas, ballsUpWas, ballsZeroWas, resetPoseWas;

    // ---------- Odometry ----------
    private double poseX, poseY, poseH;           // inches, inches, radians (CCW+)
    private double prevX, prevY, prevH;
    private double vx, vy, omega;                 // robot frame: forward, left, CCW (per second)
    private boolean stopped = true;

    // ---------- Shooting spot ----------
    private boolean hasShootSpot = SHOOT_SPOT_DEFAULT_SET;
    private double shootX = SHOOT_X_DEFAULT, shootY = SHOOT_Y_DEFAULT, shootH = SHOOT_HEADING_DEFAULT;
    private double fwdErr, leftErr, headErr;      // robot-frame error to the spot

    // ---------- Vision ----------
    private boolean haveTarget = false;
    private double tx, ty, area, conf;
    private double lastTargetMs = -1e9;
    private double lastNearIntakeMs = -1e9;

    // ---------- Ball counting ----------
    private int balls = 0;
    private double intakeAmps = 0;
    private boolean currentHighWas = false, spikeCounted = false, beamBrokenWas = false;
    private double spikeStartMs = 0, lastBallMs = -1e9;

    // ---------- Teacher command (what the rules would do) ----------
    private double autoFwd, autoStrafe, autoTurn;
    private boolean autoFeed;   // teacher wants to fire

    // ---------- Applied auto command (for the rate limit) ----------
    private double cmdFwd, cmdStrafe, cmdTurn;

    // ---------- Laptop link ----------
    private PolicyLink link;
    private double lastObsSentMs = -1e9;
    private long obsSeq = 0;
    private long agreeCount = 0, compareCount = 0;

    // ---------- Logging ----------
    private CsvLogger log;
    private int episode = 0;
    private String event = "";   // written once on the row where something happens
    private boolean newEpisodeNext = false;

    // =====================================================================
    // 4. MAIN PROGRAM
    // =====================================================================

    @Override
    public void runOpMode() {

        // -----------------------------------------------------------------
        // INIT: hardware map (same names as MyTeleOp)
        // -----------------------------------------------------------------
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");

        gate = hardwareMap.get(Servo.class, "gate");
        transfer = hardwareMap.get(DcMotor.class, "transfer");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        kickwheel = hardwareMap.get(DcMotor.class, "kickwheel");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        // Optional devices: null if they are not in the configuration
        limelight = hardwareMap.tryGet(Limelight3A.class, LIMELIGHT_NAME);
        intakeBeam = hardwareMap.tryGet(DigitalChannel.class, "intakeBeam");

        // Pinpoint odometry, using the tuned config from pedro/Constants.
        // Keep the robot still during INIT (the Pinpoint calibrates its IMU).
        odometry = new PinpointLocalizer(hardwareMap, Constants.localizerConfig);

        // -----------------------------------------------------------------
        // INIT: motor directions (same as MyTeleOp)
        // -----------------------------------------------------------------
        fl.setDirection(DcMotorSimple.Direction.FORWARD);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        transfer.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        kickwheel.setDirection(DcMotorSimple.Direction.REVERSE);

        // -----------------------------------------------------------------
        // INIT: run modes + zero-power behavior (same as MyTeleOp)
        // -----------------------------------------------------------------
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        gate.setPosition(GATE_CLOSED);

        // -----------------------------------------------------------------
        // INIT: Limelight, laptop link, log file
        // -----------------------------------------------------------------
        if (limelight != null) {
            limelight.pipelineSwitch(DETECTOR_PIPELINE);
            limelight.setPollRateHz(100);
            limelight.start();
        }

        link = new PolicyLink(HUB_PORT);
        String linkStatus = link.start() ? "listening on port " + HUB_PORT : "FAILED to open port";

        log = new CsvLogger(LOG_DIR);
        String logStatus = log.open() ? log.path() : "FAILED to open log";

        telemetry.addLine("Decision model teleop");
        telemetry.addData("Limelight", limelight != null ? "found" : "not found (OK)");
        telemetry.addData("Beam break", intakeBeam != null ? "found" : "not found (current spikes)");
        telemetry.addData("Laptop link", linkStatus);
        telemetry.addData("Log", logStatus);
        telemetry.update();

        waitForStart();

        gate.setPosition(GATE_CLOSED);
        odometry.setPose(Pose.zero());   // field frame = where the robot is right now
        odometry.update();
        readPose();
        prevX = poseX; prevY = poseY; prevH = poseH;

        long previousTime = System.nanoTime();

        // -----------------------------------------------------------------
        // MAIN LOOP
        // -----------------------------------------------------------------
        while (opModeIsActive()) {

            // ---- Timing: ONE timestamp per loop ----
            long now = System.nanoTime();
            double dt = (now - previousTime) / 1e9;
            previousTime = now;
            double nowMs = now / 1e6;

            // =============================================================
            // BUTTONS: mode changes + cancel
            // =============================================================
            boolean a = gamepad1.a, x = gamepad1.x, y = gamepad1.y;
            boolean stickMoved = Math.abs(gamepad1.left_stick_x) > STICK_CANCEL
                    || Math.abs(gamepad1.left_stick_y) > STICK_CANCEL
                    || Math.abs(gamepad1.right_stick_x) > STICK_CANCEL
                    || Math.abs(gamepad1.right_stick_y) > STICK_CANCEL;

            if (mode != Mode.MANUAL && (gamepad1.b || stickMoved)) {
                cancelToManual(gamepad1.b ? "cancelled (B)" : "cancelled (stick)");
            } else if (a && !aWas) {
                if (limelight == null) {
                    message = "A needs the Limelight (not found). Use X.";
                } else if (!hasShootSpot) {
                    message = "No shooting spot: gamepad1 left bumper to save one";
                } else {
                    startAuto(Mode.RULES, true, State.SEARCH);
                }
            } else if (x && !xWas) {
                if (!hasShootSpot) {
                    message = "No shooting spot: gamepad1 left bumper to save one";
                } else {
                    startAuto(Mode.RULES, false, State.GO_TO_SHOOT);
                }
            } else if (y && !yWas) {
                if (mode == Mode.AI) {
                    cancelToManual("AI mode off");
                } else {
                    startAuto(Mode.AI, true, State.SEARCH);
                }
            }
            aWas = a; xWas = x; yWas = y;

            // ---- Shooting spot, ball count, odometry reset (gamepad1, or gamepad2 as backup) ----
            boolean saveSpot = gamepad1.left_bumper || gamepad2.dpad_up;
            boolean ballsUp = gamepad1.dpad_right || gamepad2.dpad_right;
            boolean ballsDown = gamepad1.dpad_left || gamepad2.dpad_left;
            boolean ballsZero = gamepad1.right_bumper || gamepad2.a;
            boolean resetPose = gamepad1.back || gamepad2.back;

            if (saveSpot && !saveSpotWas) {
                shootX = poseX; shootY = poseY; shootH = poseH;
                hasShootSpot = true;
                message = String.format(Locale.US, "Shooting spot saved: %.1f, %.1f, %.0f deg",
                        shootX, shootY, Math.toDegrees(shootH));
            }
            if (ballsUp && !ballsUpWas) balls = Math.min(BALLS_FULL, balls + 1);
            if (ballsDown && !ballsDownWas) balls = Math.max(0, balls - 1);
            if (ballsZero && !ballsZeroWas) balls = 0;
            if (resetPose && !resetPoseWas) {
                odometry.setPose(Pose.zero());
                message = "Odometry reset to (0, 0, 0)";
            }
            saveSpotWas = saveSpot; ballsUpWas = ballsUp; ballsDownWas = ballsDown;
            ballsZeroWas = ballsZero; resetPoseWas = resetPose;

            // =============================================================
            // ODOMETRY + VELOCITY
            // =============================================================
            odometry.update();
            readPose();
            if (dt > 0) {
                // Field-frame velocity from the pose change, rotated into the robot frame
                double dxField = (poseX - prevX) / dt;
                double dyField = (poseY - prevY) / dt;
                double cos = Math.cos(poseH), sin = Math.sin(poseH);
                double vxRaw = dxField * cos + dyField * sin;    // forward
                double vyRaw = -dxField * sin + dyField * cos;   // left
                double omegaRaw = wrapAngle(poseH - prevH) / dt;
                vx = VEL_FILTER * vx + (1 - VEL_FILTER) * vxRaw;
                vy = VEL_FILTER * vy + (1 - VEL_FILTER) * vyRaw;
                omega = VEL_FILTER * omega + (1 - VEL_FILTER) * omegaRaw;
            }
            prevX = poseX; prevY = poseY; prevH = poseH;
            stopped = Math.hypot(vx, vy) < STOP_SPEED_IN_S && Math.abs(omega) < STOP_TURN_RAD_S;

            // Error to the shooting spot in the robot frame (zeros if no spot saved)
            if (hasShootSpot) {
                double dx = shootX - poseX, dy = shootY - poseY;
                double cos = Math.cos(poseH), sin = Math.sin(poseH);
                fwdErr = dx * cos + dy * sin;
                leftErr = -dx * sin + dy * cos;
                headErr = wrapAngle(shootH - poseH);
            } else {
                fwdErr = 0; leftErr = 0; headErr = 0;
            }

            // =============================================================
            // VISION: best pollen detection (zeros if none / no Limelight)
            // =============================================================
            readVision(nowMs);

            // =============================================================
            // BALL COUNTING
            // =============================================================
            countBalls(nowMs);

            // =============================================================
            // TEACHER: rule-based state machine (runs in RULES and AI)
            // =============================================================
            runTeacher();

            // =============================================================
            // LAPTOP LINK: send the observation, read the latest action
            // =============================================================
            if (nowMs - lastObsSentMs >= OBS_PERIOD_MS) {
                lastObsSentMs = nowMs;
                obsSeq++;
                link.sendObservation(String.format(Locale.US,
                        "O,%d,%d,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.4f,%.3f,%.3f,%d,%d",
                        obsSeq, (long) nowMs, haveTarget ? 1 : 0, tx, ty, area, conf,
                        fwdErr, leftErr, headErr, vx, vy, balls, state.ordinal()), obsSeq);
            }
            PolicyLink.Action model = link.latestAction(nowMs, LINK_TIMEOUT_MS, obsSeq - MAX_SEQ_LAG);

            // Shadow mode: compare the model to the teacher whenever both exist
            boolean teacherActive = mode != Mode.MANUAL && state != State.IDLE;
            if (teacherActive && model != null) {
                compareCount++;
                if (Math.abs(model.fwd - autoFwd) < AGREE_TOL
                        && Math.abs(model.strafe - autoStrafe) < AGREE_TOL
                        && Math.abs(model.turn - autoTurn) < AGREE_TOL
                        && model.fire == autoFeed) agreeCount++;
            }

            // =============================================================
            // CHOOSE + APPLY THE DRIVE COMMAND
            // =============================================================
            Source source;
            if (mode == Mode.MANUAL) source = Source.MANUAL;
            else if (mode == Mode.AI && model != null) source = Source.MODEL;
            else if (mode == Mode.AI && !AI_FALLBACK_TO_RULES) source = Source.STOPPED;
            else source = Source.RULES;

            double flPower, frPower, blPower, brPower;
            if (source == Source.MANUAL) {
                // ---- Exactly MyTeleOp: cubed sticks, strafe boosted 10% ----
                double yIn = Math.pow(-gamepad1.left_stick_y, 3);
                double xIn = Math.pow(gamepad1.left_stick_x, 3) * 1.1;
                double rx = Math.pow(gamepad1.right_stick_x, 3);
                double clamp = Math.max(Math.abs(xIn) + Math.abs(yIn) + Math.abs(rx), 1);
                flPower = (yIn + xIn + rx) / clamp;
                frPower = (yIn - xIn - rx) / clamp;
                blPower = (yIn - xIn + rx) / clamp;
                brPower = (yIn + xIn - rx) / clamp;
                cmdFwd = 0; cmdStrafe = 0; cmdTurn = 0; // auto ramps start from zero
            } else {
                // ---- Auto: pick the wanted command ----
                double wantFwd = 0, wantStrafe = 0, wantTurn = 0;
                boolean fireRequest = false;
                if (source == Source.RULES) {
                    wantFwd = autoFwd; wantStrafe = autoStrafe; wantTurn = autoTurn;
                    fireRequest = autoFeed;
                } else if (source == Source.MODEL) {
                    wantFwd = model.fwd; wantStrafe = model.strafe; wantTurn = model.turn;
                    fireRequest = model.fire;
                }

                // Fire is only a REQUEST: never without ready + stopped
                if (fireRequest && !firing && ready && stopped) startFire(nowMs);

                // Hold still while firing
                if (firing) { wantFwd = 0; wantStrafe = 0; wantTurn = 0; }

                // Speed cap, then rate limit
                double step = AUTO_MAX_ACCEL * dt;
                cmdFwd = approach(cmdFwd, clip(wantFwd, AUTO_SPEED_CAP), step);
                cmdStrafe = approach(cmdStrafe, clip(wantStrafe, AUTO_SPEED_CAP), step);
                cmdTurn = approach(cmdTurn, clip(wantTurn, AUTO_SPEED_CAP), step);

                // Same mecanum mix as manual (no cubing — commands are already linear)
                double clamp = Math.max(Math.abs(cmdStrafe) + Math.abs(cmdFwd) + Math.abs(cmdTurn), 1);
                flPower = (cmdFwd + cmdStrafe + cmdTurn) / clamp;
                frPower = (cmdFwd - cmdStrafe - cmdTurn) / clamp;
                blPower = (cmdFwd - cmdStrafe + cmdTurn) / clamp;
                brPower = (cmdFwd + cmdStrafe - cmdTurn) / clamp;
            }

            fl.setPower(flPower * mult);
            fr.setPower(frPower * mult);
            bl.setPower(blPower * mult);
            br.setPower(brPower * mult);

            // ---- End of a volley ----
            if (firing && nowMs - fireStartMs >= FIRE_MS) finishFire();

            // =============================================================
            // SHOOTER (bang-bang + PIDF) — unchanged from MyTeleOp
            // =============================================================
            if (gamepad1.dpad_up && !dpadUpWas) targetRPM += 50;
            if (gamepad1.dpad_down && !dpadDownWas) targetRPM = Math.max(0, targetRPM - 50);
            dpadUpWas = gamepad1.dpad_up;
            dpadDownWas = gamepad1.dpad_down;

            boolean shooting = true; // flywheel always on

            double rawRPM = shooter.getVelocity() / TICKS_PER_REV * 60.0;
            filteredRPM = 0.2 * filteredRPM + 0.8 * rawRPM;

            double error = targetRPM - filteredRPM;
            double output = 0;

            boolean bangBangActive = shooting && Math.abs(error) > BANG_BANG_WINDOW;

            if (bangBangActive) {
                output = (error > 0) ? BANG_HIGH_POWER : BANG_LOW_POWER;
                shooter.setPower(output);
                integral = 0;
            } else if (shooting && dt > 0) {
                double pTerm = kP * error;
                double dTerm = -kD * (filteredRPM - previousRPM) / dt;
                double fTerm = kF * targetRPM;
                double unsaturated = pTerm + kI * integral + dTerm + fTerm;
                boolean saturatedHigh = unsaturated > 1.0 && error > 0;
                boolean saturatedLow = unsaturated < 0.0 && error < 0;
                if (!saturatedHigh && !saturatedLow) {
                    integral += error * dt;
                    if (kI != 0) {
                        double maxIntegral = MAX_I_POWER / Math.abs(kI);
                        integral = Math.max(-maxIntegral, Math.min(maxIntegral, integral));
                    }
                }
                double iTerm = kI * integral;
                output = pTerm + iTerm + dTerm + fTerm;
                output = Math.max(0.0, Math.min(1.0, output));
                shooter.setPower(output);
            } else {
                shooter.setPower(0);
                integral = 0;
            }
            previousRPM = filteredRPM;

            // =============================================================
            // READY CHECK (unchanged from MyTeleOp)
            // =============================================================
            if (!shooting) {
                ready = false;
            } else if (!ready && Math.abs(error) < AT_SPEED_TOLERANCE) {
                ready = true;
            } else if (ready && Math.abs(error) > LOSE_SPEED_TOLERANCE) {
                ready = false;
            }

            // =============================================================
            // GATE + KICKWHEEL + INTAKE + TRANSFER
            // =============================================================
            // Feeding = right trigger (driver) OR an auto/AI volley
            boolean feeding = gamepad1.right_trigger > TRIGGER_THRESHOLD || firing;

            if (feeding && !feedingWas) feedStartTime = System.nanoTime();
            feedingWas = feeding;
            boolean feedReady = feeding
                    && (System.nanoTime() - feedStartTime) / 1e6 >= FEED_DELAY_MS;

            gate.setPosition(feeding ? GATE_OPEN : GATE_CLOSED);
            kickwheel.setPower(0.5); // constant 50%; use (feedReady ? 1 : 0) to run only while feeding

            intake.setPower(INTAKE_POWER);
            transfer.setPower(feeding ? FEED_POWER : IDLE_POWER);

            // =============================================================
            // LOGGING (one row per loop)
            // =============================================================
            log.row(new Object[]{
                    (long) nowMs, dt * 1000, episode, event, mode, source, state.ordinal(), state,
                    poseX, poseY, poseH, vx, vy, omega,
                    balls, haveTarget ? 1 : 0, tx, ty, area, conf,
                    fwdErr, leftErr, headErr, hasShootSpot ? 1 : 0,
                    targetRPM, filteredRPM, ready ? 1 : 0, feeding ? 1 : 0, firing ? 1 : 0,
                    intakeAmps, stopped ? 1 : 0,
                    autoFwd, autoStrafe, autoTurn, autoFeed ? 1 : 0, teacherActive ? 1 : 0,
                    model != null ? 1 : 0,
                    model != null ? model.fwd : 0, model != null ? model.strafe : 0,
                    model != null ? model.turn : 0, model != null && model.fire ? 1 : 0,
                    cmdFwd, cmdStrafe, cmdTurn, obsSeq
            });
            event = "";
            if (newEpisodeNext) { episode++; newEpisodeNext = false; }

            // =============================================================
            // TELEMETRY (one update per loop)
            // =============================================================
            telemetry.addData("Mode / source", mode + " / " + source);
            telemetry.addData("State", state + (cycling ? " (cycling)" : ""));
            telemetry.addData("Balls", balls + " / " + BALLS_FULL);
            telemetry.addData("Pose", String.format(Locale.US, "%.1f, %.1f, %.0f deg",
                    poseX, poseY, Math.toDegrees(poseH)));
            telemetry.addData("Shooting spot", hasShootSpot
                    ? String.format(Locale.US, "fwd %.1f  left %.1f  head %.0f deg",
                    fwdErr, leftErr, Math.toDegrees(headErr))
                    : "not saved (gamepad1 left bumper)");
            telemetry.addData("Stopped", stopped);
            telemetry.addData("Pollen", limelight == null ? "no Limelight"
                    : haveTarget ? String.format(Locale.US, "tx %.1f  ty %.1f  area %.2f", tx, ty, area)
                    : "none");
            telemetry.addData("Laptop", link.describe(nowMs) + (compareCount > 0
                    ? String.format(Locale.US, "  agree %.0f%%", 100.0 * agreeCount / compareCount) : ""));
            telemetry.addData("Shooter mode", bangBangActive ? "BANG-BANG" : "PIDF");
            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Actual RPM", filteredRPM);
            telemetry.addData("Ready (at speed)", ready);
            telemetry.addData("Feeding", feeding);
            telemetry.addData("Intake amps", String.format(Locale.US, "%.2f", intakeAmps));
            if (!message.isEmpty()) telemetry.addLine(message);
            telemetry.update();
        }

        // -----------------------------------------------------------------
        // STOP: close everything
        // -----------------------------------------------------------------
        link.stop();
        log.close();
        if (limelight != null) limelight.stop();
    }

    // =====================================================================
    // TEACHER: the rule-based state machine. Sets autoFwd/autoStrafe/autoTurn
    // and autoFeed (= "I would fire now"). It never touches motors itself.
    // =====================================================================
    private void runTeacher() {
        autoFwd = 0; autoStrafe = 0; autoTurn = 0; autoFeed = false;
        if (mode == Mode.MANUAL) return;

        double nowMs = System.nanoTime() / 1e6;

        switch (state) {
            case IDLE:
                break;

            case SEARCH:
                if (balls >= BALLS_FULL) { state = State.GO_TO_SHOOT; break; }
                if (haveTarget) { state = State.COLLECT; break; }
                autoTurn = SEARCH_TURN_POWER;
                break;

            case COLLECT:
                if (balls >= BALLS_FULL) { state = State.GO_TO_SHOOT; break; }
                if (!haveTarget) {
                    // Pollen drops out of view right before the intake grabs it,
                    // so keep rolling forward for a moment before giving up.
                    if (nowMs - lastTargetMs > TARGET_LOST_MS) state = State.SEARCH;
                    else autoFwd = COLLECT_FWD_POWER;
                    break;
                }
                autoTurn = VISION_TURN_SIGN * K_TX * tx;
                autoFwd = COLLECT_FWD_POWER * Math.max(0, 1 - Math.abs(tx) / TX_SLOW_DEG);
                break;

            case GO_TO_SHOOT:
                if (!hasShootSpot) {
                    // RULES: give control back. AI: the teacher just stops labelling.
                    if (mode == Mode.RULES) cancelToManual("No shooting spot saved");
                    else state = State.IDLE;
                    break;
                }
                if (atSpot(1.0)) { state = State.SETTLE; break; }
                autoFwd = K_POS * fwdErr;
                autoStrafe = STRAFE_SIGN * K_POS * leftErr;
                autoTurn = HEADING_TURN_SIGN * K_HEAD * headErr;
                break;

            case SETTLE:
                if (!atSpot(SETTLE_SLACK)) { state = State.GO_TO_SHOOT; break; }
                // Hold still; fire once stopped and the flywheel is at speed
                autoFeed = stopped && ready;
                break;

            case FIRE:
                autoFeed = true;
                break;
        }
    }

    private boolean atSpot(double slack) {
        return Math.hypot(fwdErr, leftErr) < POS_TOL_IN * slack
                && Math.abs(headErr) < HEAD_TOL_RAD * slack;
    }

    private void startAuto(Mode newMode, boolean repeat, State first) {
        mode = newMode;
        cycling = repeat;
        state = first;
        firing = false;
        episode++;
        event = "start";
        message = "";
    }

    private void cancelToManual(String why) {
        if (mode != Mode.MANUAL) event = "cancel";
        mode = Mode.MANUAL;
        state = State.IDLE;
        firing = false;
        cycling = false;
        message = why;
    }

    private void startFire(double nowMs) {
        firing = true;
        fireStartMs = nowMs;
        state = State.FIRE;
    }

    private void finishFire() {
        firing = false;
        balls = 0;
        event = "shot";
        if (cycling) {
            state = State.SEARCH;
            newEpisodeNext = true; // next cycle = new episode (after this "shot" row is logged)
        } else {
            mode = Mode.MANUAL;
            state = State.IDLE;
            message = "Shot fired";
        }
    }

    // =====================================================================
    // SENSORS
    // =====================================================================

    /** Pedro 3: localizer.pose() gives x(), y() in inches and heading() in radians. */
    private void readPose() {
        Pose p = odometry.pose();
        poseX = p.x();
        poseY = p.y();
        poseH = p.heading();
    }

    /** Picks the biggest (closest) pollen detection. Zeros if none. */
    private void readVision(double nowMs) {
        haveTarget = false;
        tx = 0; ty = 0; area = 0; conf = 0;
        if (limelight == null) return;

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid() || result.getStaleness() > MAX_STALENESS_MS) return;

        List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();
        if (detections == null) return;

        LLResultTypes.DetectorResult best = null;
        for (LLResultTypes.DetectorResult d : detections) {
            if (d.getConfidence() < MIN_CONFIDENCE) continue;
            if (!POLLEN_CLASS.isEmpty() && !POLLEN_CLASS.equals(d.getClassName())) continue;
            if (best == null || d.getTargetArea() > best.getTargetArea()) best = d;
        }
        if (best == null) return;

        haveTarget = true;
        tx = best.getTargetXDegrees();
        ty = best.getTargetYDegrees();
        area = best.getTargetArea();
        conf = best.getConfidence();
        lastTargetMs = nowMs;
        if (ty < NEAR_TY_DEG && area > NEAR_AREA) lastNearIntakeMs = nowMs;
    }

    /** Beam break if present, otherwise intake current spikes gated by vision. */
    private void countBalls(double nowMs) {
        intakeAmps = intake.getCurrent(CurrentUnit.AMPS);
        if (firing) return; // balls are leaving, not arriving

        boolean newBall = false;
        if (intakeBeam != null) {
            boolean broken = !intakeBeam.getState(); // most beam breaks read false when blocked
            newBall = broken && !beamBrokenWas;
            beamBrokenWas = broken;
        } else {
            boolean high = intakeAmps > BALL_CURRENT_AMPS;
            if (high && !currentHighWas) { spikeStartMs = nowMs; spikeCounted = false; }
            currentHighWas = high;
            // Without a Limelight we can't gate, so every spike counts
            boolean visionOk = limelight == null || nowMs - lastNearIntakeMs < VISION_GATE_MS;
            if (high && !spikeCounted && nowMs - spikeStartMs >= BALL_SPIKE_MIN_MS && visionOk) {
                newBall = true;
                spikeCounted = true;
            }
        }
        if (newBall && nowMs - lastBallMs > BALL_DEBOUNCE_MS) {
            balls = Math.min(BALLS_FULL, balls + 1);
            lastBallMs = nowMs;
        }
    }

    // =====================================================================
    // 5. HELPERS
    // =====================================================================

    private static double wrapAngle(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }

    private static double clip(double v, double max) {
        if (Double.isNaN(v)) return 0;
        return Math.max(-max, Math.min(max, v));
    }

    /** Moves current toward target by at most step (the rate limit). */
    private static double approach(double current, double target, double step) {
        if (target > current + step) return current + step;
        if (target < current - step) return current - step;
        return target;
    }

    // ---------------------------------------------------------------------
    // CSV logger: one file per run in LOG_DIR. Decimals always use dots.
    // Copy the files to the laptop with:  adb pull /sdcard/FIRST/decision_logs logs
    // ---------------------------------------------------------------------
    private static class CsvLogger {
        private static final String HEADER = "t_ms,loop_ms,episode,event,mode,source,state,state_name,"
                + "x,y,heading,vx,vy,omega,"
                + "balls,have_target,tx,ty,area,conf,"
                + "shoot_fwd_err,shoot_left_err,shoot_head_err,has_spot,"
                + "target_rpm,rpm,ready,feeding,firing,intake_amps,stopped,"
                + "act_fwd,act_strafe,act_turn,act_fire,teacher_active,"
                + "model_ok,model_fwd,model_strafe,model_turn,model_fire,"
                + "cmd_fwd,cmd_strafe,cmd_turn,obs_seq";

        private final String dir;
        private File file;
        private BufferedWriter out;
        private int rows = 0;

        CsvLogger(String dir) { this.dir = dir; }

        boolean open() {
            try {
                File folder = new File(dir);
                if (!folder.exists() && !folder.mkdirs()) return false;
                String stamp = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
                file = new File(folder, "run_" + stamp + ".csv");
                out = new BufferedWriter(new FileWriter(file));
                out.write(HEADER);
                out.newLine();
                return true;
            } catch (IOException e) {
                out = null;
                return false;
            }
        }

        String path() { return file == null ? "" : file.getAbsolutePath(); }

        void row(Object[] values) {
            if (out == null) return;
            StringBuilder sb = new StringBuilder(400);
            for (int i = 0; i < values.length; i++) {
                if (i > 0) sb.append(',');
                Object v = values[i];
                if (v instanceof Double) sb.append(String.format(Locale.US, "%.4f", (Double) v));
                else sb.append(v);
            }
            try {
                out.write(sb.toString());
                out.newLine();
                if (++rows % 30 == 0) out.flush(); // about once a second
            } catch (IOException e) {
                out = null; // stop logging rather than crash the OpMode
            }
        }

        void close() {
            if (out == null) return;
            try { out.close(); } catch (IOException ignored) { }
            out = null;
        }
    }

    // ---------------------------------------------------------------------
    // Laptop link: a background thread with one UDP socket.
    //   Laptop -> Hub:  "H"  (hello, so the Hub learns the laptop's address)
    //                   "A,seq,fwd,strafe,turn,fire"  (seq = the observation it answers)
    //   Hub -> laptop:  "O,seq,t_ms,have_target,tx,ty,area,conf,
    //                    shoot_fwd_err,shoot_left_err,shoot_head_err,vx,vy,balls,state"
    // The OpMode only reads the latest accepted action; old or out-of-order ones are dropped.
    // ---------------------------------------------------------------------
    private static class PolicyLink implements Runnable {

        static class Action {
            final long seq;
            final double fwd, strafe, turn;
            final boolean fire;
            final double receivedMs;

            Action(long seq, double fwd, double strafe, double turn, boolean fire, double receivedMs) {
                this.seq = seq; this.fwd = fwd; this.strafe = strafe; this.turn = turn;
                this.fire = fire; this.receivedMs = receivedMs;
            }
        }

        private final int port;
        private DatagramSocket socket;
        private Thread thread;
        private volatile boolean running = false;
        private volatile InetSocketAddress laptop = null;
        private volatile String pendingObs = null;
        private volatile long lastObsSeq = 0;
        private volatile double lastPacketMs = -1e9;
        private Action latest = null;          // guarded by "this"
        private long rejected = 0;

        PolicyLink(int port) { this.port = port; }

        boolean start() {
            try {
                socket = new DatagramSocket(port);
                socket.setSoTimeout(5);
            } catch (IOException e) {
                return false;
            }
            running = true;
            thread = new Thread(this, "PolicyLink");
            thread.setDaemon(true);
            thread.start();
            return true;
        }

        void stop() {
            running = false;
            if (socket != null) socket.close();
        }

        /** Called by the OpMode; the thread sends it (no networking on the loop thread). */
        void sendObservation(String obs, long seq) {
            lastObsSeq = seq;
            pendingObs = obs;
        }

        /** Latest action if it is fresh and answers a recent observation, else null. */
        synchronized Action latestAction(double nowMs, double timeoutMs, long minSeq) {
            if (latest == null) return null;
            if (nowMs - latest.receivedMs > timeoutMs) return null;
            if (latest.seq < minSeq) return null;
            return latest;
        }

        String describe(double nowMs) {
            if (socket == null) return "port not open";
            if (laptop == null) return "waiting for laptop";
            double age = nowMs - lastPacketMs;
            return String.format(Locale.US, "%s (last %.0f ms ago, dropped %d)",
                    age < 500 ? "connected" : "QUIET", age, rejected);
        }

        @Override
        public void run() {
            byte[] buf = new byte[256];
            while (running) {
                // ---- Receive (5 ms timeout so sending stays responsive) ----
                try {
                    DatagramPacket p = new DatagramPacket(buf, buf.length);
                    socket.receive(p);
                    handle(new String(p.getData(), 0, p.getLength(), StandardCharsets.US_ASCII).trim(),
                            (InetSocketAddress) p.getSocketAddress());
                } catch (SocketTimeoutException ignored) {
                    // nothing arrived
                } catch (IOException e) {
                    if (!running) break;
                }

                // ---- Send the newest observation, if any ----
                String obs = pendingObs;
                InetSocketAddress to = laptop;
                if (obs != null && to != null) {
                    pendingObs = null;
                    try {
                        byte[] data = obs.getBytes(StandardCharsets.US_ASCII);
                        socket.send(new DatagramPacket(data, data.length, to));
                    } catch (IOException ignored) {
                        // drop it; the next one comes in 33 ms
                    }
                }
            }
        }

        private void handle(String msg, InetSocketAddress from) {
            laptop = from;
            lastPacketMs = System.nanoTime() / 1e6;
            if (!msg.startsWith("A,")) return; // hello or unknown
            String[] f = msg.split(",");
            if (f.length < 6) { rejected++; return; }
            try {
                long seq = Long.parseLong(f[1]);
                double fwd = Double.parseDouble(f[2]);
                double strafe = Double.parseDouble(f[3]);
                double turn = Double.parseDouble(f[4]);
                boolean fire = Integer.parseInt(f[5].trim()) != 0;
                if (Double.isNaN(fwd) || Double.isNaN(strafe) || Double.isNaN(turn)) { rejected++; return; }
                synchronized (this) {
                    // Out of order, or answering an observation we never sent
                    if ((latest != null && seq <= latest.seq) || seq > lastObsSeq) { rejected++; return; }
                    latest = new Action(seq, fwd, strafe, turn, fire, lastPacketMs);
                }
            } catch (NumberFormatException e) {
                rejected++;
            }
        }
    }
}
