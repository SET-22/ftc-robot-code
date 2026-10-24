// package here — must match your folder
package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/*
 * ============================== OVERVIEW ==============================
 *
 * CONTROLS (gamepad1)
 *   Left stick ........ drive (forward/back + strafe)
 *   Right stick X ..... turn
 *   Right trigger ..... hold to FEED: opens the gate and boosts the transfer to full power
 *   D-pad up/down ..... shooter target RPM +50 / -50 per press
 *
 * ALWAYS RUNNING (after PLAY)
 *   Shooter ........... PIDF holds targetRPM (bang-bang gets it close first)
 *   Intake ............ INTAKE_POWER (100%)
 *   Transfer .......... IDLE_POWER (15%), or FEED_POWER (100%) while feeding
 *   Kickwheel ......... constant 50% (see the kickwheel line in the loop)
 *
 * FILE LAYOUT
 *   1. Tunable constants (all the "tune" values live here)
 *   2. Hardware objects
 *   3. Controller state (variables that change while running)
 *   4. runOpMode: init, then the main loop
 *        - Drive
 *        - Shooter (bang-bang + PIDF)
 *        - Ready check
 *        - Gate + kickwheel
 *        - Intake + transfer
 *        - Telemetry
 */
@TeleOp(name = "My_TeleOp", group = "Practice")
public class MyTeleOp extends LinearOpMode {

    // =====================================================================
    // 1. TUNABLE CONSTANTS
    // =====================================================================

    // ---------- Drive ----------
    private double mult = 0.9; // overall drive speed multiplier (0 to 1)

    // ---------- Gate ----------
    // GATE_CLOSED = in place (blocking), where it sits from init.
    // GATE_OPEN   = moved out of the way while the right trigger is held.
    // Swap/tune these to match your mechanism.
    private static final double GATE_CLOSED = 0.7; // tune
    private static final double GATE_OPEN = 0.0;   // tune

    // Delay between the gate starting to open and the kickwheel being allowed to feed,
    // so the gate is out of the way before balls get pushed through.
    // (Used to compute feedReady below.)
    private static final double FEED_DELAY_MS = 250; // tune

    // ---------- Intake + transfer ----------
    // Intake always runs at INTAKE_POWER.
    // Transfer always runs: IDLE_POWER normally, FEED_POWER while the gate is open.
    private static final double INTAKE_POWER = 1.0; // intake, always on (tune)
    private static final double IDLE_POWER = 0.15;  // transfer idle (tune)
    private static final double FEED_POWER = 1.0;   // transfer while feeding (tune)

    // ---------- Controls ----------
    // Triggers count as pressed past this value (0 to 1)
    private static final double TRIGGER_THRESHOLD = 0.5;

    // ---------- Shooter: bang-bang ----------
    // Bang-bang is automatic: while the RPM is MORE than BANG_BANG_WINDOW away from the
    // target it runs full power (too slow) or low power (too fast) to get there quickly.
    // Once within BANG_BANG_WINDOW of the target it turns off and PIDF takes over.
    private static final double BANG_BANG_WINDOW = 100.0; // RPM
    private static final double BANG_HIGH_POWER = 1.0;    // power while too slow
    private static final double BANG_LOW_POWER = 0.0;     // power while too fast (tune)

    // ---------- Shooter: encoder ----------
    private static final double TICKS_PER_REV = 28.0;

    // ---------- Shooter: PIDF ----------
    // output = kP*error + kI*integral(error) + kD*(rate of change) + kF*target
    //
    // Tune in this order: kF, then kP, then kD, then kI.
    //   kF: baseline power for the target speed. About 1 / (RPM at full power).
    //       Motor limit is about 5800 RPM at full power, so 1/5800.
    //       If it settles ABOVE the target with kP = kI = 0, lower kF. Below, raise it.
    //   kP: raise until it reaches the target fast; lower if it oscillates.
    //   kD: raise (slowly) if it overshoots on spin-up; lower if it gets jittery.
    //   kI: raise only if it settles a little off the target and stays there.
    private double kP = 0.0009;
    private double kI = 0;
    private double kD = 0;
    private double kF = 1.0 / 5800.0;

    // Largest power the I term is allowed to add (anti-windup)
    private static final double MAX_I_POWER = 0.3;

    // ---------- Shooter: "ready" thresholds ----------
    // Ready when within AT_SPEED_TOLERANCE of the target; stays ready until the error
    // grows past LOSE_SPEED_TOLERANCE (the gap prevents the flag from flickering).
    private static final double AT_SPEED_TOLERANCE = 100.0;
    private static final double LOSE_SPEED_TOLERANCE = 150.0;

    // =====================================================================
    // 2. HARDWARE
    // =====================================================================

    // ---------- Drive motors ----------
    private DcMotor fr;
    private DcMotor fl;
    private DcMotor br;
    private DcMotor bl;

    // ---------- Mechanisms ----------
    private DcMotor transfer;   // 15% idle, 100% while the gate is open
    private DcMotor intake;     // always 100%
    private Servo gate;         // closed from init; opens while the right trigger is held
    private DcMotor kickwheel;  // currently runs constantly at 50%
    private DcMotorEx shooter;  // flywheel, held at targetRPM by PIDF

    // =====================================================================
    // 3. CONTROLLER STATE (changes while running)
    // =====================================================================

    // ---------- Shooter target ----------
    private double targetRPM = 4100.0;   // D-pad up/down changes it by 50
    private boolean dpadUpWas = false;   // previous button states, for press detection
    private boolean dpadDownWas = false;

    // ---------- Shooter PIDF memory ----------
    private double integral = 0.0;       // running total of error over time (I term)
    private double previousRPM = 0.0;    // last loop's RPM, for the D term
    private double filteredRPM = 0.0;    // smoothed RPM reading
    private boolean ready = false;       // true when the flywheel is at speed

    // ---------- Feeding (right trigger) ----------
    private boolean feedingWas = false;  // previous trigger state, for press detection
    private long feedStartTime = 0;      // when the trigger was first pressed (nanoseconds)

    // =====================================================================
    // 4. MAIN PROGRAM
    // =====================================================================

    @Override
    public void runOpMode() {

        // -----------------------------------------------------------------
        // INIT: hardware map
        // -----------------------------------------------------------------
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");

        gate = hardwareMap.get(Servo.class, "gate");
        transfer = hardwareMap.get(DcMotor.class, "transfer");
        intake = hardwareMap.get(DcMotor.class, "intake");
        kickwheel = hardwareMap.get(DcMotor.class, "kickwheel");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        // -----------------------------------------------------------------
        // INIT: motor directions
        // -----------------------------------------------------------------
        fl.setDirection(DcMotorSimple.Direction.FORWARD);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        transfer.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        kickwheel.setDirection(DcMotorSimple.Direction.REVERSE); // flip if it spins the wrong way

        // If Actual RPM reads negative when it spins the right way:
        // shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        // -----------------------------------------------------------------
        // INIT: run modes
        // -----------------------------------------------------------------
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // our PID sets the power

        // Make sure intake/transfer are in plain power mode (rules out a leftover run mode)
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // -----------------------------------------------------------------
        // INIT: zero-power behavior (what each motor does at 0 power)
        // -----------------------------------------------------------------
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // -----------------------------------------------------------------
        // INIT: starting positions
        // -----------------------------------------------------------------
        // Gate is activated (closed) immediately at init, not left at 0.0
        gate.setPosition(GATE_CLOSED);

        telemetry.addLine("start");
        telemetry.update();

        waitForStart(); // waits for PLAY

        // Re-command the gate the moment the match starts
        gate.setPosition(GATE_CLOSED);

        // Set the timer AFTER start so the first dt isn't huge
        long previousTime = System.nanoTime();

        // -----------------------------------------------------------------
        // MAIN LOOP
        // -----------------------------------------------------------------
        while (opModeIsActive()) {

            // =============================================================
            // DRIVE
            // =============================================================
            // Sticks are cubed for finer control at low speeds.
            // x is boosted 10% to make up for slower strafing.
            double y = Math.pow(-gamepad1.left_stick_y, 3);
            double x = Math.pow(gamepad1.left_stick_x, 3) * 1.1;
            double rx = Math.pow(gamepad1.right_stick_x, 3);

            // Scale all four wheels down together if any would exceed full power
            double clamp = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(rx), 1);
            double flPower = (y + x + rx) / clamp;
            double frPower = (y - x - rx) / clamp;
            double blPower = (y - x + rx) / clamp;
            double brPower = (y + x - rx) / clamp;

            fl.setPower(flPower * mult);
            fr.setPower(frPower * mult);
            bl.setPower(blPower * mult);
            br.setPower(brPower * mult);

            // =============================================================
            // SHOOTER (bang-bang + PIDF)
            // =============================================================

            // ---- Target RPM: D-pad up/down changes it by 50 RPM per press ----
            if (gamepad1.dpad_up && !dpadUpWas) targetRPM += 50;
            if (gamepad1.dpad_down && !dpadDownWas) targetRPM = Math.max(0, targetRPM - 50);
            dpadUpWas = gamepad1.dpad_up;
            dpadDownWas = gamepad1.dpad_down;

            boolean shooting = true; // flywheel always on

            // ---- Timing: ONE timestamp per loop ----
            long now = System.nanoTime();
            double dt = (now - previousTime) / 1e9;
            previousTime = now;

            // ---- Measure RPM and smooth it a bit (encoder velocity is noisy) ----
            double rawRPM = shooter.getVelocity() / TICKS_PER_REV * 60.0;
            filteredRPM = 0.2 * filteredRPM + 0.8 * rawRPM;

            double error = targetRPM - filteredRPM;
            double output = 0;

            // Bang-bang is active only while outside the window around the target
            boolean bangBangActive = shooting && Math.abs(error) > BANG_BANG_WINDOW;

            if (bangBangActive) {
                // ---- Far from target: bang-bang ----
                // Full power if too slow, low power if too fast
                output = (error > 0) ? BANG_HIGH_POWER : BANG_LOW_POWER;
                shooter.setPower(output);
                integral = 0; // keep PIDF memory clean for when we switch back
            } else if (shooting && dt > 0) {
                // ---- Near target: PIDF ----

                // P: reacts to the current error
                double pTerm = kP * error;

                // D: reacts to how fast the RPM is changing. Uses the measurement
                // (not the error) so changing the target doesn't cause a sudden kick.
                double dTerm = -kD * (filteredRPM - previousRPM) / dt;

                // F: baseline power for the target speed
                double fTerm = kF * targetRPM;

                // I: accumulates error over time. Stops accumulating while the
                // output is maxed out (anti-windup) and is capped to MAX_I_POWER.
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

                // Combine all four terms and keep the result in the motor power range
                output = pTerm + iTerm + dTerm + fTerm;
                output = Math.max(0.0, Math.min(1.0, output));
                shooter.setPower(output);
            } else {
                // ---- Not shooting: stop and reset the controller memory ----
                shooter.setPower(0);
                integral = 0;
            }
            previousRPM = filteredRPM; // save for next loop's D term

            // =============================================================
            // READY CHECK: is the flywheel fast enough?
            // =============================================================
            // Uses two thresholds (hysteresis) so the flag doesn't flicker.
            if (!shooting) {
                ready = false;
            } else if (!ready && Math.abs(error) < AT_SPEED_TOLERANCE) {
                ready = true;
            } else if (ready && Math.abs(error) > LOSE_SPEED_TOLERANCE) {
                ready = false;
            }

            // =============================================================
            // GATE + KICKWHEEL (independent of the shooter)
            // =============================================================
            // Gate is always actively commanded: closed normally, open while the
            // right trigger is held.
            boolean feeding = gamepad1.right_trigger > TRIGGER_THRESHOLD;

            // Start the delay timer the moment the trigger is first pressed
            if (feeding && !feedingWas) feedStartTime = System.nanoTime();
            feedingWas = feeding;

            // True only once the gate has had FEED_DELAY_MS to move out of the way
            boolean feedReady = feeding
                    && (System.nanoTime() - feedStartTime) / 1e6 >= FEED_DELAY_MS;

            gate.setPosition(feeding ? GATE_OPEN : GATE_CLOSED);
            kickwheel.setPower(0.5); // constant 50%; use (feedReady ? 1 : 0) to run only while feeding

            // =============================================================
            // INTAKE + TRANSFER
            // =============================================================
            // Intake: always 100%. Transfer: 15% idle, 100% while the gate is open.
            double transferPower = feeding ? FEED_POWER : IDLE_POWER;
            intake.setPower(INTAKE_POWER);
            transfer.setPower(transferPower);

            // =============================================================
            // TELEMETRY (one update per loop)
            // =============================================================
            telemetry.addData("Shooter mode", bangBangActive ? "BANG-BANG" : "PIDF");
            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Actual RPM", filteredRPM);
            telemetry.addData("Error", error);
            telemetry.addData("Motor Power", output);
            telemetry.addData("Ready (at speed)", ready);
            telemetry.addData("Feeding (RT)", feeding);
            telemetry.addData("Gate pos", feeding ? GATE_OPEN : GATE_CLOSED);
            telemetry.addData("Intake power", intake.getPower());
            telemetry.addData("Transfer power", transfer.getPower());
            telemetry.update();
        }
    }
}