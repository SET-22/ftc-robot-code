// package here — must match your folder
package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "My_TeleOp", group = "Practice")
public class MyTeleOp extends LinearOpMode {

    // ---------- Drive ----------
    private DcMotor fr;
    private DcMotor fl;
    private DcMotor br;
    private DcMotor bl;
    private double mult = 0.9;

    // ---------- Mechanisms ----------
    private DcMotor transfer;   // hold A, or runs with left bumper
    private DcMotor intake;     // hold X
    private Servo gate;         // active (closed) from init; hold left bumper to open (with kickwheel + transfer)
    private DcMotor kickwheel;  // runs while left bumper is held

    // GATE_CLOSED = in place (blocking), where it sits from init.
    // GATE_OPEN = moved out of the way while left bumper is held. Swap/tune these to your mechanism.
    private static final double GATE_CLOSED = 0.7; // tune
    private static final double GATE_OPEN = 0.0;   // tune

    // ---------- Shooter: PIDF holds targetRPM while right bumper is held ----------
    private DcMotorEx shooter;
    private double targetRPM = 4100.0;   // D-pad up/down changes it by 50
    private boolean dpadUpWas = false;
    private boolean dpadDownWas = false;

    private static final double TICKS_PER_REV = 28.0;

    // output = kP*error + kI*integral(error) + kD*(rate of change) + kF*target
    //
    // Tune in this order: kF, then kP, then kD, then kI.
    //   kF: baseline power for the target speed. About 1 / (RPM at full power).
    //       From your earlier test, full power is roughly 7500 RPM, so 1/7500.
    //       If it settles ABOVE the target with kP = kI = 0, lower kF. Below, raise it.
    //   kP: raise until it reaches the target fast; lower if it oscillates.
    //   kD: raise (slowly) if it overshoots on spin-up; lower if it gets jittery.
    //   kI: raise only if it settles a little off the target and stays there.
    private double kP = 0.0002;
    private double kI = 0.0002;
    private double kD = 0.00001;
    private double kF = 1.0 / 7500.0;

    // Largest power the I term is allowed to add (anti-windup)
    private static final double MAX_I_POWER = 0.3;

    private double integral = 0.0;
    private double previousRPM = 0.0;
    private double filteredRPM = 0.0;    // smoothed RPM reading

    // Ready when within 100 RPM of the target; stays ready until the error
    // grows past 150 (prevents the kickwheel from flickering).
    private static final double AT_SPEED_TOLERANCE = 100.0;
    private static final double LOSE_SPEED_TOLERANCE = 150.0;
    private boolean ready = false;

    @Override
    public void runOpMode() {
        // ---------- Hardware map ----------
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");

        gate = hardwareMap.get(Servo.class, "gate");
        transfer = hardwareMap.get(DcMotor.class, "transfer");
        intake = hardwareMap.get(DcMotor.class, "intake");
        kickwheel = hardwareMap.get(DcMotor.class, "kickwheel");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        fl.setDirection(DcMotorSimple.Direction.FORWARD);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        transfer.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        kickwheel.setDirection(DcMotorSimple.Direction.REVERSE); // flip if it spins the wrong way

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // our PID sets the power
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // If Actual RPM reads negative when it spins the right way:
        // shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        // Gate is activated (closed) immediately at init, not left at 0.0
        gate.setPosition(GATE_CLOSED);

        telemetry.addLine("start");
        telemetry.update();

        waitForStart(); // waits for PLAY

        // Re-command the gate the moment the match starts
        gate.setPosition(GATE_CLOSED);

        // Set the timer AFTER start so the first dt isn't huge
        long previousTime = System.nanoTime();

        while (opModeIsActive()) {

            // ---------- Drive ----------
            double y = Math.pow(-gamepad1.left_stick_y, 3);
            double x = Math.pow(gamepad1.left_stick_x, 3) * 1.1;
            double rx = Math.pow(gamepad1.right_stick_x, 3);
            double clamp = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(rx), 1);
            double flPower = (y + x + rx) / clamp;
            double frPower = (y - x - rx) / clamp;
            double blPower = (y - x + rx) / clamp;
            double brPower = (y + x - rx) / clamp;

            fl.setPower(flPower * mult);
            fr.setPower(frPower * mult);
            bl.setPower(blPower * mult);
            br.setPower(brPower * mult);

            // ---------- Intake ----------
            intake.setPower(gamepad1.x ? 1 : 0);

            // ---------- Shooter: PIDF ----------
            // D-pad up/down changes the target by 50 RPM per press
            if (gamepad1.dpad_up && !dpadUpWas) targetRPM += 50;
            if (gamepad1.dpad_down && !dpadDownWas) targetRPM = Math.max(0, targetRPM - 50);
            dpadUpWas = gamepad1.dpad_up;
            dpadDownWas = gamepad1.dpad_down;

            boolean shooting = gamepad1.right_bumper;

            // ONE timestamp per loop
            long now = System.nanoTime();
            double dt = (now - previousTime) / 1e9;
            previousTime = now;

            // Measure RPM and smooth it a bit (encoder velocity is noisy)
            double rawRPM = shooter.getVelocity() / TICKS_PER_REV * 60.0;
            filteredRPM = 0.7 * filteredRPM + 0.3 * rawRPM;

            double error = targetRPM - filteredRPM;
            double output = 0;

            if (shooting && dt > 0) {
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

                output = pTerm + iTerm + dTerm + fTerm;
                output = Math.max(0.0, Math.min(1.0, output)); // motor power range
                shooter.setPower(output);
            } else {
                // Not shooting: stop and reset the controller memory
                shooter.setPower(0);
                integral = 0;
            }
            previousRPM = filteredRPM;

            // ---------- Ready check: is the flywheel fast enough? ----------
            if (!shooting) {
                ready = false;
            } else if (!ready && Math.abs(error) < AT_SPEED_TOLERANCE) {
                ready = true;
            } else if (ready && Math.abs(error) > LOSE_SPEED_TOLERANCE) {
                ready = false;
            }

            // ---------- Gate + kickwheel (independent of the shooter) ----------
            // Gate is always actively commanded: closed normally, open while left bumper is held.
            // Left bumper fires everything at once: gate + kickwheel + transfer.
            boolean feeding = gamepad1.left_bumper;
            gate.setPosition(feeding ? GATE_OPEN : GATE_CLOSED);
            kickwheel.setPower(feeding ? 1 : 0);
            // Transfer also runs on A by itself (e.g. to adjust balls); LB runs it with the shot.
            transfer.setPower((feeding || gamepad1.a) ? 1 : 0);

            // ---------- Telemetry (one update per loop) ----------
            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Actual RPM", filteredRPM);
            telemetry.addData("Error", error);
            telemetry.addData("Motor Power", output);
            telemetry.addData("Ready (at speed)", ready);
            telemetry.addData("Shooting (RB)", shooting);
            telemetry.addData("Feeding (LB)", feeding);
            telemetry.addData("Gate pos", feeding ? GATE_OPEN : GATE_CLOSED);
            telemetry.update();
        }
    }
}