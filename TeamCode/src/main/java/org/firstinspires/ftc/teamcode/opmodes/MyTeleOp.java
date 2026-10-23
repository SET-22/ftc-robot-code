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
    private DcMotor transfer;   // hold A
    private DcMotor intake;     // hold X
    private Servo gate;         // press B to toggle open/closed
    private DcMotor kickwheel;  // runs while B is held

    private static final double GATE_CLOSED = 0.0; // tune
    private static final double GATE_OPEN = 0.5;   // tune
    private boolean gateOpen = false;
    private boolean bWasPressed = false;

    // ---------- Shooter: holds targetRPM while right bumper is held ----------
    // Self-correcting: the integral term keeps adjusting power until the
    // measured RPM matches the target, so kF doesn't need to be exact.
    private DcMotorEx shooter;
    private double targetRPM = 3100.0;   // D-pad up/down changes it by 100
    private boolean dpadUpWas = false;
    private boolean dpadDownWas = false;

    private static final double TICKS_PER_REV = 28.0;

    private double kF = 1.0 / 7500.0;  // only the starting guess for power
    private double kP = 0.00005;       // quick push proportional to error
    private double kI = 0.0004;        // power added per (RPM of error * second)
    // If RPM oscillates around the target: lower kI (and kP).
    // If it takes too long to reach the target: raise kI.

    private double controllerPower = 0.0;
    private boolean wasShooting = false;
    private double filteredRPM = 0.0;  // smoothed RPM reading
    private static final double AT_SPEED_TOLERANCE = 100.0;

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
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // If Actual RPM reads negative when it spins the right way:
        // shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        gate.setPosition(GATE_OPEN);

        telemetry.addLine("start");
        telemetry.update();

        waitForStart(); // waits for PLAY

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

            // ---------- Mechanisms ----------
            transfer.setPower(gamepad1.a ? 1 : 0);
            intake.setPower(gamepad1.x ? 1 : 0);

            // Gate toggles on each B press and STAYS there
            if (gamepad1.b && !bWasPressed) gateOpen = !gateOpen;
            bWasPressed = gamepad1.b;
            gate.setPosition(gateOpen ? GATE_OPEN : GATE_CLOSED);

            kickwheel.setPower(gamepad1.b ? 1 : 0);

            // ---------- Shooter ----------
            // D-pad up/down changes the target by 100 RPM per press
            if (gamepad1.dpad_up && !dpadUpWas) targetRPM += 50;
            if (gamepad1.dpad_down && !dpadDownWas) targetRPM = Math.max(0, targetRPM - 50);
            dpadUpWas = gamepad1.dpad_up;
            dpadDownWas = gamepad1.dpad_down;

            long now = System.nanoTime();
            double dt = (now - previousTime) / 1e9;
            previousTime = now;

            // Read RPM and smooth it a bit (encoder velocity is noisy)
            double rawRPM = shooter.getVelocity() / TICKS_PER_REV * 60.0;
            filteredRPM = 0.7 * filteredRPM + 0.3 * rawRPM;

            double error = targetRPM - filteredRPM;
            double output = 0;

            if (gamepad1.right_bumper && dt > 0) {
                if (!wasShooting) {
                    controllerPower = kF * targetRPM; // start near the right power
                }
                // Integral: keeps nudging power until the RPM matches the target
                controllerPower += kI * error * dt;
                controllerPower = Math.max(0.0, Math.min(1.0, controllerPower));

                output = controllerPower + kP * error;
                output = Math.max(0.0, Math.min(1.0, output));
                shooter.setPower(output);
            } else {
                shooter.setPower(0);
            }
            wasShooting = gamepad1.right_bumper;

            // ---------- Telemetry (one update per loop) ----------
            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Actual RPM", filteredRPM);
            telemetry.addData("Error", error);
            telemetry.addData("Motor Power", output);
            telemetry.addData("At speed", gamepad1.right_bumper
                    && Math.abs(error) < AT_SPEED_TOLERANCE);
            telemetry.addData("Gate open", gateOpen);
            telemetry.update();
        }
    }
}