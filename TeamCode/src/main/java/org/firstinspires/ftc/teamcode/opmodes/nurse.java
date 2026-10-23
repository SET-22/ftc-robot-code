package org.firstinspires.ftc.teamcode.opmodes;
// package here — must match your folder

// imports here
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;

/**
 * Diagnostic OpMode: every part runs on its own button so you can find
 * wiring, direction, encoder, and config problems one piece at a time.
 *
 * CONTROLS (gamepad1)
 *   Drive motors (low power):  X = fl   Y = fr   A = bl   B = br
 *   D-pad up = intake   D-pad down = transfer   D-pad left = kickwheel
 *       (hold LEFT BUMPER with any of these to run it in reverse)
 *   D-pad right = shooter at test power
 *       Right bumper = test power +0.05   Right trigger = test power -0.05
 *   Left stick press = gate OPEN   Right stick press = gate CLOSED
 *
 * A device missing from the Robot Configuration does NOT crash this OpMode.
 * It is listed as MISSING on telemetry instead.
 */
@TeleOp(name = "nurse", group = "Practice")
public class nurse extends LinearOpMode {

    // ---------- Devices ----------
    private DcMotor fl;
    private DcMotor bl;
    private DcMotor fr;
    private DcMotor br;
    private DcMotor intake;
    private DcMotor transfer;
    private DcMotor kickwheel;
    private DcMotorEx shooter;
    private Servo gate;

    // ---------- Test settings ----------
    private static final double DRIVE_TEST_POWER = 0.2;
    private static final double GATE_CLOSED = 0.0; // same as teleop, tune
    private static final double GATE_OPEN = 0.5;   // same as teleop, tune
    private static final double TICKS_PER_REV = 28.0;

    private double shooterPower = 0.5;
    private boolean rbWas = false;
    private boolean rtWas = false;
    private double gatePos = GATE_CLOSED;

    private final ArrayList<String> missing = new ArrayList<>();

    @Override
    public void runOpMode() {

        // The name must match the Robot Configuration exactly
        fl = getDevice(DcMotor.class, "fl");
        bl = getDevice(DcMotor.class, "bl");
        fr = getDevice(DcMotor.class, "fr");
        br = getDevice(DcMotor.class, "br");
        intake = getDevice(DcMotor.class, "intake");
        transfer = getDevice(DcMotor.class, "transfer");
        kickwheel = getDevice(DcMotor.class, "kickwheel");
        shooter = getDevice(DcMotorEx.class, "shooter");
        gate = getDevice(Servo.class, "gate");

        // Directions copied from MyTeleOp so the test matches real driving
        setDir(fl, DcMotorSimple.Direction.FORWARD);
        setDir(bl, DcMotorSimple.Direction.FORWARD);
        setDir(fr, DcMotorSimple.Direction.REVERSE);
        setDir(br, DcMotorSimple.Direction.REVERSE);
        setDir(intake, DcMotorSimple.Direction.REVERSE);
        setDir(transfer, DcMotorSimple.Direction.REVERSE);
        setDir(kickwheel, DcMotorSimple.Direction.REVERSE);
        // shooter keeps its default direction, same as MyTeleOp

        if (shooter != null) {
            shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        if (gate != null) gate.setPosition(GATE_CLOSED);

        telemetry.addLine("ready");
        if (missing.isEmpty()) {
            telemetry.addLine("All devices found");
        } else {
            telemetry.addData("MISSING from config", missing.toString());
        }
        telemetry.update();

        waitForStart();   // waits for PLAY

        while (opModeIsActive()) {

            // ---------- Drive motor tests ----------
            double flPower = gamepad1.x ? DRIVE_TEST_POWER : 0;
            double frPower = gamepad1.y ? DRIVE_TEST_POWER : 0;
            double blPower = gamepad1.a ? DRIVE_TEST_POWER : 0;
            double brPower = gamepad1.b ? DRIVE_TEST_POWER : 0;

            // ---------- Mechanisms (D-pad; hold LEFT BUMPER to reverse) ----------
            double dir = gamepad1.left_bumper ? -1.0 : 1.0;
            double intakePower = gamepad1.dpad_up ? dir : 0;
            double transferPower = gamepad1.dpad_down ? dir : 0;
            double kickPower = gamepad1.dpad_left ? dir : 0;

            // ---------- Shooter (D-pad right) ----------
            // Right bumper = test power +0.05, right trigger = test power -0.05
            boolean rtPressed = gamepad1.right_trigger > 0.5;
            if (gamepad1.right_bumper && !rbWas) shooterPower = Math.min(1.0, shooterPower + 0.05);
            if (rtPressed && !rtWas) shooterPower = Math.max(0.0, shooterPower - 0.05);
            rbWas = gamepad1.right_bumper;
            rtWas = rtPressed;
            double shooterOut = gamepad1.dpad_right ? shooterPower : 0;

            // ---------- Gate ----------
            if (gamepad1.left_stick_button) gatePos = GATE_OPEN;
            else if (gamepad1.right_stick_button) gatePos = GATE_CLOSED;

            // ---------- Apply ----------
            setPower(fl, flPower);
            setPower(fr, frPower);
            setPower(bl, blPower);
            setPower(br, brPower);
            setPower(intake, intakePower);
            setPower(transfer, transferPower);
            setPower(kickwheel, kickPower);
            setPower(shooter, shooterOut);
            if (gate != null) gate.setPosition(gatePos);

            // ---------- Telemetry ----------
            if (!missing.isEmpty()) {
                telemetry.addData("MISSING from config", missing.toString());
            }
            telemetry.addData("flPower / enc", flPower + " / " + encoder(fl));
            telemetry.addData("frPower / enc", frPower + " / " + encoder(fr));
            telemetry.addData("blPower / enc", blPower + " / " + encoder(bl));
            telemetry.addData("brPower / enc", brPower + " / " + encoder(br));
            telemetry.addData("intakePower", intakePower);
            telemetry.addData("transferPower", transferPower);
            telemetry.addData("kickwheelPower", kickPower);
            telemetry.addData("shooter test power", shooterPower);
            telemetry.addData("shooter output", shooterOut);
            if (shooter != null) {
                double ticksPerSec = shooter.getVelocity();
                telemetry.addData("shooter ticks", shooter.getCurrentPosition());
                telemetry.addData("shooter ticks/sec", ticksPerSec);
                telemetry.addData("shooter RPM", ticksPerSec / TICKS_PER_REV * 60.0);
            }
            telemetry.addData("gate position", gate == null ? "MISSING" : String.valueOf(gatePos));
            telemetry.update();
        }
    }

    // ---------- Helpers: null-safe so a missing device can't crash the test ----------
    private <T> T getDevice(Class<? extends T> type, String name) {
        T device = hardwareMap.tryGet(type, name);
        if (device == null) missing.add(name);
        return device;
    }

    private void setDir(DcMotor motor, DcMotorSimple.Direction dir) {
        if (motor != null) motor.setDirection(dir);
    }

    private void setPower(DcMotor motor, double power) {
        if (motor != null) motor.setPower(power);
    }

    private String encoder(DcMotor motor) {
        return motor == null ? "MISSING" : String.valueOf(motor.getCurrentPosition());
    }
}