package org.firstinspires.ftc.teamcode.opmodes;
// package here — must match your folder
// imports here
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


// this line is what makes it show up on the Driver Station
@TeleOp(name = "nurse", group = "Practice")
public class nurse extends LinearOpMode {
    private DcMotor fl;
    private DcMotor bl;
    private DcMotor fr;
    private DcMotor br;
    private DcMotor intake;

    double flPower, frPower, blPower, brPower, intakePower;

    @Override
    public void runOpMode() {

        fl = hardwareMap.get(DcMotor.class, "fl");  // the name must match the Robot Configuration exactly
        bl = hardwareMap.get(DcMotor.class, "bl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        br = hardwareMap.get(DcMotor.class, "br");
        intake = hardwareMap.get(DcMotor.class, "intake");

        fl.setDirection(DcMotorSimple.Direction.FORWARD);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);  // same as teleop

        telemetry.addLine("ready");
        telemetry.update();  // telemetry is a field you inherit; send data to the Driver Station

        waitForStart();   // waits for PLAY

        while (opModeIsActive()) {

            // Drive motor tests: X = fl, Y = fr, dpad left = bl, dpad right = br
            flPower = gamepad1.x ? 0.2 : 0;
            frPower = gamepad1.y ? 0.2 : 0;
            blPower = gamepad1.dpad_left ? 0.2 : 0;
            brPower = gamepad1.dpad_right ? 0.2 : 0;

            // Intake (same as teleop): A = intake, B = reverse
            if (gamepad1.a) {
                intakePower = 1.0;
            } else if (gamepad1.b) {
                intakePower = -1.0;
            } else {
                intakePower = 0;
            }

            fr.setPower(frPower);
            fl.setPower(flPower);
            bl.setPower(blPower);
            br.setPower(brPower);
            intake.setPower(intakePower);

            telemetry.addData("flPower", flPower);
            telemetry.addData("frPower", frPower);
            telemetry.addData("blPower", blPower);
            telemetry.addData("brPower", brPower);
            telemetry.addData("intakePower", intakePower);
            telemetry.update();
        }
    }
}