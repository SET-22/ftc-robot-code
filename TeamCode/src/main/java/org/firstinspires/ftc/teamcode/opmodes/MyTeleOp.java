package org.firstinspires.ftc.teamcode.opmodes;
// package here — must match your folder
// imports here
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


        // this line is what makes it show up on the Driver Station
        @TeleOp(name = "My TeleOp", group = "Practice")
        public class MyTeleOp extends LinearOpMode {
            private DcMotor fl;
            private DcMotor bl;
            private DcMotor fr;
            private DcMotor br;
            private final double mult = 0.9;
            private DcMotor intake;

            @Override
            public void runOpMode() {


                fl = hardwareMap.get(DcMotor.class, "fl");  // the name must match the Robot Configuration exactly
                bl = hardwareMap.get(DcMotor.class, "bl");
                fr = hardwareMap.get(DcMotor.class, "fr");
                br = hardwareMap.get(DcMotor.class, "br");
                intake = hardwareMap.get(DcMotor.class, "intake");

                fl.setDirection(DcMotorSimple.Direction.REVERSE);  // flip so positive power drives forward
                bl.setDirection(DcMotorSimple.Direction.REVERSE);
                fr.setDirection(DcMotorSimple.Direction.FORWARD);
                br.setDirection(DcMotorSimple.Direction.FORWARD);
                intake.setDirection(DcMotorSimple.Direction.FORWARD); // flip to REVERSE if it spins the wrong way


                telemetry.addLine("ready");
                telemetry.update();  // telemetry is a field you inherit; send data to the Driver Station

                waitForStart();   // waits for PLAY

                while (opModeIsActive()) {

                    double x = Math.pow(-gamepad1.left_stick_x, 3) * 1.1;
                    double y = Math.pow(gamepad1.left_stick_y, 3);
                    double rx = Math.pow(-gamepad1.right_stick_x, 3);
                    double clamp = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(rx), 1);
                    double flPower = (y + x + rx) / clamp;
                    double blPower = (y - x + rx) / clamp;
                    double frPower = (y - x - rx) / clamp;
                    double brPower = (y + x - rx) / clamp;

                    fr.setPower(frPower * mult);
                    br.setPower(brPower * mult);
                    fl.setPower(flPower * mult);
                    bl.setPower(blPower * mult);

                    if (gamepad1.a) {
                        intake.setPower(1.0);      // intake
                    } else if (gamepad1.b) {
                        intake.setPower(-1.0);     // reverse
                    } else {
                        intake.setPower(0);        // stop when neither is held
                    }
                    telemetry.update();
                }
            }
        }