
/*
package org.firstinspires.ftc.teamcode.opmodes;

import static com.pedropathing.api.Paths.*;   // lets us call line() directly

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "ExampleAuto")
public class ExampleAuto extends OpMode {

    private Follower follower;
    private int state = 0;

    // PoseFactory: p.of(x, y, heading in DEGREES), in Pedro coordinates.
    // PLACEHOLDERS: set startPose to where the robot is actually placed, and replace the targets.
    private final PoseFactory p = PoseFactory.degrees();

    private final Pose startPose = p.of(24, 24, 0);
    private final Pose scorePose = p.of(48, 24, 0);

    private Path driveToScore;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        driveToScore = line(startPose, scorePose).linear(startPose, scorePose);
    }

    @Override
    public void start() {
        state = 0;
    }

    @Override
    public void loop() {
        follower.update();   // must run every loop

        switch (state) {
            case 0:
                follower.follow(driveToScore);
                state = 1;
                break;

            case 1:
                // Wait for the path to finish, then hold position
                if (!follower.isBusy()) {
                    follower.hold(scorePose);
                    state = 2;
                }
                break;

            case 2:
                // Add your mechanism actions here (shooter, gate, kickwheel, transfer)
                break;
        }

        telemetry.addData("State", state);
        telemetry.addData("Pose", follower.pose());
        telemetry.update();
    }
}
*?
 */