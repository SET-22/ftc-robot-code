
package org.firstinspires.ftc.teamcode.opmodes;

import static com.pedropathing.api.Paths.*;

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

    private final PoseFactory p = PoseFactory.degrees();

    // Example starting position (inches, degrees)
    private final Pose startPose = p.of(24, 24, 0);

    // Target is 24 inches forward along the X-axis
    private final Pose targetPose = p.of(48, 24, 0);

    private Path driveForward;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);

        follower.setPose(startPose);

        driveForward = line(startPose, targetPose)
                .linear(startPose, targetPose);
    }

    @Override
    public void start() {
        // Start the path exactly once
        follower.follow(driveForward);
    }

    @Override
    public void loop() {
        // Update Pedro Pathing every loop
        follower.update();

        telemetry.addData("Busy", follower.isBusy());
        telemetry.addData("Pose", follower.pose());
        telemetry.update();
    }
}
