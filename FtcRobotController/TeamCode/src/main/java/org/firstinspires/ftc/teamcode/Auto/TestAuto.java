package org.firstinspires.ftc.teamcode.Auto;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.path;
import static org.firstinspires.ftc.teamcode.Utilities.Utils.splitBezierControlPoints;


import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.pedro.Constants;
@Autonomous
public class TestAuto extends CommandOpMode {
    Follower follower;
    PoseFactory poseFactory = PoseFactory.degrees();

    // The robot starts facing up the field (+Y).
    private final Pose start = poseFactory.of(70.4172, 8, 90);
    private final Pose straight = poseFactory.of(70.4981, 8.2706, -90);
    private final Pose straightControl1 = poseFactory.of(130.0363, 68.5784, 0);
    private final Pose straightControl2 = poseFactory.of(65.6845, 115.8776, 0);
    private final Pose straightControl3 = poseFactory.of(69.0344, 109.6998, 0);
    // Bezier parameter, originally chosen to split at 59.98% of this curve's length.
    private final double splitProgress = 0.7233941399749743;
    private final Pose[][] segments = splitBezierControlPoints(
            new Pose[]{start, straightControl1, straightControl2, straightControl3, straight}, splitProgress);
    private final Pose straightSegment1Target = poseFactory.of(72, 71.5, 0);

    @Override
    public void initialize() {
        super.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();
        // Face the target directly, avoiding the old piecewise heading jump.
        Path outbound = curve(segments[0])
                .facingPoint(straightSegment1Target);
        // Turn smoothly from the outbound heading to the final heading.
        Path inbound = curve(segments[1])
                .linear(outbound.endPose().heading(), straight.heading());
        // Follow both parts in order so the nearby finish cannot skip the loop.
        Path path = path(outbound, inbound);
        schedule(new FollowPathCommand(follower, path));
    }

    @Override
    public void run() {
        super.run();
        follower.update();
        telemetry.addData("x",follower.pose().x());
        telemetry.addData("y",follower.pose().y());
        telemetry.addData("heading",follower.pose().heading());
        telemetry.addData("status",follower.isBusy());
        // Show which part of the route is active during testing.
        telemetry.addData("path segment", follower.pathIndex());
        telemetry.update();
    }
}
