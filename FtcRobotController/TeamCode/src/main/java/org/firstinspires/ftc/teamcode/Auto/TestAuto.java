package org.firstinspires.ftc.teamcode.Auto;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.path;


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
    // These points preserve the original curve, split at 59.98% of its length.
    // If the route changes, regenerate both sets of control points together.
    private final Pose outboundControl1 = poseFactory.of(113.545307570582, 51.822059569060, 0);
    private final Pose outboundControl2 = poseFactory.of(91.799557007426, 88.695125971744, 0);
    private final Pose outboundControl3 = poseFactory.of(77.737896477899, 103.402262155795, 0);
    private final Pose split = poseFactory.of(72.374194868174, 80.294791883761, 0);
    private final Pose inboundControl1 = poseFactory.of(70.323264103477, 71.459136002141, 0);
    private final Pose inboundControl2 = poseFactory.of(69.544049331007, 57.094661208711, 0);
    private final Pose inboundControl3 = poseFactory.of(70.093232002681, 36.326511097650, 0);
    private final Pose straightSegment1Target = poseFactory.of(72, 71.5, 0);

    @Override
    public void initialize() {
        super.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();
        // Face the target directly, avoiding the old piecewise heading jump.
        Path outbound = curve(start, outboundControl1, outboundControl2, outboundControl3, split)
                .facingPoint(straightSegment1Target);
        // Turn smoothly from the outbound heading to the final heading.
        Path inbound = curve(split, inboundControl1, inboundControl2, inboundControl3, straight)
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
