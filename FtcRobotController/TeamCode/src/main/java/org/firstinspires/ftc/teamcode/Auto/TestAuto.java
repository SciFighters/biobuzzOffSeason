package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.pedro.Constants;
@Autonomous
public class TestAuto extends CommandOpMode {
    Follower follower;
    PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose start = poseFactory.of(70.4172, 8, 90);
    private final Pose straight = poseFactory.of(70.4981, 8.2706, 90);
    private final Pose straightControl1 = poseFactory.of(130.0363, 68.5784, 0);
    private final Pose straightControl2 = poseFactory.of(65.6845, 115.8776, 0);
    private final Pose straightControl3 = poseFactory.of(69.0344, 109.6998, 0);
    private final Pose straightSegment1Target = poseFactory.of(71.5, 71.5, 0);
    private final Pose straightSegment2Heading = poseFactory.of(70.4981, 8.2706, 0);
    @Override
    public void initialize() {
        super.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        Path path = Paths.curve(start, straightControl1, straightControl2, straightControl3, straight).constant(straight);
//                .heading(Interpolator.piecewise().until(0.5998, Interpolator.facingPoint(straightSegment1Target))
//                        .until(1, Interpolator.constant(straightSegment2Heading)));

        schedule(new FollowPathCommand(follower,path));
    }

    @Override
    public void run() {
        super.run();
        follower.update();
        telemetry.addData("x",follower.pose().x());
        telemetry.addData("y",follower.pose().y());
        telemetry.addData("heading",follower.pose().heading());
        telemetry.addData("status",follower.isBusy());
        telemetry.update();
    }
}
