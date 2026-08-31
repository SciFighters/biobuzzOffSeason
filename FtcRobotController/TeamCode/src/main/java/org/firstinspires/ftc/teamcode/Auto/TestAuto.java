package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.commands.DriveCmds;
import org.firstinspires.ftc.teamcode.subSystems.DriveSub;

/**
 * Test autonomous using Pedro's built-in mecanum drivetrain and Pinpoint localizer.
 */
@Autonomous(name = "TestAuto PedroPath", group = "Auto")
public class TestAuto extends CommandOpMode {

    private DriveSub driveSub;

    @Override
    public void initialize() {
        driveSub = new DriveSub(hardwareMap);

        Pose start = new Pose();
        Pose end = new Pose(24, 0, 0);
        driveSub.setStartingPose(start);
        PathChain forward = driveSub.getFollower().pathBuilder()
                .addPath(new BezierLine(start, end))
                .setConstantHeadingInterpolation(start.getHeading())
                .build();
        schedule(new DriveCmds.FollowPath(driveSub, forward));
    }
}
