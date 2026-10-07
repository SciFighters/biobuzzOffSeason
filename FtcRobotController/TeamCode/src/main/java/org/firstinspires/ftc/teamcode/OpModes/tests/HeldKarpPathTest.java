package org.firstinspires.ftc.teamcode.OpModes.tests;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.Utilities.ActionOpMode;
import org.firstinspires.ftc.teamcode.Utilities.HeldKarpPath;
import org.firstinspires.ftc.teamcode.subSystems.DriveSubsystem;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

@Autonomous(name = "HeldKarpPathTester", group = "Test")
public class HeldKarpPathTest extends ActionOpMode {
    Follower follower;
    private DriveSubsystem drive;
    PoseFactory poseFactory = PoseFactory.degrees();
    private FollowPathCommand pathCommand;
    private boolean hasStarted;

    private final Pose start = poseFactory.of(96, 24, 90);
    private final List<Pose> points = Arrays.asList(
            poseFactory.of(108, 36, 0),
            poseFactory.of(84, 48, 0),
            poseFactory.of(120, 60, 0),
            poseFactory.of(96, 84, 0));

    @Override
    public void initialize() {
        super.reset();
        hasStarted = false;
        drive = new DriveSubsystem(hardwareMap);
        follower = drive.getFollower();
        drive.setStartingPose(start);
        drive.startTeleopDrive();
        drive.periodic();

        List<Pose> targets = new ArrayList<>(points);
        targets.sort(Comparator.comparingDouble(point ->
                Math.hypot(point.x() - start.x(), point.y() - start.y())));
        Path route = HeldKarpPath.plan(start, targets);
        pathCommand = new FollowPathCommand(follower, route, true, 0.4);
        pathCommand.addRequirements(drive);
    }

    @Override
    public void initialize_loop() {
        drive.periodic();
        telemeter();
    }

    @Override
    public void preRun() {
        hasStarted = true;
        schedule(pathCommand);
    }

    @Override
    public void run() {
        super.run();
        telemeter();
    }

    @Override
    public void end() {
        super.end();
        if (pathCommand != null) pathCommand.cancel();
        if (drive != null) {
            drive.stop();
            drive.periodic();
        }
    }

    public void telemeter() {
        Pose current = follower.pose();
        String status = !hasStarted ? "Ready - press START"
                : pathCommand.isScheduled() ? "Following" : "Complete - holding finish";

        multipleTelemetry.addData("Status", status);
        multipleTelemetry.addData("Path segment", hasStarted
                ? (follower.pathIndex() + 1) + " / " + points.size() : "Not started");
        multipleTelemetry.addData("Pose", "X %.1f in | Y %.1f in | Heading %.1f deg",
                current.x(), current.y(), Math.toDegrees(current.heading()));
        multipleTelemetry.update();
    }
}
