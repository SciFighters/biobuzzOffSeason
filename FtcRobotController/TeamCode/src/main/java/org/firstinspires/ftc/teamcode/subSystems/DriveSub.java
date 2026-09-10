package org.firstinspires.ftc.teamcode.subSystems;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.pedro.Constants;

public class DriveSub extends SubsystemBase {
    private final Follower follower;

    public DriveSub(HardwareMap hardwareMap) {
        follower = Constants.create(hardwareMap);
    }

    @Override
    public void periodic() {
        follower.update();
    }

    public void setStartingPose(Pose pose) {
        follower.setPose(pose);
    }

    public void startTeleopDrive() {
        follower.manual();
    }

    public void setTeleopDrive(double forward, double strafe, double turn) {
        DrivePowers powers = ManualDrive.fieldCentric(
                forward,
                strafe,
                turn,
                follower.pose().heading()
        );
        follower.manual(powers);
        follower.update();
    }

    public void followPath(Path path) {
        follower.follow(path);
    }

    public boolean isBusy() {
        return follower.isBusy();
    }

    public void stop() {

        follower.manual(0, 0, 0);
    }

    public Follower getFollower() {
        return follower;
    }
}
