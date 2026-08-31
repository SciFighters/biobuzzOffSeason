package org.firstinspires.ftc.teamcode.subSystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class DriveSub extends SubsystemBase {
    private final Follower follower;

    public DriveSub(HardwareMap hardwareMap) {
        follower = Constants.createFollower(hardwareMap);
    }

    @Override
    public void periodic() {
        follower.update();
    }

    public void setStartingPose(Pose pose) {
        follower.setStartingPose(pose);
    }

    public void startTeleopDrive() {
        follower.startTeleOpDrive();
    }

    public void setTeleopDrive(double forward, double strafe, double turn, boolean robotCentric) {
        follower.setTeleOpDrive(forward, strafe, turn, robotCentric);
    }

    public void followPath(PathChain path) {
        follower.followPath(path);
    }

    public boolean isBusy() {
        return follower.isBusy();
    }

    public void stop() {
        follower.startTeleOpDrive(true);
        follower.setTeleOpDrive(0, 0, 0, true);
    }

    public Follower getFollower() {
        return follower;
    }
}
