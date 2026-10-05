package org.firstinspires.ftc.teamcode.subSystems;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

public class AutoShooter {
    final PoseFactory poseFactory = PoseFactory.degrees();
    private final Calibration[] calibrations = {calibration(1,2,3)};//arbitrary
    private final Follower follower;
    private final LimelightSubsystem limelightSubsystem;

    public AutoShooter(Follower follower, LimelightSubsystem limelightSubsystem){
        this.follower = follower;
        this.limelightSubsystem = limelightSubsystem;
    }

    public Calibration getDesiredState(){
        LimelightSubsystem.Cell cell = limelightSubsystem.getCurrentCell();
        //todo: in the first test change to a constant cell
        double distance = follower.pose().distance(cell.getPose());
        Calibration min = calibration(-10,0,0);
        Calibration max = calibration(200, 0, 0);
        for (Calibration calibration : calibrations) {
            if (calibration.distance < distance & calibration.distance > min.distance) {
                min = calibration;
            } else if (calibration.distance < max.distance) {
                max = calibration;
            }
        }

        double lowerRatio = distance - min.distance;
        double higherRatio = max.distance - distance;
        double ratio = 1 / (max.distance - min.distance);

        return calibration(0,(lowerRatio * max.angle + higherRatio * min.angle) * ratio,
                (lowerRatio * max.RPM + higherRatio * min.RPM) * ratio);


    }
    private double getTurretAngle(){
        LimelightSubsystem.Cell cell = limelightSubsystem.lastCell;
        Pose pose = follower.localizer.pose();
        double heading = pose.heading();
        return (Math.atan2(cell.getPose().y() - pose.y(),cell.getPose().x() - pose.x()) - heading)%360;
    }


    private static Calibration calibration(double distance, double angle,double RPM){
        return new Calibration(distance, angle, RPM);
    }
    //for better readability and debugging
    public static class Calibration{
        public final double distance, angle, RPM;
        public Calibration(double distance, double angle,double RPM){
            this.distance = distance;
            this.angle = angle;
            this.RPM = RPM;
        }
    }
}
