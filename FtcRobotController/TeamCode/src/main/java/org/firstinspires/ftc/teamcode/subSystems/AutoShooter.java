package org.firstinspires.ftc.teamcode.subSystems;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

public class AutoShooter {
    PoseFactory poseFactory;
    final Pose firstCell, secondCell;
    private final Calibration[] calibrations = {calibration(1,2,3)};//arbitrary
    private final Follower follower;

    public AutoShooter(Follower follower){
        poseFactory = (GlobalVariables.teamColor == GlobalVariables.TeamColor.RED) ?
                PoseFactory.degrees() :
                PoseFactory.degrees().rotateAround(new Pose(70.75,70.75),180);
        firstCell = poseFactory.of(58,53.75,0);
        secondCell = poseFactory.of(58,87.75,0);
        this.follower = follower;
    }

    public Calibration getWantedState(){
        //todo: change later to limelight detection for which cell it is
        double distance = follower.pose().distance(firstCell);
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
