package org.firstinspires.ftc.teamcode.subSystems;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class LimelightSubsystem {
    Limelight3A limelight;

    //FIRST is the one that is up in the starting configuration.
    public enum Cell {
        FIRST(PoseFactory.degrees().of(58,53.75,0)),
        SECOND(PoseFactory.degrees().of(58,87.75,0));
        private final Pose pose;
        private Cell(Pose pose) {
            this.pose = pose;
        }
        public Pose getPose() {
            return this.pose;
        }
    }
    double bottomHeight = 36.6, topHeight = 53.6;
    Cell lastCell = Cell.FIRST;
    Set<Integer> firstRed = Set.of(34,35,36,37), secondRed = Set.of(30,31,32,33);
    Set<Integer>  firstBlue = Set.of(42,43,44,45), secondBlue = Set.of(38,39,40,41);
    Pose startConfig = PoseFactory.degrees().of(0,12,45);//arbitrary
    public LimelightSubsystem(HardwareMap hm){
        limelight = hm.get(Limelight3A.class,"limelight");
        switch (GlobalVariables.teamColor){
            case RED:
                setPipeline(1);
            case BLUE:
                setPipeline(2);
        }
        setPipeline((GlobalVariables.teamColor == GlobalVariables.TeamColor.RED) ? 1:2);
    }
    public Cell getCurrentCell(){//todo: check  if works
        List<LLResultTypes.FiducialResult> results = limelight.getLatestResult().getFiducialResults();
        for (LLResultTypes.FiducialResult fiducialResult : results) {
            switch (GlobalVariables.teamColor){
                case BLUE:
                    lastCell =
                            ((firstBlue.contains(fiducialResult.getFiducialId()) &&
                                    fiducialResult.getTargetPoseRobotSpace().getPosition().z < bottomHeight - startConfig.y()))
                                    || (secondBlue.contains(fiducialResult.getFiducialId()) &&
                                    fiducialResult.getTargetPoseRobotSpace().getPosition().z > topHeight - startConfig.y())?
                                    Cell.SECOND : Cell.FIRST;
                case RED:
                    lastCell =
                            ((firstRed.contains(fiducialResult.getFiducialId()) &&
                                    fiducialResult.getTargetPoseRobotSpace().getPosition().z < bottomHeight - startConfig.y()))
                                    || (secondRed.contains(fiducialResult.getFiducialId()) &&
                                    fiducialResult.getTargetPoseRobotSpace().getPosition().z > topHeight - startConfig.y())?
                                    Cell.SECOND : Cell.FIRST;
            }

        }
        return lastCell;
    }
    public void setPipeline(int pipeline){
        limelight.pipelineSwitch(pipeline);
    }
    public void setCell(Cell cell){
        lastCell = cell;
    }
    public void start(){
        limelight.start();
    }
    public void stop(){
        limelight.stop();
    }


}
