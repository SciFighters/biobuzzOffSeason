package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subSystems.BoxSub;

public class BoxCmds {
    
    public static class Discharge extends CommandBase {

        private final BoxSub box;
        boolean isBoxOpen;

        public Discharge(BoxSub box) {
            this.box = box;
            addRequirements(box);
        }

        @Override
        public void initialize() {
            // Open both servos fully
            // uncalibrated value
            box.setLeftServoPosition(1.0);
            box.setRightServoPosition(1.0);
        }

        @Override
        public boolean isFinished() {
            return true;
            isBoxOpen = true; // for control
        }
    }

    public static class Close extends CommandBase {

        private final BoxSub box;

        public Close(BoxSub box) {
            this.box = box;
            addRequirements(box);
        }

        @Override
        public void initialize() {
            // Close both servos fully
            // uncalibrated value
            box.setLeftServoPosition(0.0);
            box.setRightServoPosition(0.0);
        }

        @Override
        public boolean isFinished() {
            // Command completes immediately
            return true;
            isBoxOpen = false; // for control
        }
    }
}