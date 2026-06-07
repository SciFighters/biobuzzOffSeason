package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subSystems.BoxSub;

public class BoxCmds {

    // Discharge the box (open servos)
    public static class Discharge extends CommandBase {

        private final BoxSub box;

        public Discharge(BoxSub box) {
            this.box = box;
            addRequirements(box);
        }

        @Override
        public void initialize() {
            box.setLeftServoPosition(1.0);
            box.setRightServoPosition(1.0);
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    // close the box (close servos)
    public static class Close extends CommandBase {

        private final BoxSub box;

        public Close(BoxSub box) {
            this.box = box;
            addRequirements(box);
        }

        @Override
        public void initialize() {
            box.setLeftServoPosition(0.0);
            box.setRightServoPosition(0.0);
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }
}
