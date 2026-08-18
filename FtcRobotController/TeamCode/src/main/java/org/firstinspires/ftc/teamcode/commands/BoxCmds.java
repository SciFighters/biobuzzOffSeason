package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subSystems.BoxSub;

public class BoxCmds {
    static boolean isBoxOpen;

    public static class Discharge extends CommandBase {

        private final BoxSub box;

        public Discharge(BoxSub box) {
            this.box = box;
            addRequirements(box);
        }

        @Override
        public void initialize() {
            // Open both servos fully
            // uncalibrated value
            box.setServosPosition(1.0);
        }

        @Override
        public boolean isFinished() {
            return true;
        }

        @Override
        public void end(boolean interrupted) {
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
            box.setServosPosition(0.0);

        }

        @Override
        public boolean isFinished() {
            return true;
        }

        @Override
        public void end(boolean interrupted) {
            isBoxOpen = false; // for control

        }
    }
}