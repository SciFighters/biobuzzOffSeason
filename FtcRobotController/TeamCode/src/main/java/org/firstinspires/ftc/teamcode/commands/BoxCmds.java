package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subSystems.BoxSub;

/**
 * Collection of commands that control the box servo mechanism.
 * 
 * <p>Each inner class represents a specific command that can be scheduled 
 * to open or close the game piece holding box.
 */
public class BoxCmds {

    /**
     * Command to open the box and discharge the game piece.
     * This sets both left and right servos to position 1.0 (fully open).
     */
    public static class Discharge extends CommandBase {

        private final BoxSub box;

        public Discharge(BoxSub box) {
            this.box = box;
            addRequirements(box);
        }

        @Override
        public void initialize() {
            // Open both servos fully
            box.setLeftServoPosition(1.0);
            box.setRightServoPosition(1.0);
        }

        @Override
        public boolean isFinished() {
            // Command completes immediately
            return true;
        }
    }

    /**
     * Command to close the box and hold the game piece.
     * This sets both left and right servos to position 0.0 (fully closed).
     */
    public static class Close extends CommandBase {

        private final BoxSub box;

        public Close(BoxSub box) {
            this.box = box;
            addRequirements(box);
        }

        @Override
        public void initialize() {
            // Close both servos fully
            box.setLeftServoPosition(0.0);
            box.setRightServoPosition(0.0);
        }

        @Override
        public boolean isFinished() {
            // Command completes immediately
            return true;
        }
    }
}