package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subSystems.LiftSub;

// Lift commands following reference patterns
public class LiftCmds {

    // Command to set lift to a specific height
    public static class SetHeight extends CommandBase {
        private final LiftSub lift;
        private final double height;

        public SetHeight(LiftSub lift, double height) {
            this.lift = lift;
            this.height = height;
            addRequirements(lift);
        }

        @Override
        public void initialize() {
            lift.setPosition((int) height);
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    // Command to home the lift (position 0)
    public static class Home extends CommandBase {
        private final LiftSub lift;

        public Home(LiftSub lift) {
            this.lift = lift;
            addRequirements(lift);
        }

        @Override
        public void initialize() {
            lift.setPosition(0);
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    // Command to set lift to manual power
    public static class SetManualPower extends CommandBase {
        private final LiftSub lift;
        private final double power;

        public SetManualPower(LiftSub lift, double power) {
            this.lift = lift;
            this.power = power;
            addRequirements(lift);
        }

        @Override
        public void initialize() {
            lift.setPower(power);
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }
}