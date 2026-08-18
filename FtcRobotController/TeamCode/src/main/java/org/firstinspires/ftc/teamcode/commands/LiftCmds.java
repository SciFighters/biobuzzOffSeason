package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subSystems.LiftSub;
import java.util.Objects;

public class LiftCmds {

    private static final double toleranceMm= 1.0;

    public static class GoToHeight extends CommandBase {
        private final LiftSub liftSub;
        private final double targetHeight;

        public GoToHeight(LiftSub liftSub, double targetHeight) {
            this.liftSub = Objects.requireNonNull(liftSub, "liftSub cannot be null");
            this.targetHeight = targetHeight;
            addRequirements(liftSub);
        }

        @Override
        public void initialize() {
            liftSub.resetArmPID();
        }

        @Override
        public void execute() {
            liftSub.powerToTargetHeight(targetHeight);
        }

        @Override
        public boolean isFinished() {
            return liftSub.atTargetHeight();
        }

        @Override
        public void end(boolean interrupted) {
            if (!interrupted) {
                new LiftHoldHeight(liftSub, targetHeight).schedule();
            } else {
                liftSub.setPower(0);
            }
        }
    }

    public static class HoldHeight extends CommandBase {
        private final LiftSub liftSub;
        private double targetHeight;
        private final boolean useProvidedTarget;

        // Create a hold command that maintains the lift's current height
        public LiftHoldHeight(LiftSub liftSub) {
            this.liftSub = Objects.requireNonNull(liftSub, "liftSub cannot be null");
            this.targetHeight = 0.0;
            this.useProvidedTarget = false;
            addRequirements(liftSub);
        }

        //Create a hold command that maintains a specific height (mm)
        public LiftHoldHeight(LiftSub liftSub, double targetHeight) {
            this.liftSub = Objects.requireNonNull(liftSub, "liftSub cannot be null");
            this.targetHeight = targetHeight;
            this.useProvidedTarget = true;
            addRequirements(liftSub);
        }

        @Override
        public void initialize() {
            if (!useProvidedTarget) {
                this.targetHeight = liftSub.getHeightAvg();
            }
            liftSub.resetArmPID();
        }

        @Override
        public void execute() {
            liftSub.powerToTargetHeight(targetHeight);
        }

        @Override
        public boolean isFinished() {
            return false; // hold until cancelled
        }

        @Override
        public void end(boolean interrupted) {
            liftSub.setPower(0);
        }
    }

    public static class LiftPower extends CommandBase {
        private final LiftSub liftSub;
        private final double power;
        private final long finishTime;

        public LiftPower(LiftSub liftSub, double power, long durationMs) {
            this.liftSub = Objects.requireNonNull(liftSub, "liftSub cannot be null");
            this.power = power;
            this.finishTime = System.currentTimeMillis() + durationMs;
            addRequirements(liftSub);
        }

        @Override
        public void initialize() {
            liftSub.setPower(power);
        }

        @Override
        public boolean isFinished() {
            return System.currentTimeMillis() >= finishTime;
        }

        @Override
        public void end(boolean interrupted) {
            liftSub.setPower(0);
        }
    }
}