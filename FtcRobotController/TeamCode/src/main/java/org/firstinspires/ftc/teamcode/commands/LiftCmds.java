package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subSystems.LiftSub;
import java.util.Objects;

public class LiftCmds {


    public static class LiftGoToHeight extends CommandBase {
        private final LiftSub liftSub;
        private final double targetHeightM;

        public LiftGoToHeight(LiftSub liftSub, double targetHeightM) {
            this.liftSub = Objects.requireNonNull(liftSub, "liftSub cannot be null");
            this.targetHeightM = targetHeightM;
            addRequirements(liftSub);
        }

        @Override
        public void initialize() {
            liftSub.resetLiftPID();
        }

        @Override
        public void execute() {
            liftSub.goToHeight(targetHeightM);
        }

        @Override
        public boolean isFinished() {
            return liftSub.atTargetHeight();
        }

        @Override
        public void end(boolean interrupted) {
            if (!interrupted) {
                new LiftHoldPosition(liftSub).schedule();
            }
        }
    }

    public static class LiftHoldPosition extends CommandBase {
        private final LiftSub liftSub;
        private double targetHeightMm;
        double current;

        public LiftHoldPosition(LiftSub liftSub) {
            this.liftSub = Objects.requireNonNull(liftSub, "liftSub cannot be null");
            this.current = liftSub.getHeightAvg();
            addRequirements(liftSub);
        }


        @Override
        public void initialize() {
            liftSub.resetLiftPID();
        }

        @Override
        public void execute() {
            liftSub.setPower(0);
        }

        @Override
        public boolean isFinished() {
            return false; // hold until canceled
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