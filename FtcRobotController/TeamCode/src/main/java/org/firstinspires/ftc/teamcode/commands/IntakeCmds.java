package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subSystems.IntakeSub;

public class IntakeCmds {
    static boolean button;

    public static class Intake extends CommandBase {

        private final IntakeSub intake;

        public Intake(IntakeSub intake) {
            this.intake = intake;
            addRequirements(intake);
        }

        @Override
        public void execute() {
            // uncalibrated value
            intake.setPower(1.0);
        }

        @Override
        public boolean isFinished() {
            return false;
        }
    }

    public static class IntakeEject extends CommandBase {

        private final IntakeSub intake;

        public IntakeEject(IntakeSub intake) {
            this.intake = intake;
            addRequirements(intake);
        }

        @Override
        public void execute() {
            intake.setPower(-1.0);
        }

        @Override
        public boolean isFinished() {
            return false;
        }
    }

    public static class IdleIntake extends CommandBase {

        private final IntakeSub intake;

        public IdleIntake(IntakeSub intake) {
            this.intake = intake;
            addRequirements(intake);
        }

        @Override
        public void initialize() {
            intake.setPower(0.0);
        }

        @Override
        public boolean isFinished() {
            // Command completes immediately
            return true;
        }
    }
}