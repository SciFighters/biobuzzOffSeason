package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.InstantCommand;
import org.firstinspires.ftc.teamcode.subSystems.IntakeSub;

public class IntakeCmds {
    public static class ExpandIntake extends InstantCommand {
        public ExpandIntake(IntakeSub intake) {
            // uncalibrated value
            super(() -> intake.servoPos(1.0), intake);
        }
    }

    public static class RetractIntake extends InstantCommand {
        public RetractIntake(IntakeSub intake) {
            // uncalibrated value
            super(() -> intake.servoPos(0.0), intake);
        }
    }

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
        public void end(boolean interrupted) {
            intake.setPower(0.0);
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
        public void end(boolean interrupted) {
            intake.setPower(0.0);
        }
    }

    public static class StopIntake extends CommandBase {

        private final IntakeSub intake;

        public StopIntake(IntakeSub intake) {
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
