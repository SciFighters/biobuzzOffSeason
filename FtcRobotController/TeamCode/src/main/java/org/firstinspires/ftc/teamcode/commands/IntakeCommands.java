package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.InstantCommand;
import org.firstinspires.ftc.teamcode.subSystems.IntakeSubsystem;

public class IntakeCommands {
    public static class ExpandIntake extends InstantCommand {
        public ExpandIntake(IntakeSubsystem intake) {
            // uncalibrated value
            super(() -> intake.servoPos(1.0), intake);
        }
    }

    public static class RetractIntake extends InstantCommand {
        public RetractIntake(IntakeSubsystem intake) {
            // uncalibrated value
            super(() -> intake.servoPos(0.0), intake);
        }
    }

    public static class StartIntake extends CommandBase {

        private final IntakeSubsystem intake;

        public StartIntake(IntakeSubsystem intake) {
            this.intake = intake;
            addRequirements(intake);
        }

        @Override
        public void initialize() {
            // uncalibrated value
            intake.setPower(1.0);
        }

        @Override
        public void end(boolean interrupted) {
            intake.setPower(0.0);
        }
    }

    public static class StartOuttake extends CommandBase {

        private final IntakeSubsystem intake;

        public StartOuttake(IntakeSubsystem intake) {
            this.intake = intake;
            addRequirements(intake);
        }

        @Override
        public void initialize() {
            intake.setPower(-1.0);
        }

        @Override
        public void end(boolean interrupted) {
            intake.setPower(0.0);
        }
    }

    public static class StopIntake extends CommandBase {

        private final IntakeSubsystem intake;

        public StopIntake(IntakeSubsystem intake) {
            this.intake = intake;
            addRequirements(intake);
        }

        @Override
        public void initialize() {
            intake.setPower(0.0);
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }
}
