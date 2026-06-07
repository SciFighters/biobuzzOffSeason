package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subSystems.IntakeSub;

// Intake commands following reference patterns
public class IntakeCmds {

    // Command to set intake to forward
    public static class IntakeForward extends CommandBase {
        private final IntakeSub intake;

        public IntakeForward(IntakeSub intake) {
            this.intake = intake;
            addRequirements(intake);
        }

        @Override
        public void initialize() {
            intake.setPower(1.0);
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    // Command to set intake to backward
    public static class IntakeBackward extends CommandBase {
        private final IntakeSub intake;

        public IntakeBackward(IntakeSub intake) {
            this.intake = intake;
            addRequirements(intake);
        }

        @Override
        public void initialize() {
            intake.setPower(-1.0);
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    // Command to set intake to off
    public static class IntakeOff extends CommandBase {
        private final IntakeSub intake;

        public IntakeOff(IntakeSub intake) {
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