package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subSystems.IntakeSub;


/**
 * Collection of commands that control the intake mechanism.
 * 
 * <p>Each inner class represents a specific command that can be scheduled 
 * to set the intake motor power in various directions.
 */
public class IntakeCmds {

    /**
     * Command to spin the intake forward.
     * This sets the intake motor power to 1.0 (full forward).
     */
    public static class IntakeForward extends CommandBase {

        private final IntakeSub intake;

        public IntakeForward(IntakeSub intake) {
            this.intake = intake;
            addRequirements(intake);
        }

        @Override
        public void initialize() {
            // Spin intake forward
            intake.setPower(1.0);
        }

        @Override
        public boolean isFinished() {
            // Command completes immediately
            return true;
        }
    }

    /**
     * Command to spin the intake backward.
     * This sets the intake motor power to -1.0 (full reverse).
     */
    public static class IntakeBackward extends CommandBase {

        private final IntakeSub intake;

        public IntakeBackward(IntakeSub intake) {
            this.intake = intake;
            addRequirements(intake);
        }

        @Override
        public void initialize() {
            // Spin intake backward
            intake.setPower(-1.0);
        }

        @Override
        public boolean isFinished() {
            // Command completes immediately
            return true;
        }
    }

    /**
     * Command to stop the intake.
     * This sets the intake motor power to 0.0 (off).
     */
    public static class IntakeOff extends CommandBase {

        private final IntakeSub intake;

        public IntakeOff(IntakeSub intake) {
            this.intake = intake;
            addRequirements(intake);
        }

        @Override
        public void initialize() {
            // Stop intake motor
            intake.setPower(0.0);
        }

        @Override
        public boolean isFinished() {
            // Command completes immediately
            return true;
        }
    }
}