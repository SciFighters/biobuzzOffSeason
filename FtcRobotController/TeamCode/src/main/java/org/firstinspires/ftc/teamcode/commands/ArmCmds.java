package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subSystems.ArmSub;

/**
 * Collection of commands for controlling the arm subsystem.
 * 
 * <p>All commands use PID control for precise positioning and movement.
 * Each command follows the CommandBase pattern and integrates with the ArmSub subsystem.
 */
public class ArmCmds {

    /**
     * Command to set arm to a specific target position (in degrees) using PID control.
     * Automatically calculates power adjustments to reach target angle.
     */
    public static class ArmSetPosition extends CommandBase {
        private final ArmSub armSub;
        private final double targetDegrees;
        private static final double TOLERANCE_DEGREES = 2.0;

        public ArmSetPosition(ArmSub armSub, double targetDegrees) {
            this.armSub = armSub;
            this.targetDegrees = targetDegrees;
            addRequirements(armSub);
        }

        @Override
        public void initialize() {
            armSub.pid.reset(); // Clear PID history to prevent windup
        }

        @Override
        public void execute() {
            double currentAngle = armSub.getAngle();
            double power = armSub.pid.calculate(currentAngle, targetDegrees);
            armSub.setPower(power);
        }

        @Override
        public boolean isFinished() {
            double error = Math.abs(armSub.getAngle() - targetDegrees);
            return error <= TOLERANCE_DEGREES;
        }

        @Override
        public void end(boolean interrupted) {
            armSub.setPower(0); // Stop motor when command completes
        }
    }

    /**
     * Command to hold the arm at its current position using PID control.
     * Maintains position indefinitely until interrupted or completed.
     */
    public static class ArmHoldPosition extends CommandBase {
        private final ArmSub armSub;
        private double targetDegrees; // Current target angle

        public ArmHoldPosition(ArmSub armSub) {
            this.armSub = armSub;
            addRequirements(armSub);
        }

        @Override
        public void initialize() {
            targetDegrees = armSub.getAngle(); // Set target to current position
            armSub.pid.reset(); // Clear PID history
        }

        @Override
        public void execute() {
            double currentAngle = armSub.getAngle();
            double power = armSub.pid.calculate(currentAngle, targetDegrees);
            armSub.setPower(power);
        }

        @Override
        public boolean isFinished() {
            return false; // Continues until interrupted
        }

        @Override
        public void end(boolean interrupted) {
            armSub.setPower(0); // Stop motor when command ends
        }
    }

    /**
     * Command to set arm power directly (open-loop control).
     * Use for simple manual control without PID precision.
     */
    public static class ArmPower extends CommandBase {
        private final ArmSub armSub;
        private final double power;

        public ArmPower(ArmSub armSub, double power) {
            this.armSub = armSub;
            this.power = power;
            addRequirements(armSub);
        }

        @Override
        public void initialize() {
            armSub.setPower(power); // Set immediate power level
        }

        @Override
        public boolean isFinished() {
            return true; // One-shot command
        }
    }

    /**
     * Command to move arm to a specific position using PID.
     * *Deprecated in favor of ArmSetPosition - use ArmSetPosition instead.*
     */
    public static class MoveArmToPos extends CommandBase {
        private final ArmSub armSub;
        private final double target;

        public MoveArmToPos(ArmSub armSub, double targetDeg) {
            this.armSub = armSub;
            this.target = targetDeg;
            addRequirements(armSub);
        }

        @Override
        public void execute() {
            // Deprecated implementation - continues execution
        }

        @Override
        public boolean isFinished() {
            return false; // Never finishes
        }
    }
}