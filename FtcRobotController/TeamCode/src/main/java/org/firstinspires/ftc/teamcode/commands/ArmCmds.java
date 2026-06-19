package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subSystems.ArmSub;
import java.util.Objects;

/**
 * Collection of commands for controlling the arm subsystem.
 * 
 * <p>All commands use PID control for precise positioning and movement.
 * Each command follows the CommandBase pattern and integrates with the ArmSub subsystem.
 */
public class ArmCmds {

public static class ArmGoToAngle extends CommandBase {
         private  ArmSub armSub;
         private final double targetDegrees;
          private static final double TOLERANCE_DEGREES = 5.0;

         public ArmGoToAngle(ArmSub armSub, double targetDegrees) {
             this.armSub = Objects.requireNonNull(armSub, "armSub cannot be null");
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
              double target = targetDegrees;
              double error = target - currentAngle;
              double kp = 0.05; // proportional gain
              double power = kp * error;
              // Limit power to avoid saturation
              if (power > 1.0) power = 1.0;
              if (power < -1.0) power = -1.0;
              armSub.setPower(power);
          }

          @Override
          public boolean isFinished() {
              double current = armSub.getAngle();
              double target = targetDegrees;
              double error = Math.abs(current - target);
              return error <= TOLERANCE_DEGREES;
          }

         @Override
         public void end(boolean interrupted) {
             armSub.setPower(0); // Stop motor when command completes
         }
     }


    public static class ArmHoldPosition extends CommandBase {
        private final ArmSub armSub;
        private double targetDegrees; // Current target angle

        public ArmHoldPosition(ArmSub armSub) {
            this.armSub = Objects.requireNonNull(armSub, "armSub cannot be null");
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
            this.armSub = Objects.requireNonNull(armSub, "armSub cannot be null");
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
            this.armSub = Objects.requireNonNull(armSub, "armSub cannot be null");
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