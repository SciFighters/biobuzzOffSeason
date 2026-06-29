package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subSystems.ArmSub;
import java.util.Objects;

/**
 * Arm command implementations.
 * <p>
 *  • ArmGoToAngle – moves to an absolute angle using inline PID.
 *  • ArmHoldPosition – holds the current angle (prevents gravity drop).
 *  • ArmPower – one‑shot open‑loop power command.
 *  • MoveArmToPos – legacy no‑op kept for backward compatibility.
 * </p>
 * <p>
 *  The PID math is done on‑the‑spot – no external PIDController required.
 * </p>
 */
public class ArmCmds extends CommandBase {

    /* --------------------------------------------------------------- */
    /*  Tunable constants – adjust to match your mechanism               */
    /* --------------------------------------------------------------- */
    private static final double KP = 0.14;   // proportional gain
    private static final double KI = 0.0;    // integral gain
    private static final double KD = 0.07;   // derivative gain
    private static final double MAX_POWER = 1.0;
    private static final double MIN_POWER = -1.0;
    private static final double TOLERANCE_DEGREES = 20.0;

    public static class ArmGoToAngle extends CommandBase {
        private final ArmSub armSub;
        private final double targetDegrees;

        public ArmGoToAngle(ArmSub armSub, double targetDegrees) {
            this.armSub = Objects.requireNonNull(armSub, "armSub cannot be null");
            this.targetDegrees = targetDegrees;
            addRequirements(armSub);
        }

        @Override
        public void initialize() {
            // Reset PID state for a clean start
            armSub.resetArmPID();
        }

        @Override
        public void execute() {
            // Let ArmSub handle PID
            armSub.setTargetAngle(targetDegrees);
        }

        @Override
        public boolean isFinished() {
            // finished when within tolerance defined in ArmSub
            return armSub.atTargetAngle();
        }

        @Override
        public void end(boolean interrupted) {
            if (!interrupted) {
                // Hold the achieved position
                new ArmHoldPosition(armSub, targetDegrees).schedule();
            } else {
                // If interrupted, stop power to avoid unexpected motion
                armSub.setPower(0);
            }
        }
    }

    /* --------------------------------------------------------------- */
    /*  ArmHoldPosition – keeps the arm at its *current* angle          */
    /* --------------------------------------------------------------- */
    public static class ArmHoldPosition extends CommandBase {
        private final ArmSub armSub;
        private double targetDeg;   // angle we are trying to maintain
        private final boolean useProvidedTarget;

        /** Create a hold command that maintains the arm's current angle */
        public ArmHoldPosition(ArmSub armSub) {
            this.armSub = Objects.requireNonNull(armSub, "armSub cannot be null");
            this.targetDeg = 0.0; // placeholder
            this.useProvidedTarget = false;
            addRequirements(armSub);
        }

        /** Create a hold command that maintains a specific angle */
        public ArmHoldPosition(ArmSub armSub, double targetDeg) {
            this.armSub = Objects.requireNonNull(armSub, "armSub cannot be null");
            this.targetDeg = targetDeg;
            this.useProvidedTarget = true;
            addRequirements(armSub);
        }

        @Override
        public void initialize() {
            // If no target was provided, capture current angle
            if (!useProvidedTarget) {
                this.targetDeg = armSub.getAngle();
            }
            armSub.resetArmPID();
        }

        @Override
        public void execute() {
            armSub.setTargetAngle(targetDeg);
        }

        @Override
        public boolean isFinished() {
            return false; // hold until cancelled
        }

        @Override
        public void end(boolean interrupted) {
            armSub.setPower(0);
        }
    }

    /* --------------------------------------------------------------- */
    /*  ArmPower – one‑shot open‑loop power command for a limited time */
    /* --------------------------------------------------------------- */
    public static class ArmPower extends CommandBase {
        private final ArmSub armSub;
        private final double power;
        private final long finishTime; // epoch ms when the command ends

        public ArmPower(ArmSub armSub, double power, long durationMs) {
            this.armSub = Objects.requireNonNull(armSub, "armSub cannot be null");
            this.power = power;
            this.finishTime = System.currentTimeMillis() + durationMs;
            addRequirements(armSub);
        }

        @Override
        public void initialize() {
            armSub.setPower(power);
        }

        @Override
        public boolean isFinished() {
            return System.currentTimeMillis() >= finishTime;
        }

        @Override
        public void end(boolean interrupted) {
            armSub.setPower(0);
        }
    }

    /* --------------------------------------------------------------- */
    /*  MoveArmToPos – legacy placeholder (does nothing)                */
    /* --------------------------------------------------------------- */
    public static class MoveArmToPos extends CommandBase {
        private final ArmSub armSub;
        private final double targetDeg;

        public MoveArmToPos(ArmSub armSub, double targetDeg) {
            this.armSub = Objects.requireNonNull(armSub, "armSub cannot be null");
            this.targetDeg = targetDeg;
            addRequirements(armSub);
        }

        @Override
        public void execute() {
            // Reserved for legacy code; does nothing.
        }

        @Override
        public boolean isFinished() {
            return false;
        }
    }
}