package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.subSystems.DriveSub;

/**
 * Commands wrapping DriveSub's field-oriented PIDF movements.
 * Each runs until the robot reaches its target or times out.
 */
public final class DriveCmds {

    private DriveCmds() {}

    /** Reset the Pinpoint position and IMU to zero. */
    public static class ResetPose extends CommandBase {
        private final DriveSub drive;

        public ResetPose(DriveSub drive) {
            this.drive = drive;
            addRequirements(drive);
        }

        @Override
        public void initialize() {
            drive.resetHeading();
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    /** Turn to a specific field heading in radians. */
    public static class Rotate extends CommandBase {
        private final DriveSub drive;
        private final double targetRad;
        private final double power;
        private final long timeoutMs;
        private final double tolRad;
        private long startTime;

        public Rotate(DriveSub drive, double targetRad, double power) {
            this(drive, targetRad, power, 3000, 0.05);
        }

        public Rotate(DriveSub drive, double targetRad, double power,
                      long timeoutMs, double tolRad) {
            this.drive = drive;
            this.targetRad = targetRad;
            this.power = power;
            this.timeoutMs = timeoutMs;
            this.tolRad = tolRad;
            addRequirements(drive);
        }

        @Override
        public void initialize() {
            startTime = System.currentTimeMillis();
        }

        @Override
        public void execute() {
            double error = wrapAngle(targetRad - drive.getHeadingRad());
            double output = Math.copySign(Math.min(Math.abs(error) * power, power), error);
            drive.drive(0.0, 0.0, output);
        }

        @Override
        public void end(boolean interrupted) {
            drive.stop();
        }

        @Override
        public boolean isFinished() {
            double error = Math.abs(wrapAngle(targetRad - drive.getHeadingRad()));
            return error < tolRad || (System.currentTimeMillis() - startTime) >= timeoutMs;
        }

        private double wrapAngle(double a) {
            while (a > Math.PI)  a -= 2.0 * Math.PI;
            while (a < -Math.PI) a += 2.0 * Math.PI;
            return a;
        }
    }

    /** Drive to a specific (x, y) spot on the field. */
    public static class TranslateTo extends CommandBase {
        private final DriveSub drive;
        private final double targetX, targetY;
        private final double power;
        private final long timeoutMs;
        private final double tolMm;
        private final DistanceUnit unit;
        private long startTime;

        public TranslateTo(DriveSub drive, double targetXMm, double targetYMm, double power) {
            this(drive, targetXMm, targetYMm, power, 3000);
        }

        public TranslateTo(DriveSub drive, double targetX, double targetY, double power,
                           long timeoutMs) {
            this(drive, targetX, targetY, power, timeoutMs, 15.0, DistanceUnit.MM);
        }

        public TranslateTo(DriveSub drive, double targetX, double targetY, double power,
                           long timeoutMs, double tolMm, DistanceUnit unit) {
            this.drive = drive;
            this.targetX = targetX;
            this.targetY = targetY;
            this.power = power;
            this.timeoutMs = timeoutMs;
            this.tolMm = tolMm;
            this.unit = unit;
            addRequirements(drive);
        }

        @Override
        public void initialize() {
            startTime = System.currentTimeMillis();
        }

        @Override
        public void execute() {
            drive.update();
            double curX = drive.getPosX(unit);
            double curY = drive.getPosY(unit);
            double dx = targetX - curX;
            double dy = targetY - curY;
            double dist = Math.hypot(dx, dy);

            if (dist <= 0) {
                drive.stop();
                return;
            }

            double error = (dist / 200.0) * power; // ms → expected range; tune
            error = Math.min(error, power);

            double xCmd = (dx / dist) * error;
            double yCmd = (dy / dist) * error;
            drive.drive(xCmd, yCmd, 0.0);
        }

        @Override
        public void end(boolean interrupted) {
            drive.stop();
        }

        @Override
        public boolean isFinished() {
            drive.update();
            double curX = drive.getPosX(unit);
            double curY = drive.getPosY(unit);
            double dist = Math.hypot(targetX - curX, targetY - curY);

            return dist < tolMm || (System.currentTimeMillis() - startTime) >= timeoutMs;
        }
    }

    /** Pause for milliseconds. */
    public static class Wait extends CommandBase {
        private final long ms;
        private long startTime;

        public Wait(long ms) {
            this.ms = ms;
        }

        @Override
        public void initialize() {
            startTime = System.currentTimeMillis();
        }

        @Override
        public boolean isFinished() {
            return (System.currentTimeMillis() - startTime) >= ms;
        }
    }

    /** Build the full TestAuto sequence. */
    public static SequentialCommandGroup testAutoSequence(DriveSub drive) {
        return new SequentialCommandGroup(
            new ResetPose(drive),
            new TranslateTo(drive, 0.0, 600.0, 0.45, 4000),   // drive to center
            new Wait(200),
            new Rotate(drive, 2.0 * Math.PI, 0.40),            // spin 360
            new Wait(200),
            new TranslateTo(drive, 0.0, 1100.0, 0.45, 3000),   // forward 50cm
            new Wait(200),
            new TranslateTo(drive, 0.0, 100.0, 0.45, 3000),    // back 50cm
            new Wait(200),
            new TranslateTo(drive, 0.0, 1100.0, 0.45, 3000),   // forward 50cm
            new Wait(200),
            new TranslateTo(drive, 0.0, 100.0, 0.45, 3000)      // back 50cm
        );
    }
}