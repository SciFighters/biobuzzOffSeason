package org.firstinspires.ftc.teamcode.subSystems;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import org.firstinspires.ftc.teamcode.HardwareConfig;

import java.util.function.BooleanSupplier;

/**
 * Mecanum drive with goBilda Pinpoint for odometry.
 * Everything moves field-relative using the Pinpoint's heading.
 */
public class DriveSub extends SubsystemBase {
    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx rearLeft;
    public DcMotorEx rearRight;
    public GoBildaPinpointDriver pinpoint;

    // Lets blocking helpers stop when the opMode ends.
    private BooleanSupplier activeGuard = () -> true;

    // PIDF gains — tune these for your robot.
    private final PIDFController rotatePID = new PIDFController(0.8, 0.0, 0.05, 0.0);
    private final PIDFController translatePID = new PIDFController(0.005, 0.0, 0.0005, 0.0);

    public DriveSub(HardwareConfig hm) {
        frontLeft = hm.frontLeft;
        frontRight = hm.frontRight;
        rearLeft = hm.rearLeft;
        rearRight = hm.rearRight;
        pinpoint = hm.pinpoint;

        rotatePID.setTolerance(0.03);     // ~1.7 degrees
        translatePID.setTolerance(8.0);   // 8mm
    }

    /** Pass in opModeIsActive() so loops can bail out cleanly. */
    public void setActiveGuard(BooleanSupplier guard) { this.activeGuard = guard; }

    /** Grab the latest pose from the Pinpoint. Call this before reading pos/heading. */
    public void update() {
        if (pinpoint != null) pinpoint.update();
    }

    // --- where am I ---

    /** Current heading in radians, wrapped to [-PI, PI]. */
    public double getHeadingRad() {
        if (pinpoint == null) return 0.0;
        update();
        return pinpoint.getPosition().getHeading(AngleUnit.RADIANS);
    }

    /** Current X position on the field. */
    public double getPosX(DistanceUnit unit) {
        if (pinpoint == null) return 0.0;
        update();
        return pinpoint.getPosition().getX(unit);
    }

    /** Current Y position on the field. */
    public double getPosY(DistanceUnit unit) {
        if (pinpoint == null) return 0.0;
        update();
        return pinpoint.getPosition().getY(unit);
    }

    // --- reset / set pose ---

    /** Zero out position and re-calibrate the IMU. */
    public void resetHeading() {
        if (pinpoint == null) return;
        pinpoint.resetPosAndIMU();
    }

    /** Manually set where the robot thinks it is. */
    public void setPose(double x, double y, double headingDeg, DistanceUnit unit) {
        if (pinpoint == null) return;
        pinpoint.setPosition(new Pose2D(unit, x, y, AngleUnit.DEGREES, headingDeg));
    }

    // --- driving ---

    /**
     * Field-centric drive. No matter which way the robot faces, +Y always
     * goes the same direction on the field.
     */
    public void drive(double xSpeed, double ySpeed, double rotSpeed) {
        double heading = getHeadingRad();

        double cosH = Math.cos(-heading);
        double sinH = Math.sin(-heading);

        // Rotate the field-relative input into the robot's frame.
        double rotX = xSpeed * cosH - ySpeed * sinH; // forward/back
        double rotY = xSpeed * sinH + ySpeed * cosH; // strafe

        double frontLeftPower  = rotY + rotX + rotSpeed;
        double frontRightPower = rotY - rotX - rotSpeed;
        double rearLeftPower   = rotY - rotX + rotSpeed;
        double rearRightPower  = rotY + rotX - rotSpeed;

        // Don't let any wheel exceed full power.
        double max = Math.max(Math.abs(frontLeftPower), Math.max(
                Math.abs(frontRightPower), Math.max(
                        Math.abs(rearLeftPower), Math.abs(rearRightPower)
                )
        ));
        if (max > 1.0) {
            frontLeftPower  /= max;
            frontRightPower /= max;
            rearLeftPower  /= max;
            rearRightPower /= max;
        }

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        rearLeft.setPower(rearLeftPower);
        rearRight.setPower(rearRightPower);
    }

    /** Kill all motor power. */
    public void stop() { drive(0.0, 0.0, 0.0); }

    // --- motion helpers ---

    /**
     * Turn to a specific field heading using PIDF.
     * Picks the shortest way around, bails out if it times out.
     */
    public void rotate(double targetRad, double power, long timeoutMs, double tolRad) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        rotatePID.reset();
        while (activeGuard.getAsBoolean() && timer.milliseconds() < timeoutMs) {
            double error = wrapAngle(targetRad - getHeadingRad());
            if (Math.abs(error) < tolRad) break;

            // PIDF output is proportional to how far off we are.
            double output = rotatePID.calculate(0.0, -error);
            output = Math.max(-power, Math.min(power, output));
            drive(0.0, 0.0, output);
        }
        stop();
    }

    /** Same thing with sane defaults. */
    public void rotate(double targetRad, double power) {
        rotate(targetRad, power, 3000, 0.05);
    }

    /**
     * Drive in a direction at a power for a set time.
     * Field-relative: x is left/right, y is forward/back.
     */
    public void translateTime(double xPower, double yPower, long ms) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (activeGuard.getAsBoolean() && timer.milliseconds() < ms) {
            drive(xPower, yPower, 0.0);
        }
        stop();
    }

    /** Convenience for going straight forward (+1) or backward (-1). */
    public void translateStraight(double power, int dir, long ms) {
        translateTime(0.0, power * dir, ms);
    }

    /**
     * Drive to a specific spot on the field using PIDF.
     * Slows down as it gets close, stops within tolMm.
     */
    public void translateTo(double targetX, double targetY, double power,
                             long timeoutMs, double tolMm, DistanceUnit unit) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        translatePID.reset();
        while (activeGuard.getAsBoolean() && timer.milliseconds() < timeoutMs) {
            update();
            double curX = getPosX(unit);
            double curY = getPosY(unit);
            double dx = targetX - curX;
            double dy = targetY - curY;
            double dist = Math.hypot(dx, dy);

            if (dist < tolMm) break;

            // PIDF scales the speed based on remaining distance.
            double mag = translatePID.calculate(0.0, -dist);
            mag = Math.max(-power, Math.min(power, mag));

            // Point at the target and go.
            double xCmd = (dx / dist) * mag;
            double yCmd = (dy / dist) * mag;
            drive(xCmd, yCmd, 0.0);
        }
        stop();
    }

    /** MM version with a default tolerance. */
    public void translateTo(double targetXMm, double targetYMm, double power, long timeoutMs) {
        translateTo(targetXMm, targetYMm, power, timeoutMs, 15.0, DistanceUnit.MM);
    }

    // --- utilities ---

    /** Keep an angle in the [-PI, PI] range. */
    private double wrapAngle(double a) {
        while (a > Math.PI)  a -= 2.0 * Math.PI;
        while (a < -Math.PI) a += 2.0 * Math.PI;
        return a;
    }
}
