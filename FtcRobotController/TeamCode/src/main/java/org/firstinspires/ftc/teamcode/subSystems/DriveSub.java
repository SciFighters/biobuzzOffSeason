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

public class DriveSub extends SubsystemBase {
    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx rearLeft;
    public DcMotorEx rearRight;
    public GoBildaPinpointDriver pinpoint;

    private BooleanSupplier activeGuard = () -> true;

    private final PIDFController rotatePID = new PIDFController(0.8, 0.0, 0.05, 0.0);
    private final PIDFController translatePID = new PIDFController(0.005, 0.0, 0.0005, 0.0);

    public DriveSub(HardwareConfig hm) {
        frontLeft = hm.frontLeft;
        frontRight = hm.frontRight;
        rearLeft = hm.rearLeft;
        rearRight = hm.rearRight;
        pinpoint = hm.pinpoint;

        rotatePID.setTolerance(0.03); // ~1.7 degrees
        translatePID.setTolerance(8.0); // 8mm
    }

    public void setActiveGuard(BooleanSupplier guard) { this.activeGuard = guard; }

    public void update() {
        if (pinpoint != null) pinpoint.update();
    }

    public double getHeadingRad() {
        if (pinpoint == null) return 0.0;
        update();
        return pinpoint.getPosition().getHeading(AngleUnit.RADIANS);
    }

    public double[] getPos2d(DistanceUnit unit) {
        if (pinpoint == null) return new double[] {0.0, 0.0};
        update();
        return new double[] {pinpoint.getPosition().getX(unit), pinpoint.getPosition().getY(unit)};
    }

    public void resetHeading() {
        if (pinpoint == null) return;
        pinpoint.resetPosAndIMU();
    }

    public void manualResetPose(double x, double y, double headingDeg, DistanceUnit unit) {
        if (pinpoint == null) return;
        pinpoint.setPosition(new Pose2D(unit, x, y, AngleUnit.DEGREES, headingDeg));
    }

    public void drive(double xSpeed, double ySpeed, double rotSpeed) {
        double heading = getHeadingRad();

        double cosH = Math.cos(-heading);
        double sinH = Math.sin(-heading);

        double rotX = xSpeed * cosH - ySpeed * sinH;
        double rotY = xSpeed * sinH + ySpeed * cosH;

        double frontLeftPower  = rotY + rotX + rotSpeed;
        double frontRightPower = rotY - rotX - rotSpeed;
        double rearLeftPower   = rotY - rotX + rotSpeed;
        double rearRightPower  = rotY + rotX - rotSpeed;

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

    public void stop() { drive(0.0, 0.0, 0.0); }


    public void rotate(double targetRad, double power, long timeoutMs, double tolRad) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        rotatePID.reset();
        while (activeGuard.getAsBoolean() && timer.milliseconds() < timeoutMs) {
            double error = wrapAngle(targetRad - getHeadingRad());
            if (Math.abs(error) < tolRad) break;

            double output = rotatePID.calculate(0.0, -error);
            output = Math.max(-power, Math.min(power, output));
            drive(0.0, 0.0, output);
        }
        stop();
    }

    public void rotate(double targetRad, double power) {
        rotate(targetRad, power, 3000, 0.05);
    }

    public void translateTo(double targetX, double targetY, double power,
                             long timeoutMs, double tolMm, DistanceUnit unit) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        translatePID.reset();
        while (activeGuard.getAsBoolean() && timer.milliseconds() < timeoutMs) {
            update();
            double curX = getPos2d(unit)[0];
            double curY = getPos2d(unit)[1];
            double dx = targetX - curX;
            double dy = targetY - curY;
            double dist = Math.hypot(dx, dy);

            if (dist < tolMm) break;

            double mag = translatePID.calculate(0.0, -dist);
            mag = Math.max(-power, Math.min(power, mag));

            double xCmd = (dx / dist) * mag;
            double yCmd = (dy / dist) * mag;
            drive(xCmd, yCmd, 0.0);
        }
        stop();
    }

    public void translateTo(double targetXMm, double targetYMm, double power, long timeoutMs) {
        translateTo(targetXMm, targetYMm, power, timeoutMs, 15.0, DistanceUnit.MM);
    }


    // Keeps angle in the -PI -> PI range
    private double wrapAngle(double a) {
        while (a > Math.PI)  a -= 2.0 * Math.PI;
        while (a < -Math.PI) a += 2.0 * Math.PI;
        return a;
    }
}
