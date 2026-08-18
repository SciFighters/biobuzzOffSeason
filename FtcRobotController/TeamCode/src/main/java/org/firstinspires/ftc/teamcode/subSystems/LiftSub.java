package org.firstinspires.ftc.teamcode.subSystems;

import static com.seattlesolvers.solverslib.geometry.Vector2dExtKt.getAngle;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.HardwareConfig;
import org.firstinspires.ftc.teamcode.Utilities.MotorOut;
import org.firstinspires.ftc.teamcode.Utilities.pid.PIDConfig;
import org.firstinspires.ftc.teamcode.Utilities.pid.PIDController;

public class LiftSub extends SubsystemBase {
    double spoolRadius;
    public MotorOut rightMotor;
    public MotorOut leftMotor;
    
    // uncalibrated value
    private final PIDFController liftPID = new PIDFController(0.03, 0, 1e-3, 0); 



    Double startHeight = 0.0;
    public LiftSub(HardwareConfig hm) {
        liftPID.setTolerance(5);
        rightMotor = hm.rightLift;
        leftMotor = hm.leftLift;
    }

    public void setPower(double p) {
        rightMotor.setPower(p);
        leftMotor.setPower(p);
    }

    public double[] getPower() {
        return new double[] {rightMotor.getPower(), leftMotor.getPower()};
    }

    public int[] getPos() {
        return new int[] {rightMotor.getPosTicks(), leftMotor.getPosTicks()};
    }

    public double[] getHeight() {
        double rightHeight = getPos()[0]/ rightMotor.ticksPerRevolution() * 2 * Math.PI * rightMotor.getRadius();
        double leftHeight = getPos()[1]/ leftMotor.ticksPerRevolution() * 2 * Math.PI * leftMotor.getRadius();

        return new double[] { rightHeight, leftHeight};
    }

    void setStartHeight(double h) {
        startHeight = h;
    }

    public boolean isLevel() {
        return getPos()[0] == getPos()[1];
    }

    /**
     * Use PID to drive arm to a target angle (in degrees).
     * Call this repeatedly in your loop (e.g., in a command's execute).
     * Includes a simple gravity feedforward term.
     */
    public void setTargetAngle(double targetAngleDeg) {
        double[] height = getHeight();
        double output = liftPID.calculate(height[0], targetAngleDeg);

        setPower(output);
    }

    /** Reset PID integral/derivative when needed */
    public void resetArmPID() {
        liftPID.reset();
    }

    /** Check if arm is at target within tolerance */
    public boolean atTargetAngle() {
        return liftPID.atSetPoint();
    }

    /** Average height of both lift sides (mm) */
    public double getHeightAvg() {
        double[] h = getHeight();
        return (h[0] + h[1]) / 2.0;
    }

    /** Drive lift to target height (mm) using PID */
    public void powerToTargetHeight(double targetHeight) {
        double current = getHeightAvg();
        double output = liftPID.calculate(current, targetHeight);
        setPower(output);
    }

    /** True when lift within PID tolerance of target height */
    public boolean atTargetHeight() {
        return liftPID.atSetPoint();
    }
}