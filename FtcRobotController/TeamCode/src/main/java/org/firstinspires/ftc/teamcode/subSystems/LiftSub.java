package org.firstinspires.ftc.teamcode.subSystems;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.HardwareConfig;
import org.firstinspires.ftc.teamcode.Utilities.MotorOut;

public class LiftSub extends SubsystemBase {
    public MotorOut rightMotor;
    public MotorOut leftMotor;

    double radiusM;
    private static final double toleranceM = 0.1;



    private final PIDFController liftPID = new PIDFController(0.03, 0, 1e-3, 0); //uncalibrated


    public LiftSub(HardwareConfig hm) {
        liftPID.setTolerance(toleranceM);
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
        double rightHeight = getPos()[0] * 2 * Math.PI * radiusM
                / rightMotor.ticksPerRevolution();

        double leftHeight = getPos()[1]  * 2 * Math.PI * radiusM
                / leftMotor.ticksPerRevolution();

        // Value in Meters
        return new double[] { rightHeight, leftHeight };
    }

    public double getHeightAvg() {
        double[] h = getHeight();
        return (h[0] + h[1]) / 2.0;
    }

    public boolean isLevel() {
        return getPos()[0] == getPos()[1];
    }

    public void goToHeight(double targetHeightM) {
        double current = getHeightAvg();
        double output = liftPID.calculate(current, targetHeightM);
        setPower(output);
    }

    public void resetLiftPID() {
        liftPID.reset();
    }

    public boolean atTargetHeight() {
        return liftPID.atSetPoint();
    }
}