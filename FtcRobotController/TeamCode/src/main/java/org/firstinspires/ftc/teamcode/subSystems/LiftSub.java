package org.firstinspires.ftc.teamcode.subSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.HardwareConfig;
import org.firstinspires.ftc.teamcode.MotorOut;
import org.firstinspires.ftc.teamcode.Utilities.pid.PIDConfig;
import org.firstinspires.ftc.teamcode.Utilities.pid.PIDController;

public class LiftSub extends SubsystemBase {

    private final MotorOut leftLift;
    private final MotorOut rightLift;
    private final PIDController leftPID;
    private final PIDController rightPID;

    private static final PIDConfig NormalSpeed = new PIDConfig.Builder()
            .name("LIFT_NORMAL")
            .description("Standard speed for lift positioning")
            .kp(0.08)
            .ki(0.02)
            .kd(0.05)
            .tolerance(5)
            .integralZone(10)
            .maxIntegral(1.0)
            .integralLeakRate(0.95)
            .outputDeadband(0.0)
            .errorDeadband(0.0)
            .maxOutputChangePerSecond(0.5)
            .build();

    public LiftSub(HardwareConfig hm) {
        leftLift = hm.leftLift;
        rightLift = hm.rightLift;
        leftPID = new PIDController(NormalSpeed);
        rightPID = new PIDController(NormalSpeed);
    }

    public void setPower(double power) {
        leftLift.setPower(power);
        rightLift.setPower(power);
    }

    public double getPower() {
        return (leftLift.getPower() + rightLift.getPower()) / 2.0;
    }

    public void setPosition(int position) {
        int leftPos = leftLift.getPosTicks();
        int rightPos = rightLift.getPosTicks();
        double leftError = position - leftPos;
        double rightError = position - rightPos;
        double leftOutput = leftPID.calculateError(leftError);
        double rightOutput = rightPID.calculateError(rightError);
        // clamp output to motor power range
        leftOutput = Math.max(-1.0, Math.min(1.0, leftOutput));
        rightOutput = Math.max(-1.0, Math.min(1.0, rightOutput));
        leftLift.setPower(leftOutput);
        rightLift.setPower(rightOutput);
    }

    public int getPosition() {
        return (leftLift.getPosTicks() + rightLift.getPosTicks()) / 2;
    }

    /** Reset PID controllers (call when needed) */
    public void resetLiftPID() {
        leftPID.reset();
        rightPID.reset();
    }

    /** Check if lift is at target within tolerance (uses config tolerance) */
    public boolean atPosition(int targetTicks) {
        int leftPos = leftLift.getPosTicks();
        int rightPos = rightLift.getPosTicks();
        return Math.abs(leftPos - targetTicks) <= NormalSpeed.tolerance &&
               Math.abs(rightPos - targetTicks) <= NormalSpeed.tolerance;
    }
}