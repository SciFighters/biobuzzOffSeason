package org.firstinspires.ftc.teamcode.subSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.HardwareConfig;
import org.firstinspires.ftc.teamcode.MotorOut;
import org.firstinspires.ftc.teamcode.Utilities.pid.PIDConfig;


public class LiftSub extends SubsystemBase {

    private final MotorOut leftLift;
    private final MotorOut rightLift;

    
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
    }

    public void setPower(double power) {
        leftLift.setPower(power);
        rightLift.setPower(power);
    }

    
    public double getPower() {
        return (leftLift.getPower() + rightLift.getPower()) / 2.0;
    }

    public void setPosition(int position) {
        leftLift.goToPosPID(position, NormalSpeed);
        rightLift.goToPosPID(position, NormalSpeed);
    }

    public int getPosition() {
        return (leftLift.getPosTicks() + rightLift.getPosTicks()) / 2;
    }
}