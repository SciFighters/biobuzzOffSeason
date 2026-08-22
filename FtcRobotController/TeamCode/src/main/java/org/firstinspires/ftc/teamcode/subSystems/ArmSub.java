package org.firstinspires.ftc.teamcode.subSystems;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.Utilities.MotorOut;
import org.firstinspires.ftc.teamcode.HardwareConfig;

public class ArmSub extends SubsystemBase {
    public MotorOut armMotor;
    private final PIDFController armPID = new PIDFController(0.03, 0, 1e-3, 0);


    Double startAngle = 90.0;       // start angle
    public ArmSub(HardwareConfig hm) {
        armPID.setTolerance(5);
        if (hm == null) {
            throw new IllegalArgumentException("HardwareConfig cannot be null");
        }
        if (hm.armMotor == null) {
            throw new IllegalStateException("Arm motor not initialized in HardwareConfig");
        }
        armMotor = hm.armMotor;

        // Set startAngle so that getAngle() returns 90 at initial position
        double raw = armMotor.getPosAngle();
        startAngle = raw - 90.0;
    }

    public void setPower(double p) {
        if (armMotor == null) {
            throw new IllegalStateException("Arm motor not initialized");
        }
        armMotor.setPower(p);
    }

    double getPower() {
        if (armMotor == null) {
            throw new IllegalStateException("Arm motor not initialized");
        }
        return armMotor.getPower();
    }

    int getPos() {
        if (armMotor == null) {
            throw new IllegalStateException("Arm motor not initialized");
        }
        return armMotor.getPosTicks();
    }

    public double getAngle() {
        if (armMotor == null) {
            throw new IllegalStateException("Arm motor not initialized");
        }
        double raw = armMotor.getPosAngle();
        return raw - startAngle;
    }

    void setStartAngle(double a) {
        startAngle = a;
    }

    public Double getStartAngle() {
        return startAngle;
    }

    public void setTargetAngle(double targetAngleDeg) {
        double currentAngle = getAngle();
        double output = armPID.calculate(currentAngle, targetAngleDeg);
//        // Gravity feedforward: torque needed to hold position against gravity
//        // Assuming 0° = motor encoder zero, 90° = upright (vertical up).
//        double kg = 0.2; // tune this value (0.0-1.0) based on arm weight and gearing
//        double feedforward = kg * Math.cos(Math.toRadians(targetAngleDeg - 90.0));
//        output += feedforward;
//        // Clamp output to motor power range [-1,1]
//        output = Math.max(-1.0, Math.min(1.0, output));
        setPower(output);
    }

    public void resetArmPID() {
        armPID.reset();
    }
    public boolean atTargetAngle() {
        return armPID.atSetPoint();
    }
}
