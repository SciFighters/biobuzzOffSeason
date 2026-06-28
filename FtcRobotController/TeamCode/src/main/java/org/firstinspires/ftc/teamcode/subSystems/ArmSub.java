package org.firstinspires.ftc.teamcode.subSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.Utilities.pid.MotorOut;
import org.firstinspires.ftc.teamcode.HardwareConfig;
import org.firstinspires.ftc.teamcode.Utilities.pid.PIDConfig;
import org.firstinspires.ftc.teamcode.Utilities.pid.PIDController;

public class ArmSub extends SubsystemBase {
    public MotorOut armMotor;
    private final PIDController armPID;
    private final PIDConfig armConfig;

    Double startAngle = 90.0;       // start angle
    public ArmSub(HardwareConfig hm) {
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

        // PID config for arm (tuned for less overshoot)
        armConfig = PIDConfig.builder()
                .name("ARM")
                .description("Arm position hold")
                .kp(12.0/90)        // reduced proportional to curb overshoot
                .ki(0.002)       // low integral
                .kd(0.03)        // increased derivative for damping
                .tolerance(2)    // degrees tolerance
                .integralZone(12)
                .maxIntegral(10)
                .maxOutputChangePerSecond(0.6)
                .build();
        armPID = new PIDController(armConfig);
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

    /**
     * Use PID to drive arm to a target angle (in degrees).
     * Call this repeatedly in your loop (e.g., in a command's execute).
     * Includes a simple gravity feedforward term.
     */
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
        armMotor.setPower(output);
    }

    /** Reset PID integral/derivative when needed */
    public void resetArmPID() {
        armPID.reset();
    }

    /** Check if arm is at target within tolerance */
    public boolean atTargetAngle() {
        return armPID.atTarget();
    }
}