package org.firstinspires.ftc.teamcode.subSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.MotorOut;
import org.firstinspires.ftc.teamcode.HardwareConfig;
import org.firstinspires.ftc.teamcode.Utilities.pid.PIDConfig;
import org.firstinspires.ftc.teamcode.Utilities.pid.PIDController;

public class ArmSub extends SubsystemBase {
    public MotorOut armMotor;

    Double startAngle = 90.0;       // start angle
    public final PIDController pid = new PIDController(); // controller

    public ArmSub(HardwareConfig hm) {
        if (hm == null) {
            throw new IllegalArgumentException("HardwareConfig cannot be null");
        }
        if (hm.armMotor == null) {
            throw new IllegalStateException("Arm motor not initialized in HardwareConfig");
        }
        armMotor = hm.armMotor;
        pid.addProfile(PIDConfig.builder()
                .name("default")
                .kp(0.02)
                .ki(0.0005)
                .kd(0.001)
                .build());
        pid.setProfile("default");
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
        return armMotor.getPosAngle();
    }

    void setStartAngle(double a) {
        startAngle = a;
    }

    public Double getStartAngle() {
        return startAngle;
    }
}