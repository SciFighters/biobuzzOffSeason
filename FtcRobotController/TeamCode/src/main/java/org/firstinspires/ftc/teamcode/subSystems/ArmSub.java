package org.firstinspires.ftc.teamcode.subSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.HardwareConfig;
import org.firstinspires.ftc.teamcode.Utilities.pid.PIDConfig;
import org.firstinspires.ftc.teamcode.Utilities.pid.PIDController;

public class ArmSub extends SubsystemBase {
    private DcMotorEx armMotor;
    private static final int tickPerRev = 1120;
    
    Double startAngle = null;       // start angle
    public final PIDController pid = new PIDController(); // controller

    public ArmSub(HardwareConfig hm) {
        armMotor = hm.armMotor;
        pid.addProfile(PIDConfig.builder()
                .name("default")
                .kp(0.001)
                .build());
        pid.setProfile("default");
    }

    public void setPower(double p) {
        armMotor.setPower(p);
    }

    double getPower() {
        return armMotor.getPower();
    }

    int getPos() {
        return armMotor.getCurrentPosition();
    }

    public double getAngle() {
        return getPos() * 360.0 / tickPerRev;
    }

    void setStartAngle(double a) {
        startAngle = a;
    }

    Double getStartAngle() {
        return startAngle;
    }
}