package org.firstinspires.ftc.teamcode.subSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.HardwareProvider;

// Intake subsystem
public class IntakeSub {
    private DcMotor intakeMotor;

    public IntakeSub() {
        intakeMotor = HardwareProvider.getDcMotor("intakeMotor");
    }

    // Set intake power
    // @param power -1 to 1
    public void setPower(double power) {
        intakeMotor.setPower(power);
    }

    // Get intake power
    // @return power level
    public double getPower() {
        return intakeMotor.getPower();
    }
}