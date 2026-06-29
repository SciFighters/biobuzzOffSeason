package org.firstinspires.ftc.teamcode.subSystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.HardwareConfig;
import org.firstinspires.ftc.teamcode.Utilities.pid.MotorOut;

// Intake subsystem
public class IntakeSub extends SubsystemBase {

    private MotorOut intakeMotor ;

    public IntakeSub(HardwareConfig hm) {

        intakeMotor = hm.intakeMotor;
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
