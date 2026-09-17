package org.firstinspires.ftc.teamcode.subSystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Utilities.MotorOut;

public class IntakeSub extends SubsystemBase {

    private final MotorOut intakeMotor;

    public IntakeSub(MotorOut intakeMotor) {
        this.intakeMotor = intakeMotor;
    }

    public void setPower(double power) {
        intakeMotor.setPower(power);
    }

    public double getPower() {
        return intakeMotor.getPower();
    }
}
