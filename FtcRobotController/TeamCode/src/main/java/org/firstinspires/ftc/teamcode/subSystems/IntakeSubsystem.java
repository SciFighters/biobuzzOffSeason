package org.firstinspires.ftc.teamcode.subSystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    private final DcMotorEx intakeMotor;
    private final Servo intakeServo;

    public IntakeSubsystem(HardwareMap hm) {
        intakeMotor = hm.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intakeServo = hm.get(Servo.class, "intakeServo");
    }

    public void setPower(double power) {
        intakeMotor.setPower(power);
    }

    public double getPower() {
        return intakeMotor.getPower();
    }

    public void servoPos(double pos) {
        intakeServo.setPosition(pos);
    }
}
