package org.firstinspires.ftc.teamcode.subSystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import kotlin.time.Instant;

public class FeedThroughSub extends SubsystemBase {
    private DcMotorEx transferMotor;
    private Servo gateServo;

    public FeedThroughSub(HardwareMap hm) {
        transferMotor = hm.get(DcMotorEx.class, "transferMotor");
        gateServo = hm.get(Servo.class, "gateServo");
    }

    public void setPower(double power) {
        transferMotor.setPower(power);
    }

    public double getPower() {
        return transferMotor.getPower();
    }

    public void servoPos(double pos) {
        gateServo.setPosition(pos);
    }
//    dan says this good, perhaps?
//    public CommandBase open() {
//        return new InstantCommand(() -> servoPos(1));
//    }
}
