package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.List;

public class HardwareConfig {

    public Servo leftServo;
    public Servo rightServo;
    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx rearLeft;
    public DcMotorEx rearRight;
    public DcMotorEx intakeMotor;
    public DcMotorEx leftLift;
    public DcMotorEx rightLift;

    public void init(HardwareMap hardwareMap) {
        // Drive motors
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        rearLeft = hardwareMap.get(DcMotorEx.class, "rearLeft");
        rearRight = hardwareMap.get(DcMotorEx.class, "rearRight");

        // Set directions
        frontLeft.setDirection(DcMotorEx.Direction.FORWARD);
        frontRight.setDirection(DcMotorEx.Direction.FORWARD);
        rearLeft.setDirection(DcMotorEx.Direction.FORWARD);
        rearRight.setDirection(DcMotorEx.Direction.FORWARD);

        // zero power behavior
        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        rearLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        rearRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        // Box servos
        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");

        // Intake motor
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");

        // Lift motors
        leftLift = hardwareMap.get(DcMotorEx.class, "leftLift");
        rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");

        // Lift motor directions
        rightLift.setDirection(DcMotorEx.Direction.REVERSE);
    }
}
