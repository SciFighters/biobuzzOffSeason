package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Utilities.GobildaPlanetery;
import org.firstinspires.ftc.teamcode.Utilities.MotorOut;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class HardwareConfig {

    public Servo leftServo;
    public Servo rightServo;
    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx rearLeft;
    public DcMotorEx rearRight;
    public MotorOut intakeMotor;
    public MotorOut leftLift;
    public MotorOut rightLift;
    public MotorOut armMotor;
    public IMU imu;

    public void init(HardwareMap hardwareMap) {
        // Drive motors
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        rearLeft = hardwareMap.get(DcMotorEx.class, "rearLeft");
        rearRight = hardwareMap.get(DcMotorEx.class, "rearRight");

        // Set directions
        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        rearLeft.setDirection(DcMotorEx.Direction.REVERSE);
        frontRight.setDirection(DcMotorEx.Direction.FORWARD);
        rearRight.setDirection(DcMotorEx.Direction.FORWARD);

        // zero power behavior
        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        rearLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        rearRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        // Box servos
        //leftServo = hardwareMap.get(Servo.class, "leftServo");
        //rightServo = hardwareMap.get(Servo.class, "rightServo");

        // Intake motor
        this.intakeMotor = new MotorOut((hardwareMap.get(DcMotorEx.class, "intakeMotor")), GobildaPlanetery.RPM1150, 1);




         // Lift motors
         //this.leftLift = new MotorOut((hardwareMap.get(DcMotorEx.class, "leftLift")), 168, 1000);
         //this.rightLift = new MotorOut((hardwareMap.get(DcMotorEx.class, "rightLift")), 168, 1000);

        // Lift motor directions

        // Arm motor
        this.armMotor = new MotorOut((hardwareMap.get(DcMotorEx.class, "armMotor")), GobildaPlanetery.RPM60, 1);
        // IMU
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT)));
    }
}