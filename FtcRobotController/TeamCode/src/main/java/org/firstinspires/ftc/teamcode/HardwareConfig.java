package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Utilities.GobildaPlanetery;
import org.firstinspires.ftc.teamcode.Utilities.MotorOut;

public class HardwareConfig {

    public Servo leftServo;
    public Servo rightServo;
    public MotorOut intakeMotor;
    public MotorOut leftLift;
    public MotorOut rightLift;
    public MotorOut armMotor;
    public void init(HardwareMap hardwareMap) {
        // Box servos
        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");

        // Intake motor
        this.intakeMotor = new MotorOut((hardwareMap.get(DcMotorEx.class, "intakeMotor")), GobildaPlanetery.RPM1150, 1);

        // Lift motors
        this.leftLift = new MotorOut((hardwareMap.get(DcMotorEx.class, "leftLift")), GobildaPlanetery.RPM312, 1000);
        this.rightLift = new MotorOut((hardwareMap.get(DcMotorEx.class, "rightLift")), GobildaPlanetery.RPM312, 1000);


        // Arm motor (deprecated)
        this.armMotor = new MotorOut((hardwareMap.get(DcMotorEx.class, "armMotor")), GobildaPlanetery.RPM60, 1);
    }
}
