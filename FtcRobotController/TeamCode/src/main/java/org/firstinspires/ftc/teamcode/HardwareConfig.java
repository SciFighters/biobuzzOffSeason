package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Utilities.GobildaPlanetery;
import org.firstinspires.ftc.teamcode.Utilities.MotorOut;

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
    public GoBildaPinpointDriver pinpoint;

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
        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");

        // Intake motor
        this.intakeMotor = new MotorOut((hardwareMap.get(DcMotorEx.class, "intakeMotor")), GobildaPlanetery.RPM1150, 1);

        // Lift motors
        this.leftLift = new MotorOut((hardwareMap.get(DcMotorEx.class, "leftLift")), GobildaPlanetery.RPM312, 1000);
        this.rightLift = new MotorOut((hardwareMap.get(DcMotorEx.class, "rightLift")), GobildaPlanetery.RPM312, 1000);


        // Arm motor
        this.armMotor = new MotorOut((hardwareMap.get(DcMotorEx.class, "armMotor")), GobildaPlanetery.RPM60, 1);

        // goBilda Pinpoint IMU + odometry
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        configurePinpoint();
    }

    /** tune offsets/directions per robot build */
    private void configurePinpoint() {
        // odometry pod offsets (mm) relative to tracking point
        pinpoint.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        // goBilda 4-bar pods
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        // both pods count forward
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        // start at origin, heading 0
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
        pinpoint.resetPosAndIMU();
    }
}