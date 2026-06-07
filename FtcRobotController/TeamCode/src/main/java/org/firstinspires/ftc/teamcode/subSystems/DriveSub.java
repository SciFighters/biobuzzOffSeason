package org.firstinspires.ftc.teamcode.subSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.HardwareProvider;

// Drive subsystem for mecanum wheels
public class DriveSub {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor rearLeft;
    private DcMotor rearRight;

    public DriveSub() {
        frontLeft = HardwareConfig.frontLeft;
        frontRight = HardwareConfig.frontRight;
        rearLeft = HardwareConfig.rearLeft;
        rearRight = HardwareConfig.rearRight;
    }

    // Mecanum drive calculation
    // @param xSpeed   Left/right (-1 left, 1 right)
    // @param ySpeed   Forward/backward (-1 back, 1 forward)
    // @param rotSpeed Rotation (-1 CW, 1 CCW)
    public void drive(double xSpeed, double ySpeed, double rotSpeed) {
        //  wheel powers
        double leftFrontPower = ySpeed + xSpeed + rotSpeed;
        double rightFrontPower = ySpeed - xSpeed + rotSpeed;
        double leftRearPower = ySpeed - xSpeed - rotSpeed;
        double rightRearPower = ySpeed + xSpeed - rotSpeed;

        // Normalize
        double max = Math.max(
            Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)),
            Math.max(Math.abs(leftRearPower), Math.abs(rightRearPower))
        );
        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftRearPower /= max;
            rightRearPower /= max;
        }

        frontLeft.setPower(leftFrontPower);
        frontRight.setPower(rightFrontPower);
        rearLeft.setPower(leftRearPower);
        rearRight.setPower(rightRearPower);
    }
}
