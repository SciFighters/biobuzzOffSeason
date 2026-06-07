package org.firstinspires.ftc.teamcode.subSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.HardwareConfig;

// Drive subsystem for mecanum wheels
public class DriveSub extends SubsystemBase {

    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx rearLeft;
    private DcMotorEx rearRight;

    public DriveSub(HardwareConfig hm) {
        frontLeft = hm.frontLeft;
        frontRight = hm.frontRight;
        rearLeft = hm.rearLeft;
        rearRight = hm.rearRight;
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
