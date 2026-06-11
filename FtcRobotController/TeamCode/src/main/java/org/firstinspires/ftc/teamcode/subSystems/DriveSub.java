package org.firstinspires.ftc.teamcode.subSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.teamcode.HardwareConfig;

public class DriveSub extends SubsystemBase {
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx rearLeft;
    private DcMotorEx rearRight;
    private IMU imu;

    public DriveSub(HardwareConfig hm) {
        frontLeft = hm.frontLeft;
        frontRight = hm.frontRight;
        rearLeft = hm.rearLeft;
        rearRight = hm.rearRight;
        imu = hm.imu;
    }

    public void drive(double xSpeed, double ySpeed, double rotSpeed) {
        double heading = 0.0;
        if (imu != null) {
        Orientation angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        heading = angles.firstAngle; // yaw in radians
        }
        double rotX = xSpeed * Math.cos(-heading) - ySpeed * Math.sin(-heading);
        double rotY = xSpeed * Math.sin(-heading) + ySpeed * Math.cos(-heading);
        double leftFrontPower = rotY + rotX + rotSpeed;
        double rightFrontPower = rotY - rotX - rotSpeed;
        double leftRearPower = rotY - rotX + rotSpeed;
        double rightRearPower = rotY + rotX - rotSpeed;
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