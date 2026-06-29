package org.firstinspires.ftc.teamcode.subSystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;
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
        // Get current robot heading (yaw) in radians
        double heading = 0.0;
        if (imu != null) {
            Orientation angles = imu.getRobotOrientation(
                    AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            heading = angles.firstAngle; // yaw
        }

        // Rotate joystick inputs relative to robot's heading to achieve field‑oriented control
        double cosH = Math.cos(-heading);
        double sinH = Math.sin(-heading);

        double rotX = xSpeed * cosH - ySpeed * sinH;          // forward/backward component
        double rotY = xSpeed * sinH + ySpeed * cosH;          // strafe component

        // Apply driver throttle and heading‑relative turn
        double frontLeftPower  = rotY + rotX + rotSpeed;
        double frontRightPower = rotY - rotX - rotSpeed;
        double rearLeftPower   = rotY - rotX + rotSpeed;
        double rearRightPower  = rotY + rotX - rotSpeed;

        // Clip to [-1, 1] and scale down if any wheel exceeds 1.0
        double max = Math.max(
                Math.abs(frontLeftPower),
                Math.max(
                        Math.abs(frontRightPower),
                        Math.max(
                                Math.abs(rearLeftPower),
                                Math.abs(rearRightPower)
                        )
                )
        );
        if (max > 1.0) {
            frontLeftPower  /= max;
            frontRightPower /= max;
            rearLeftPower   /= max;
            rearRightPower  /= max;
        }

        // Send power to motors
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        rearLeft.setPower(rearLeftPower);
        rearRight.setPower(rearRightPower);
    }
}