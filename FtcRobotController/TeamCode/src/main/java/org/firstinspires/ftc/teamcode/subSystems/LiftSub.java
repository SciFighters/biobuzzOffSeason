package org.firstinspires.ftc.teamcode.subSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.HardwareProvider;

// Lift subsystem
public class LiftSub {
    private DcMotor leftLift;
    private DcMotor rightLift;

    public LiftSub() {
        leftLift = HardwareProvider.getDcMotor("leftLift");
        rightLift = HardwareProvider.getDcMotor("rightLift");
        rightLift.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    // Set lift power
    // @param power -1 to 1
    public void setPower(double power) {
        leftLift.setPower(power);
        rightLift.setPower(power);
    }

    // Get lift power
    // @return average power
    public double getPower() {
        return (leftLift.getPower() + rightLift.getPower()) / 2.0;
    }

    // Set lift position
    // @param position target position
    public void setPosition(int position) {
        leftLift.setTargetPosition(position);
        rightLift.setTargetPosition(position);
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // Get lift position
    // @return average position
    public int getPosition() {
        return (leftLift.getCurrentPosition() + rightLift.getCurrentPosition()) / 2;
    }
}