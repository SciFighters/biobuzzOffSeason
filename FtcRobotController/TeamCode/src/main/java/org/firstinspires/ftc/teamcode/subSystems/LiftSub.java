package org.firstinspires.ftc.teamcode.subSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.HardwareConfig;

// Lift subsystem
public class LiftSub extends SubsystemBase {

    private DcMotorEx leftLift;
    private DcMotorEx rightLift;

    public LiftSub(HardwareConfig hm) {
        leftLift = hm.leftLift;
        rightLift = hm.rightLift;
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
        leftLift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightLift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }

    // Get lift position
    // @return average position
    public int getPosition() {
        return (
            (leftLift.getCurrentPosition() + rightLift.getCurrentPosition()) / 2
        );
    }
}
