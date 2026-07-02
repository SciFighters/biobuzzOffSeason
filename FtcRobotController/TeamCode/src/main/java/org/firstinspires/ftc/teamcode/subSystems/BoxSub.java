package org.firstinspires.ftc.teamcode.subSystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.HardwareConfig;

// Box discharge subsystem
public class BoxSub extends SubsystemBase {

    private Servo leftServo;
    private Servo rightServo;

    public BoxSub(HardwareConfig hm) {
        leftServo = hm.leftServo;
        rightServo = hm.rightServo;
    }

    // Set left servo position
    // @param position 0.0 to 1.0
    public void setLeftServoPosition(double position) {
        leftServo.setPosition(position);
    }

    // Get left servo position
    // @return current position
    public double getLeftServoPosition() {
        return leftServo.getPosition();
    }

    // Set right servo position
    // @param position 0.0 to 1.0
    public void setRightServoPosition(double position) {
        rightServo.setPosition(position);
    }

    // Get right servo position
    // @return current position
    public double getRightServoPosition() {
        return rightServo.getPosition();
    }
}
