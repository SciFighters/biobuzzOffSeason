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

    public double getServosPosition() {
        return new double[] {leftServo.getPosition(), rightServo.getPosition};
    }

    public void setServosPosition(double pos) {
        rightServo.setPosition(pos);
        leftServo.setPosition(pos)
    }
}
