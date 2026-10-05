package org.firstinspires.ftc.teamcode.subSystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

public class DischargeSubsystem extends SubsystemBase {
    private final Motor flyWheelMotor;
    private final DcMotorEx turretMotor;
    private final Servo hoodServo;
    public DischargeSubsystem(HardwareMap hm){
        flyWheelMotor = new Motor(hm,"flyWheel");
        turretMotor = hm.get(DcMotorEx.class,"turret");
        hoodServo = hm.get(Servo.class,"hoodServo");
    }
    public void setRPM(double rpm){
        if (Math.abs(rpm) < 500){
            if(flyWheelMotor.get() == 0){
                return;
            }
            flyWheelMotor.set(0);
            return;
        }
    }

}
