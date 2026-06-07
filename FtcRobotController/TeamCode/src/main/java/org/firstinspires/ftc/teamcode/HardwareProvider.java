package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.List;

public class HardwareProvider {

    private static HardwareMap hardwareMap;

    public static void init(HardwareMap ahw) {
        hardwareMap = ahw;
    }

    public static Servo getServo(String name) {
        return hardwareMap.get(Servo.class, name);
    }

    public static DcMotor getDcMotor(String name) {
        return hardwareMap.get(DcMotor.class, name);
    }

    public static DcMotorEx getDcMotorEx(String name) {
        return hardwareMap.get(DcMotorEx.class, name);
    }

    public static ColorSensor getColorSensor(String name) {
        return hardwareMap.get(ColorSensor.class, name);
    }

    public static GoBildaPinpointDriver getGoBildaPinpointDriver(String name) {
        return hardwareMap.get(GoBildaPinpointDriver.class, name);
    }

    public static HardwareMap getHardwareMap() {
        return hardwareMap;
    }
}
