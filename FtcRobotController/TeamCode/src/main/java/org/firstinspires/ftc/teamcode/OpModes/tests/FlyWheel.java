package org.firstinspires.ftc.teamcode.OpModes.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.motors.GoBILDA5202Series;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.annotations.MotorType;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.Utilities.ActionOpMode;
@Configurable
@TeleOp(name = "FlyWheelTester", group = "tests")
public class FlyWheel extends ActionOpMode {
    GamepadEx gamepad;
    GamepadButton A, B, X, Y, up, down;
    DcMotorEx motor;
    CRServo servo;
    final double gearRatio = 2.0/3.0;
    public static double power = 0;
    public static double servoPower = 0;
    @Override
    public void initialize() {
        motor = hardwareMap.get(DcMotorEx.class,"FlyWheelMotor");
//        servo = hardwareMap.get(CRServo.class,"FeederServo");//commented while not using the servo
        gamepad = new GamepadEx(gamepad1);
        A = new GamepadButton(gamepad, GamepadKeys.Button.A);
        B = new GamepadButton(gamepad, GamepadKeys.Button.B);
        X = new GamepadButton(gamepad, GamepadKeys.Button.X);
        Y = new GamepadButton(gamepad, GamepadKeys.Button.Y);
        up = new GamepadButton(gamepad, GamepadKeys.Button.DPAD_UP);
        down = new GamepadButton(gamepad, GamepadKeys.Button.DPAD_DOWN);
        A.whenPressed(() -> power -= 0.01);
        Y.whenPressed(() -> power += 0.01);
        up.whenPressed(() -> power += 0.1);
        down.whenPressed(() -> power -= 0.1);
        B.whenPressed(() -> servoPower += 0.1);
        X.whenPressed(() -> servoPower -= 0.1);
    }

    @Override
    public void run() {
        super.run();
//        servo.setPower(power);//commented while not using the servo
        multipleTelemetry.addLine("up(+) and down(-) for major RPM changes");
        multipleTelemetry.addLine("Y(+) and A(-) for minor RPM changes");
        multipleTelemetry.addLine("B(+) and X(-) for major servo speed changes");
        multipleTelemetry.addData("power",power);
        multipleTelemetry.addData("rpm",motor.getVelocity() / 28 * 60 * gearRatio);
        multipleTelemetry.update();
    }
}
