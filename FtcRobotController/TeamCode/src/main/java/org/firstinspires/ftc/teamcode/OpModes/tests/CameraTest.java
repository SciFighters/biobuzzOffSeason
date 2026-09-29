package org.firstinspires.ftc.teamcode.OpModes.tests;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Utilities.ActionOpMode;
import org.firstinspires.ftc.teamcode.commands.IntakeCameraCommands;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subSystems.IntakeCameraSubsystem;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.opencv.core.Point;

import java.util.List;

@TeleOp(name = "CameraTester", group = "Test")
public class CameraTest extends ActionOpMode {
    private IntakeCameraSubsystem camera;
    GamepadEx gamepad;
    Follower follower;

    @Override
    public void initialize() {
        follower = Constants.create(hardwareMap);
        follower.manual(0, 0, 0);
        camera = new IntakeCameraSubsystem(hardwareMap);
        gamepad = new GamepadEx(gamepad1);
        IntakeCameraCommands.ElementFocus focus = new IntakeCameraCommands.ElementFocus(follower, camera, ColorRange.YELLOW);
        IntakeCameraCommands.ElementLockIn lockIn = new IntakeCameraCommands.ElementLockIn(follower, camera, ColorRange.YELLOW);
        new GamepadButton(gamepad, GamepadKeys.Button.A).toggleWhenPressed(focus);
        new GamepadButton(gamepad, GamepadKeys.Button.B).whenPressed(() -> {
            focus.cancel();
            lockIn.schedule();
        });

    }

    @Override
    public void run() {
        super.run();


        telemeter();
        follower.update();
    }

    @Override
    public void end() {
        super.end();
        if (follower != null) {
            follower.manual(0, 0, 0);
            follower.update();
        }
        if (camera != null) camera.close();
    }

    public void telemeter() {
        List<Double> distances = camera.getDistance(ColorRange.YELLOW);
        multipleTelemetry.addData("distance",
                distances.isEmpty() ? Double.NaN : distances.get(0));

        multipleTelemetry.addData("Camera", camera.getCameraState());
        multipleTelemetry.addData("FPS", "%.1f", camera.getFps());

        List<Point> yellow = camera.scanBlobs(ColorRange.YELLOW);
        multipleTelemetry.addData("Yellow count", yellow.size());
        for (int i = 0; i < yellow.size(); i++) {
            Point center = yellow.get(i);
            multipleTelemetry.addData("Yellow blob " + (i + 1), "x=%.1f, y=%.1f", center.x, center.y);
        }

//        List<Point> red = camera.scanBlobs(ColorRange.RED);
//        multipleTelemetry.addData("Red count", red.size());
//        for (int i = 0; i < red.size(); i++) {
//            Point center = red.get(i);
//            multipleTelemetry.addData("Red blob " + (i + 1), "x=%.1f, y=%.1f", center.x, center.y);
//        }
//
//        List<Point> blue = camera.scanBlobs(ColorRange.BLUE);
//        multipleTelemetry.addData("Blue count", blue.size());
//        for (int i = 0; i < blue.size(); i++) {
//            Point center = blue.get(i);
//            multipleTelemetry.addData("Blue blob " + (i + 1), "x=%.1f, y=%.1f", center.x, center.y);
//        }
        multipleTelemetry.update();
    }
}
