package org.firstinspires.ftc.teamcode.OpModes.tests;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import org.firstinspires.ftc.teamcode.Utilities.ActionOpMode;
import org.firstinspires.ftc.teamcode.commands.IntakeCameraCommands;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subSystems.IntakeCameraSubsystem;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.opencv.core.Rect;

import java.util.List;

@TeleOp(name = "CameraTester", group = "Test")
public class CameraTest extends ActionOpMode {
    private IntakeCameraSubsystem camera;
    GamepadEx gamepad;
    Follower follower;
    private IntakeCameraCommands.ElementFocus focus;

    @Override
    public void initialize() {
        follower = Constants.create(hardwareMap);
        follower.manual(0, 0, 0);
        camera = new IntakeCameraSubsystem(hardwareMap);
        gamepad = new GamepadEx(gamepad1);
        focus = new IntakeCameraCommands.ElementFocus(follower, camera, ColorRange.YELLOW);
        IntakeCameraCommands.ElementLockIn lockIn = new IntakeCameraCommands.ElementLockIn(follower, camera, ColorRange.YELLOW);
        IntakeCameraCommands.ElementPathFinder pathFinder = new IntakeCameraCommands.ElementPathFinder(follower, camera, ColorRange.YELLOW);
        new GamepadButton(gamepad, GamepadKeys.Button.A).whenPressed(() -> {
            pathFinder.cancel();
            focus.schedule();
        });
        new GamepadButton(gamepad, GamepadKeys.Button.B).whenPressed(() -> {
            focus.cancel();
            pathFinder.schedule();

        });


        follower.setPose(new Pose((141.5-9), 9, Math.PI / 2.0));
    }

    @Override
    public void run() {
        super.run();
        DrivePowers powers = ManualDrive.fieldCentric(
                gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x * -1,
                follower.pose().heading()
        );

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
        List<Rect> blobs = camera.scanBlobsByDistance(ColorRange.YELLOW);
        double[] locations = {Double.NaN, Double.NaN};
        for (Rect blob : blobs) {
            if (blob.width > 0 && blob.height > 0) {
                locations = camera.getElementLocation(follower, blob);
                break;
            }
        }
        Pose current = follower.pose();
        double distance = Math.hypot(locations[0] - current.x(), locations[1] - current.y());

        multipleTelemetry.addLine("camera:");
        multipleTelemetry.addData("State", camera.getCameraState());
        multipleTelemetry.addData("FPS", "%.1f", camera.getFps());
        multipleTelemetry.addData("Yellow blobs", blobs.size());
        multipleTelemetry.addData("Focus active", focus.isScheduled());
        multipleTelemetry.addData("Target angle (deg)", blobs.isEmpty()
                ? Double.NaN : Math.toDegrees(camera.getXAngleOffset(blobs.get(0))));

        multipleTelemetry.addLine("position:");
        multipleTelemetry.addData("Position (in)", "X: %.1f | Y: %.1f", current.x(), current.y());
        multipleTelemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(current.heading()));

        multipleTelemetry.addLine("pollen:");
        multipleTelemetry.addData("Target position (in)", "X: %.1f | Y: %.1f", locations[0], locations[1]);
        multipleTelemetry.addData("Distance (in)", "%.1f", distance);

        multipleTelemetry.update();
    }
}
