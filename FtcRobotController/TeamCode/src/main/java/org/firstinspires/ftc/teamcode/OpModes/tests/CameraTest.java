package org.firstinspires.ftc.teamcode.OpModes.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.Utilities.ActionOpMode;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subSystems.IntakeCamera;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.opencv.core.Point;

import java.util.List;

@TeleOp(name = "CameraTester", group = "Test")
public class CameraTest extends ActionOpMode {
    private IntakeCamera camera;
    private Follower follower;
    GamepadEx gamepad;

    @Override
    public void initialize() {
        follower = Constants.create(hardwareMap);
        follower.manual(0, 0, 0);
        camera = new IntakeCamera(hardwareMap);
        camera.scanBlobs(ColorRange.YELLOW);
        camera.scanBlobs(ColorRange.RED);
        camera.scanBlobs(ColorRange.BLUE);

        gamepad = new GamepadEx(gamepad1);
    }

    @Override
    public void run() {
        super.run();
        if (gamepad1.a) {
            List<Point> yellow = camera.scanBlobs(ColorRange.YELLOW);
            if (!yellow.isEmpty()) {
                Point center = yellow.get(0);
                elementFocus(center.x);
            } else {
                follower.manual(0, 0, 0);
            }
        }


        telemeter();
        follower.update();
    }

    @Override
    public void end() {
        if (follower != null) {
            follower.manual(0, 0, 0);
            follower.update();
        }
        super.stop();
        if (camera != null) camera.close();
    }

    public void telemeter() {
        multipleTelemetry.addData("Camera", camera.getCameraState());
        multipleTelemetry.addData("FPS", "%.1f", camera.getFps());

        List<Point> yellow = camera.scanBlobs(ColorRange.YELLOW);
        multipleTelemetry.addData("Yellow count", yellow.size());
        for (int i = 0; i < yellow.size(); i++) {
            Point center = yellow.get(i);
            multipleTelemetry.addData("Yellow blob " + (i + 1), "x=%.1f, y=%.1f", center.x, center.y);
        }

        List<Point> red = camera.scanBlobs(ColorRange.RED);
        multipleTelemetry.addData("Red count", red.size());
        for (int i = 0; i < red.size(); i++) {
            Point center = red.get(i);
            multipleTelemetry.addData("Red blob " + (i + 1), "x=%.1f, y=%.1f", center.x, center.y);
        }

        List<Point> blue = camera.scanBlobs(ColorRange.BLUE);
        multipleTelemetry.addData("Blue count", blue.size());
        for (int i = 0; i < blue.size(); i++) {
            Point center = blue.get(i);
            multipleTelemetry.addData("Blue blob " + (i + 1), "x=%.1f, y=%.1f", center.x, center.y);
        }
        multipleTelemetry.update();
    }

    public void elementFocus(double posX) {
        double center = 640 / 2.0; // width resolution / 2
        double offset = center - posX;
        double degrees = Math.abs(offset) > 5
            ? Range.clip(offset * 0.02, -5, 5) : 0;
        Pose current = follower.pose();
        follower.hold(new Pose(
            current.x(),
            current.y(),
            current.heading() + Math.toRadians(degrees)
        ));
    }

//    this func is bad
//    if (me.willPower > 0) {
//        make function find out the distance via size off blob;
//        do trigo to find angle via distance and offset;
//        move that angle;
//
//        currently it just moves gradually until it locks onto the pollen :(
//     }

}
