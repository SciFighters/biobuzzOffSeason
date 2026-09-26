package org.firstinspires.ftc.teamcode.OpModes.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Utilities.ActionOpMode;
import org.firstinspires.ftc.teamcode.subSystems.IntakeCamera;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.opencv.core.Point;

import java.util.List;

@TeleOp(name = "CameraTester", group = "Test")
public class CameraTest extends ActionOpMode {
    private IntakeCamera camera;

    @Override
    public void initialize() {
        camera = new IntakeCamera(hardwareMap);
        camera.scanBlobs(ColorRange.YELLOW);
        camera.scanBlobs(ColorRange.RED);
        camera.scanBlobs(ColorRange.BLUE);
    }

    @Override
    public void run() {
        super.run();
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

    @Override
    public void end() {
        super.stop();
        if (camera != null) camera.close();
    }

}
