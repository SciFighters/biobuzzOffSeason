package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.subSystems.IntakeCamera;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.opencv.core.Point;

import java.util.List;

@TeleOp(name = "CamTest", group = "Test")
public class CamTest extends CommandOpMode {
    private IntakeCamera camera;

    @Override
    public void initialize() {
        telemetry.setMsTransmissionInterval(100);
        camera = new IntakeCamera(hardwareMap);
        camera.scanBlobs(ColorRange.YELLOW);
        camera.scanBlobs(ColorRange.RED);
        camera.scanBlobs(ColorRange.BLUE);
    }

    @Override
    public void run() {
        super.run();
        telemetry.clearAll();
        telemetry.addData("Camera", camera.getCameraState());
        telemetry.addData("FPS", "%.1f", camera.getFps());

        List<Point> yellow = camera.scanBlobs(ColorRange.YELLOW);
        telemetry.addData("Yellow count", yellow.size());        
        for (int i = 0; i < yellow.size(); i++) {
            Point center = yellow.get(i);
            telemetry.addData("Yellow blob " + (i + 1), "x=%.1f, y=%.1f", center.x, center.y);
        }

        List<Point> red = camera.scanBlobs(ColorRange.RED);
        telemetry.addData("Red count", red.size());
        for (int i = 0; i < red.size(); i++) {
            Point center = red.get(i);
            telemetry.addData("Red blob " + (i + 1), "x=%.1f, y=%.1f", center.x, center.y);
        }

        List<Point> blue = camera.scanBlobs(ColorRange.BLUE);
        telemetry.addData("Blue count", blue.size());
        for (int i = 0; i < blue.size(); i++) {
            Point center = blue.get(i);
            telemetry.addData("Blue blob " + (i + 1), "x=%.1f, y=%.1f", center.x, center.y);
        }
        telemetry.update();
    }

    @Override
    public void stop() {
        super.stop();
        if (camera != null) camera.close();
    }

}
