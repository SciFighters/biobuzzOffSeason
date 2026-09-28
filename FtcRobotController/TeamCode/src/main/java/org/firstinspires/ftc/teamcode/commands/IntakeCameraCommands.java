package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subSystems.IntakeCamera;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.opencv.core.Point;
import org.opencv.core.Rect;

import java.util.List;

public class IntakeCameraCommands {
    public static class ElementFocus extends CommandBase {
        private final Follower follower;
        IntakeCamera camera;
        double posX;

        public ElementFocus(Follower follower, IntakeCamera camera) {
            this.follower = follower;
            this.camera = camera;

        }

        @Override
        public void execute() {
            double center = 640 / 2.0; // width resolution / 2
            List<Point> yellow = camera.scanBlobs(ColorRange.YELLOW);
            if (!yellow.isEmpty()) {
                posX = yellow.get(0).x;
            } else {posX = center;
            }
            double offset = center - posX;
            double degrees = offset * 0.001;
            follower.manual(0,0,degrees);
        }
    }

    public static class ElementLockIn extends InstantCommand {
        private final Follower follower;
        private final IntakeCamera camera;
        public ElementLockIn(Follower follower, IntakeCamera camera) {
            this.follower = follower;
            this.camera = camera;
        }

        @Override
        public void execute() {
            List<Rect> blobs = camera.scanBlobBounds(ColorRange.YELLOW);
            if (blobs.isEmpty()) return;

            Rect blob = blobs.get(0);
            double blobWidth = blob.width;
            if (blobWidth <= 0) return;

            double posX = blob.x + blobWidth / 2.0;
            double opticalCenterX = 320;
            double focalLength = 622; // pixels at 640x480
            double offset = opticalCenterX - posX;
            double angle = Math.atan2(offset, focalLength);

            Pose current = follower.pose();
            follower.hold(new Pose(
                current.x(),
                current.y(),
                current.heading() + angle
            ));
        }
    }
}
