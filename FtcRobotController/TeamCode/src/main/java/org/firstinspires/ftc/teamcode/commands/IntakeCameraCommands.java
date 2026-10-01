package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.utils.Angle;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import org.firstinspires.ftc.teamcode.subSystems.IntakeCameraSubsystem;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.opencv.core.Rect;

import java.util.List;

@Config
public class IntakeCameraCommands {
    public static class ElementFocus extends CommandBase {
        private final Follower follower;
        private final IntakeCameraSubsystem camera;
        private final ColorRange color;
        Rect blob;

        PIDController pidController = new PIDController(0.01,0,0);

        public ElementFocus(Follower follower, IntakeCameraSubsystem camera, ColorRange color) {
            this.follower = follower;
            this.camera = camera;
            this.color = color;
        }

        @Override
        public void initialize() {
            follower.manual(0, 0, 0);
            pidController.setTolerance(1.0);
            pidController.reset();
        }

        @Override
        public void execute() {
            List<Rect> blobs = camera.scanBlobs(color);
            if (blobs.isEmpty()) {
                follower.manual(0, 0, 0);
                pidController.reset();
                return;
            }

            blob = blobs.get(0);
            pidController.setSetPoint(camera.getXAngleOffset(blob));
            double currentYaw = follower.pose().heading();

            follower.manual(0,0, pidController.calculate(currentYaw));
            follower.update();
        }

        @Override
        public void end(boolean interrupted) {
            follower.manual(0, 0, 0);
        }
    }

    public static class ElementLockIn extends CommandBase {
        private final Follower follower;
        private final IntakeCameraSubsystem camera;
        private final ColorRange color;
        private double targetHeading = Double.NaN;
        PIDController pidController = new PIDController(0.01, 0, 0);

        public ElementLockIn(Follower follower, IntakeCameraSubsystem camera, ColorRange color) {
            this.follower = follower;
            this.camera = camera;
            this.color = color;
            camera.scanBlobs(color);
        }

        @Override
        public void initialize() {
            targetHeading = Double.NaN;
            follower.manual(0, 0, 0);
            pidController.reset();
            pidController.setTolerance(1.0);
            pidController.setSetPoint(0);
            List<Rect> blobs = camera.scanBlobs(color);
            if (blobs.isEmpty()) return;
            Rect blob = blobs.get(0);
            double angle = camera.getXAngleOffset(blob);
            targetHeading = Angle.normalize(follower.pose().heading() + angle);
        }

        @Override
        public void execute() {
            double error = Angle.error(follower.pose().heading(), targetHeading);
            double yaw = pidController.calculate(Math.toDegrees(error));
            follower.manual(0, 0, yaw);
            follower.update();
        }

        @Override
        public boolean isFinished() {
            return pidController.atSetPoint();
        }

        @Override
        public void end(boolean interrupted) {
            follower.manual(0, 0, 0);
            follower.update();
        }
    }

    public static class ElementPathFinder extends CommandBase {
        private final Follower follower;
        private final IntakeCameraSubsystem camera;
        private final ColorRange color;
        private FollowPathCommand pathCommand;

        public ElementPathFinder(Follower follower, IntakeCameraSubsystem camera, ColorRange color) {
            this.follower = follower;
            this.camera = camera;
            this.color = color;
            camera.scanBlobs(color);
        }
    }
}
