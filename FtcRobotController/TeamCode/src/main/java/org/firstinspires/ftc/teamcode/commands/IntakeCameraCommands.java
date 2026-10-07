package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.pedropathing.utils.Angle;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import org.firstinspires.ftc.teamcode.subSystems.IntakeCameraSubsystem;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.opencv.core.Rect;

import java.util.List;

public class IntakeCameraCommands {
    public static class ElementFocus extends CommandBase {
        private final Follower follower;
        private final IntakeCameraSubsystem camera;
        private final ColorRange color;

        private final PIDController pidController = new PIDController(0.01 * 180.0 / Math.PI, 0, 0);

        public ElementFocus(Follower follower, IntakeCameraSubsystem camera, ColorRange color) {
            this.follower = follower;
            this.camera = camera;
            this.color = color;
            pidController.setTolerance(Math.toRadians(1.0));
            camera.scanBlobs(color);
        }

        @Override
        public void initialize() {
            follower.manual(0, 0, 0);
            pidController.reset();
        }

        @Override
        public void execute() {
            List<Rect> blobs = camera.scanBlobsByDistance(color);
            double turnPower = 0;
            if (blobs.isEmpty()) {
                pidController.reset();
            } else {
                turnPower = pidController.calculate(0, camera.getXAngleOffset(blobs.get(0)));
                if (pidController.atSetPoint()) turnPower = 0;
                turnPower = Math.max(-1.0, Math.min(1.0, turnPower));
            }
            follower.manual(0, 0, turnPower);
            follower.update();
        }

        @Override
        public void end(boolean interrupted) {
            follower.manual(0, 0, 0);
            follower.update();
        }
    }

    public static class ElementLockIn extends CommandBase {
        private final Follower follower;
        private final IntakeCameraSubsystem camera;
        private final ColorRange color;
        private double targetHeading = Double.NaN;
        private final PIDController pidController = new PIDController(0.01 * 180.0 / Math.PI, 0, 0);

        public ElementLockIn(Follower follower, IntakeCameraSubsystem camera, ColorRange color) {
            this.follower = follower;
            this.camera = camera;
            this.color = color;
            pidController.setTolerance(Math.toRadians(1.0));
            camera.scanBlobs(color);
        }

        @Override
        public void initialize() {
            follower.manual(0, 0, 0);
            pidController.reset();
            List<Rect> blobs = camera.scanBlobsByDistance(color);
            targetHeading = blobs.isEmpty() ? Double.NaN
                    : follower.pose().heading() + camera.getXAngleOffset(blobs.get(0));
        }

        @Override
        public void execute() {
            double turnPower = 0;
            if (!Double.isNaN(targetHeading)) {
                double error = Angle.error(follower.pose().heading(), targetHeading);
                turnPower = pidController.calculate(0, error);
                turnPower = Math.max(-1.0, Math.min(1.0, turnPower));
            }
            follower.manual(0, 0, turnPower);
            follower.update();
        }

        @Override
        public boolean isFinished() {
            return Double.isNaN(targetHeading) || pidController.atSetPoint();
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

        @Override
        public void initialize() {
            follower.manual(0, 0, 0);
            follower.update();

            Path path = camera.getElementsPath(follower, color);
            pathCommand = path == null ? null : new FollowPathCommand(follower, path);
            if (pathCommand != null) pathCommand.initialize();
        }

        @Override
        public void execute() {
            follower.update();
        }

        @Override
        public boolean isFinished() {
            return pathCommand == null || pathCommand.isFinished();
        }

        @Override
        public void end(boolean interrupted) {
            follower.manual(0, 0, 0);
            if (pathCommand != null) pathCommand.end(interrupted);
            follower.update();
        }
    }
}
