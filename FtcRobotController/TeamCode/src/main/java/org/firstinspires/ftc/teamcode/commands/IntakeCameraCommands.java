package org.firstinspires.ftc.teamcode.commands;

import static com.pedropathing.api.Paths.line;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.utils.Angle;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.subSystems.IntakeCameraSubsystem;
import org.firstinspires.ftc.vision.opencv.ColorRange;

import java.util.List;

@Config
public class IntakeCameraCommands {
    public static double focusTurnGain = 0.01;
    public static double alignmentToleranceDegrees = 1.0;

    public static class ElementFocus extends CommandBase {
        private final Follower follower;
        private final IntakeCameraSubsystem camera;
        private final ColorRange color;

        public ElementFocus(Follower follower, IntakeCameraSubsystem camera, ColorRange color) {
            this.follower = follower;
            this.camera = camera;
            this.color = color;
            camera.scanBlobs(color);
        }

        @Override
        public void initialize() {
            follower.manual(0, 0, 0);
        }

        @Override
        public void execute() {
            List<Double> angles = camera.getXAnglesOffset(color);
            double angle = angles.isEmpty() ? 0 : angles.get(angles.size() - 1);
            double turn = Math.abs(angle) <= alignmentToleranceDegrees ? 0
                    : Math.max(-1, Math.min(1, angle * focusTurnGain));
            follower.manual(0, 0, turn);
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
            List<Double> angles = camera.getXAnglesOffset(color);
            if (angles.isEmpty()) return;
            Pose current = follower.pose();
            targetHeading = current.heading() + Math.toRadians(angles.get(angles.size() - 1));
            follower.hold(current.withHeading(targetHeading));
            follower.algorithm().reset();
        }

        @Override
        public boolean isFinished() {
            return Double.isNaN(targetHeading)
                    || Math.abs(Angle.error(follower.pose().heading(), targetHeading))
                    <= Math.toRadians(alignmentToleranceDegrees);
        }

        @Override
        public void end(boolean interrupted) {
            if (interrupted) follower.manual(0, 0, 0);
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
            pathCommand = null;
            follower.manual(0, 0, 0);
            if (camera.scanBlobBounds(color).isEmpty()) return;
            double[] location = camera.getElementLocation(follower, color);
            Pose current = follower.pose();
            double heading = Math.atan2(location[1] - current.y(), location[0] - current.x());
            Pose target = new Pose(location[0], location[1], heading);
            pathCommand = new FollowPathCommand(follower,
                    line(current, target).linear(current.heading(), heading));
            pathCommand.initialize();
        }

        @Override
        public boolean isFinished() {
            return pathCommand == null || pathCommand.isFinished();
        }

        @Override
        public void end(boolean interrupted) {
            if (pathCommand != null) pathCommand.end(interrupted);
        }
    }
}
