package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subSystems.IntakeCameraSubsystem;
import org.firstinspires.ftc.vision.opencv.ColorRange;

import java.util.List;

@Config
public class IntakeCameraCommands {
    public static class ElementFocus extends CommandBase {
        private final Follower follower;
        private final IntakeCameraSubsystem camera;
        private final ColorRange color;

        public ElementFocus(Follower follower, IntakeCameraSubsystem camera, ColorRange color) {
            this.follower = follower;
            this.camera = camera;
            this.color = color;
        }

        @Override
        public void initialize() {
            follower.manual(0, 0, 0);
        }

        @Override
        public void execute() {
            List<Double> angles = camera.getXAnglesOffset(color);
            follower.manual(0, 0, angles.isEmpty() ? 0 : angles.get(0) * 0.01);
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

        public ElementLockIn(Follower follower, IntakeCameraSubsystem camera, ColorRange color) {
            this.follower = follower;
            this.camera = camera;
            this.color = color;
        }

        @Override
        public void initialize() {
            follower.manual(0, 0, 0);
            List<Double> angles = camera.getXAnglesOffset(color);
            if (angles.isEmpty()) return;
            Pose current = follower.pose();
            follower.hold(new Pose(
                    current.x(),
                    current.y(),
                    current.heading() + Math.toRadians(angles.get(0))
            ));
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    public static class ElementPathFinder extends InstantCommand {
        public ElementPathFinder(Follower follower, IntakeCameraSubsystem camera, ColorRange color) {

        }
    }
}
