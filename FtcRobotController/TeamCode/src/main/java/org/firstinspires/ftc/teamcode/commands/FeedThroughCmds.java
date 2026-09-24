package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.InstantCommand;
import org.firstinspires.ftc.teamcode.subSystems.FeedThroughSub;

public class FeedThroughCmds {
    public static class FeedThrough extends CommandBase {
        private final FeedThroughSub feedThrough;

        public FeedThrough(FeedThroughSub feedThrough) {
            this.feedThrough = feedThrough;
            addRequirements(feedThrough);
        }

        @Override
        public void execute() {
            feedThrough.setPower(1.0);
        }

        @Override
        public void end(boolean interrupted) {
            feedThrough.setPower(0.0);
        }
    }

    public static class FeedThroughEject extends CommandBase {
        private final FeedThroughSub feedThrough;

        public FeedThroughEject(FeedThroughSub feedThrough) {
            this.feedThrough = feedThrough;
            addRequirements(feedThrough);
        }

        @Override
        public void execute() {
            feedThrough.setPower(-1.0);
        }

        @Override
        public void end(boolean interrupted) {
            feedThrough.setPower(0.0);
        }
    }

    public static class StopFeedThrough extends InstantCommand {
        public StopFeedThrough(FeedThroughSub feedThrough) {
            super(() -> feedThrough.setPower(0.0), feedThrough);
        }
    }

    public static class OpenGate extends InstantCommand {
        public OpenGate(FeedThroughSub feedThrough) {
            // uncalibrated value
            super(() -> feedThrough.servoPos(1.0), feedThrough);
        }
    }

    public static class CloseGate extends InstantCommand {
        public CloseGate(FeedThroughSub feedThrough) {
            // uncalibrated value
            super(() -> feedThrough.servoPos(0.0), feedThrough);
        }
    }
}
