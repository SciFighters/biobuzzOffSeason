package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.subSystems.DriveSub;

public final class DriveCmds {
    private DriveCmds() {}

    public static class FollowPath extends CommandBase {
        private final DriveSub drive;
        private final PathChain path;

        public FollowPath(DriveSub drive, PathChain path) {
            this.drive = drive;
            this.path = path;
            addRequirements(drive);
        }

        @Override
        public void initialize() {
            drive.followPath(path);
        }

        @Override
        public void end(boolean interrupted) {
            if (interrupted) drive.stop();
        }

        @Override
        public boolean isFinished() {
            return !drive.isBusy();
        }
    }

    public static class TeleopDrive extends CommandBase {
        private final DriveSub drive;
        private final GamepadEx gamepad;

        public TeleopDrive(DriveSub drive, GamepadEx gamepad) {
            this.drive = drive;
            this.gamepad = gamepad;
            addRequirements(drive);
        }

        @Override
        public void initialize() {
            drive.startTeleopDrive();
        }

        @Override
        public void execute() {
            drive.setTeleopDrive(
                    -gamepad.getLeftY(),
                    -gamepad.getLeftX(),
                    -gamepad.getRightX(),
                    true
            );
        }
    }
}
