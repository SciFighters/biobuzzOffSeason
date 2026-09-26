package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.subSystems.DriveSubsystem;

public final class DriveCommands {
    public static class TeleopDrive extends CommandBase {
        private final DriveSubsystem drive;
        private final GamepadEx gamepad;

        public TeleopDrive(DriveSubsystem drive, GamepadEx gamepad) {
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
                    gamepad.getLeftY(),
                    -gamepad.getLeftX(),
                    -gamepad.getRightX()
            );
        }
    }
}
