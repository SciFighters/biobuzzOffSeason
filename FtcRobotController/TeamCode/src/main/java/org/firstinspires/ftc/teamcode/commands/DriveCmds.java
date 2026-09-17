package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.math.Vector2D;
import com.pedropathing.paths.Path;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.subSystems.DriveSub;

public final class DriveCmds {
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
                    gamepad.getLeftY(),
                    -gamepad.getLeftX(),
                    -gamepad.getRightX()
            );
        }
    }
}
