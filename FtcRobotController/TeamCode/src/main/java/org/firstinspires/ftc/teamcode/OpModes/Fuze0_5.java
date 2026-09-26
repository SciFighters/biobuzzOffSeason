package org.firstinspires.ftc.teamcode.OpModes;

import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Utilities.ActionOpMode;
import org.firstinspires.ftc.teamcode.commands.DriveCommands;
import org.firstinspires.ftc.teamcode.subSystems.DriveSubsystem;

@TeleOp(name = "Fuze 0.5", group = "Fuze")
public class Fuze0_5 extends ActionOpMode {
    DriveSubsystem driveSub;

    GamepadEx gamepad;

    @Override
    public void initialize() {


        driveSub = new DriveSubsystem(hardwareMap);

        this.gamepad = new GamepadEx(gamepad1);


        driveSub.setDefaultCommand(new DriveCommands.TeleopDrive(driveSub, gamepad));


       schedule(new RunCommand(() -> {
            telemetry.update();
        }));
    }
}
