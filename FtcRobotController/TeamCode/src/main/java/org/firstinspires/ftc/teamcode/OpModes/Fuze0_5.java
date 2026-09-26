package org.firstinspires.ftc.teamcode.OpModes;

import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Utilities.ActionOpMode;
import org.firstinspires.ftc.teamcode.commands.DriveCmds;
import org.firstinspires.ftc.teamcode.subSystems.DriveSub;

@TeleOp(name = "Fuze 0.5", group = "Fuze")
public class Fuze0_5 extends ActionOpMode {
    DriveSub driveSub;

    GamepadEx gamepad;

    @Override
    public void initialize() {


        driveSub = new DriveSub(hardwareMap);

        this.gamepad = new GamepadEx(gamepad1);


        driveSub.setDefaultCommand(new DriveCmds.TeleopDrive(driveSub, gamepad));


       schedule(new RunCommand(() -> {
            telemetry.update();
        }));
    }
}
