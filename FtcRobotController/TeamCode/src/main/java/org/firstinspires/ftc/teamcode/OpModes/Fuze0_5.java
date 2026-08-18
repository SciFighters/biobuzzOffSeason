package org.firstinspires.ftc.teamcode.OpModes;

import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.HardwareConfig;

import org.firstinspires.ftc.teamcode.commands.BoxCmds;
import org.firstinspires.ftc.teamcode.commands.IntakeCmds;
import org.firstinspires.ftc.teamcode.commands.LiftCmds;
import org.firstinspires.ftc.teamcode.commands.DriveCmds;
import org.firstinspires.ftc.teamcode.subSystems.DriveSub;
import org.firstinspires.ftc.teamcode.subSystems.IntakeSub;
import org.firstinspires.ftc.teamcode.subSystems.LiftSub;
import org.firstinspires.ftc.teamcode.subSystems.BoxSub;

@TeleOp(name = "Fuze 0.5", group = "Fuze")
public class Fuze0_5 extends CommandOpMode {

    private HardwareConfig hm;

    private DriveSub driveSub;
    private IntakeSub intakeSub;
    private LiftSub liftSub;
    private BoxSub boxSub;

    private GamepadEx gamepad;

    @Override
    public void initialize() {
        hm = new HardwareConfig();
        hm.init(hardwareMap);

        driveSub = new DriveSub(hm);
        intakeSub = new IntakeSub(hm);
        liftSub = new LiftSub(hm);
        boxSub = new BoxSub(hm);

        this.gamepad = new GamepadEx(gamepad1);

        liftSub.setDefaultCommand(new LiftCmds.LiftHoldPosition(liftSub));
        intakeSub.setDefaultCommand(new IntakeCmds.IdleIntake(intakeSub));
        boxSub.setDefaultCommand(new BoxCmds.Close(boxSub));

        driveSub.setDefaultCommand(new DriveCmds.TeleopDrive(driveSub, gamepad));

        new GamepadButton(gamepad, GamepadKeys.Button.DPAD_UP)
                .whenPressed(new LiftCmds.LiftGoToHeight(liftSub, 0.1));

        new GamepadButton(gamepad, GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new LiftCmds.LiftGoToHeight(liftSub, 0.2));

        new GamepadButton(gamepad, GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new LiftCmds.LiftGoToHeight(liftSub, 0.3));

        schedule(new RunCommand(() -> {
            telemetry.addData("Arm Height (mm)", String.format("%.1f", liftSub.getHeightAvg()));
            telemetry.addData("Arm Power", String.format("%.1f", liftSub.getPower()[0]));

            if (liftSub.atTargetHeight()) {
                telemetry.addData("Arm Target", "set via DPAD");
            }
            telemetry.update();
        }));
    }
}
