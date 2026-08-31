package org.firstinspires.ftc.teamcode.OpModes;

import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.HardwareConfig;
import org.firstinspires.ftc.teamcode.commands.ArmCmds;
import org.firstinspires.ftc.teamcode.commands.DriveCmds;
import org.firstinspires.ftc.teamcode.subSystems.ArmSub;
import org.firstinspires.ftc.teamcode.subSystems.DriveSub;
import org.firstinspires.ftc.teamcode.subSystems.IntakeSub;

@TeleOp(name = "BioBuzzOpMode", group = "Examples")
public class SimpleDriveIntake extends CommandOpMode {

    private HardwareConfig hm;
    private DriveSub driveSub;
    private IntakeSub intakeSub;
    private ArmSub armSub;
    private GamepadEx gamepad;

    @Override
    public void initialize() {
        hm = new HardwareConfig();
        hm.init(hardwareMap);

        driveSub = new DriveSub(hardwareMap);
        intakeSub = new IntakeSub(hm);
        armSub = new ArmSub(hm);
        this.gamepad = new GamepadEx(gamepad1);

        armSub.setDefaultCommand(new ArmCmds.ArmHoldPosition(armSub));

        driveSub.setDefaultCommand(new DriveCmds.TeleopDrive(driveSub, gamepad));

        new GamepadButton(gamepad, GamepadKeys.Button.DPAD_UP)
                .whenPressed(new ArmCmds.ArmGoToAngle(armSub, 90.0));

        new GamepadButton(gamepad, GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new ArmCmds.ArmGoToAngle(armSub, 0.0));

        new GamepadButton(gamepad, GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new ArmCmds.ArmGoToAngle(armSub, 180.0));

        schedule(new RunCommand(() -> {
            telemetry.addData("Arm Angle (°)", String.format("%.1f", armSub.getAngle()));
            telemetry.addData("Arm Power", String.format("%.1f", armSub.armMotor.getPower()));


            // Fixed type check: check the subsystem's active command, not the subsystem class itself
            if (armSub.getCurrentCommand() != null &&
                    armSub.getCurrentCommand().getClass().getSimpleName().equals("ArmGoToAngle")) {
                telemetry.addData("Arm Target (°)", "set via DPAD");
            }
            telemetry.update();
        }));
    }
}
