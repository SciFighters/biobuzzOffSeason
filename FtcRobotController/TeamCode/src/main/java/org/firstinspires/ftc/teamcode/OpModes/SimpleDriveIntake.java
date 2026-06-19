package org.firstinspires.ftc.teamcode.OpModes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.HardwareConfig;
import org.firstinspires.ftc.teamcode.commands.ArmCmds;
import org.firstinspires.ftc.teamcode.commands.IntakeCmds;
import org.firstinspires.ftc.teamcode.subSystems.ArmSub;
import org.firstinspires.ftc.teamcode.subSystems.DriveSub;
import org.firstinspires.ftc.teamcode.subSystems.IntakeSub;

import org.firstinspires.ftc.teamcode.Utilities.pid.PIDConfig;
import org.firstinspires.ftc.teamcode.Utilities.pid.PIDController;

@TeleOp(name = "BioBuzzOpMode", group = "Examples")
public class SimpleDriveIntake extends CommandOpMode {

    private HardwareConfig hm;
    private DriveSub driveSub;
    private IntakeSub intakeSub;
    private ArmSub armSub;

    @Override
    public void initialize() {

        hm = new HardwareConfig();
        hm.init(hardwareMap);

        driveSub = new DriveSub(hm);
        intakeSub = new IntakeSub(hm);
        armSub = new ArmSub(hm);
        armSub.setDefaultCommand(new ArmCmds.ArmHoldPosition(armSub));

        GamepadEx gamepad = new GamepadEx(gamepad1);

        // Set default command for drive: field-centric control
        driveSub.setDefaultCommand(
                new RunCommand(
                        () -> driveSub.drive(
                                gamepad.getLeftX(),
                                -gamepad.getLeftY(),
                                gamepad.getRightX()
                        ),
                        driveSub
                )
        );

        // Intake control via D-pad
        new GamepadButton(gamepad, GamepadKeys.Button.DPAD_UP)
                .whileHeld(new ArmCmds.ArmGoToAngle(armSub, 90.0));

        new GamepadButton(gamepad, GamepadKeys.Button.DPAD_LEFT)
                .whileHeld(new ArmCmds.ArmGoToAngle(armSub, 0.0));

        new GamepadButton(gamepad, GamepadKeys.Button.DPAD_RIGHT)
                .whileHeld(new ArmCmds.ArmGoToAngle(armSub, 180.0));
    }
}