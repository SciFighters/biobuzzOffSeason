package org.firstinspires.ftc.teamcode.OpModes;

import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.HardwareConfig;

import org.firstinspires.ftc.teamcode.subSystems.DriveSub;
import org.firstinspires.ftc.teamcode.subSystems.IntakeSub;
import org.firstinspires.ftc.teamcode.subSystems.LiftSub;
import org.firstinspires.ftc.teamcode.subSystems.BoxSub;

@TeleOp(name = "Fuze 0.5", group = "Fuze")
public class SimpleDriveIntake extends CommandOpMode {

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

        liftSub.setDefaultCommand(new LiftCmds.holdHeight(liftSub));

        driveSub.setDefaultCommand(
                new RunCommand(
                        () -> driveSub.drive(
                                gamepad.getLeftX(),
                                gamepad.getLeftY(),
                                gamepad.getRightX()
                        ),
                        driveSub
                )
        );

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
