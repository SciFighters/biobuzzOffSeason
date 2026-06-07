package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareConfig;
import org.firstinspires.ftc.teamcode.subSystems.DriveSub;
import org.firstinspires.ftc.teamcode.subSystems.IntakeSub;

/**
 * Simple OpMode for basic drive and intake motor control
 * Uses left stick for drive and turn
 * Uses right trigger for variable intake speed (forward)
 * Left trigger can be used for reverse intake if needed
 */
@TeleOp(name = "BioBuzzOpMode", group = "Examples")
public class SimpleDriveIntake extends LinearOpMode {

    private HardwareConfig hm;
    private IntakeSub intakeMotor;
    private DriveSub drive;

    @Override
    public void runOpMode() {
        // Initialize hardware
        hm = new HardwareConfig();
        hm.init(hardwareMap);

        intakeMotor = new IntakeSub(hm);
        drive = new DriveSub(hm);

        // Wait for start button
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Drive using left stick
            double leftY = -gamepad1.left_stick_y;  // Invert Y so forward is positive
            double leftX = gamepad1.left_stick_x;   // Left/right
            double rightX = gamepad1.right_stick_x; // Rotation

            drive.drive(leftX, leftY, rightX); // Note: DriveSub.drive(xSpeed, ySpeed, rotSpeed)

            // Intake motor control - right trigger for variable speed forward
            // Left trigger for reverse (if needed)
            double rightTrigger = gamepad1.right_trigger;
            double leftTrigger = gamepad1.left_trigger;
            
            if (rightTrigger > 0.1) {
                intakeMotor.setPower(rightTrigger);  // Variable speed forward based on trigger press
            } else if (leftTrigger > 0.1) {
                intakeMotor.setPower(-leftTrigger);  // Variable speed reverse
            } else {
                intakeMotor.setPower(0.0);  // Intake stop
            }

            // Telemetry
            telemetry.addData("Status", "Running");
            telemetry.addData("Intake Power",      String.format("%.2f", intakeMotor.getPower()));
            telemetry.addData("Right Trigger",     String.format("%.2f", gamepad1.right_trigger));
            telemetry.addData("Left Trigger",      String.format("%.2f", gamepad1.left_trigger));
            telemetry.update();
        }
    }
}