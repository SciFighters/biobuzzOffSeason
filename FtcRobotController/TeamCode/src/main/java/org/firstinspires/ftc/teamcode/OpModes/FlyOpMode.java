package org.firstinspires.ftc.teamcode.OpModes;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.RunCommand;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subSystems.FlySub;

// Check turn direction with wheels lifted before driving.
@TeleOp(name = "Fly :)", group = "Fuze")
public class FlyOpMode extends CommandOpMode {

    // Positive gyro rate and turn mean CCW.
    public static final double gyroScale = 1.0;
    public static final double gyroBiasRadPerSec = 0.0;
    public static final double turnSign = 1.0;
    public static final double maxPower = FlySub.maxPower;
    public static final double turnKp = 0.8;
    public static final double turnKd = 0.08;
    public static final double homeToleranceRad = Math.toRadians(5);

    private final FlySub flySub = new FlySub();
    private final double[] networkAction = new double[3];
    private Mecanum drive;
    private PinpointLocalizer localizer;
    private GoBildaPinpointDriver pinpoint;

    private long lastTimeNs;
    private double homeYaw;

    private boolean needsReset = true;
    private boolean lastX;
    private boolean lastY;
    private boolean lastReward;
    private boolean lastToggle;
    private boolean networkMode;

    @Override
    public void initialize() {
        drive = new Mecanum(hardwareMap, Constants.driveConfig);
        drive.stop();

        localizer = new PinpointLocalizer(hardwareMap, Constants.localizerConfig);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, Constants.localizerConfig.name.get());

        telemetry.setMsTransmissionInterval(100);

        schedule(new RunCommand(() -> {
            if (lastTimeNs == 0) {
                lastTimeNs = System.nanoTime();
                lastX = gamepad1.x;
                lastY = gamepad1.y;
                lastReward = gamepad1.left_bumper;
                lastToggle = gamepad1.right_bumper;
                needsReset = true;
            }

            updateFly();
        }));
    }

    @Override
    public void initialize_loop() {
        localizer.update();

        telemetry.addData("Pinpoint", pinpoint.getDeviceStatus());
        telemetry.addLine("Keep still for calibration. After START, press X to set home.");
        telemetry.update();
    }

    private void updateFly() {
        try {
            localizer.update();

            long currentTimeNs = System.nanoTime();
            double dt = (currentTimeNs - lastTimeNs) / 1e9;
            lastTimeNs = currentTimeNs;

            double yaw = pinpoint.getHeading(AngleUnit.RADIANS);
            double fieldX = pinpoint.getPosX(DistanceUnit.INCH);
            double fieldY = pinpoint.getPosY(DistanceUnit.INCH);
            double turnRate = (pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS)
                    - gyroBiasRadPerSec) * gyroScale;
            boolean ready = pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY
                    && Double.isFinite(yaw) && Double.isFinite(turnRate)
                    && Double.isFinite(fieldX) && Double.isFinite(fieldY);
            boolean reset = gamepad1.x && !lastX;
            boolean reward = gamepad1.left_bumper && !lastReward;
            boolean toggle = gamepad1.right_bumper && !lastToggle;

            if (gamepad1.y && !lastY) {
                flySub.setRecurrent(!flySub.isRecurrent());
            }

            lastX = gamepad1.x;
            lastY = gamepad1.y;
            lastReward = gamepad1.left_bumper;
            lastToggle = gamepad1.right_bumper;

            if (!ready || dt <= 0 || dt > FlySub.maxStepS) {
                needsReset = true;
            }

            if (reset && ready) {
                flySub.reset();
                homeYaw = yaw;
                needsReset = false;
            } else if (!needsReset && !flySub.update(turnRate, dt)) {
                needsReset = true;
            }
            flySub.setPosition(fieldX, fieldY);

            double heading = flySub.getHeading();
            if (networkMode && !Double.isFinite(heading)) {
                needsReset = true;
            }
            boolean returning = gamepad1.a && !gamepad1.b && !reset && !needsReset
                    && Double.isFinite(heading);

            if (toggle && !needsReset && !reset && !gamepad1.b && Double.isFinite(heading)) {
                networkMode = !networkMode;
            }
            if (needsReset || reset || gamepad1.b || gamepad1.a || !Double.isFinite(heading)) {
                networkMode = false;
            }

            if (needsReset || reset || gamepad1.b
                    || ((gamepad1.a || gamepad1.right_bumper) && !Double.isFinite(heading))) {
                flySub.forgetAction();
                drive.stop();
            } else {
                if (reward) {
                    flySub.reward();
                }

                if (networkMode) {
                    flySub.getNetworkAction(networkAction);
                    double forward = networkAction[0];
                    double strafe = networkAction[1];
                    double turn = networkAction[2];
                    double power = flySub.getNetworkPower();
                    flySub.recordAction(forward, strafe, turn);
                    drive.drive(new DrivePowers(power * forward, power * strafe,
                            turnSign * power * turn), true);
                } else if (returning) {
                    double error = FlySub.wrap(-heading);
                    double turn = Math.abs(error) <= homeToleranceRad ? 0
                            : Math.max(-maxPower, Math.min(maxPower, turnKp * error - turnKd * turnRate));
                    flySub.recordAction(0, 0, turn / maxPower);
                    drive.drive(new DrivePowers(0, 0, turnSign * turn), true);
                } else {
                    double forward = deadband(-gamepad1.left_stick_y);
                    double strafe = deadband(-gamepad1.left_stick_x);
                    double turn = deadband(-gamepad1.right_stick_x);
                    double scale = 1 / Math.max(1, Math.abs(forward) + Math.abs(strafe) + Math.abs(turn));
                    flySub.recordAction(forward * scale, strafe * scale, turn * scale);
                    drive.drive(new DrivePowers(forward * scale * maxPower, strafe * scale * maxPower,
                            turnSign * turn * scale * maxPower), true);
                }
            }

            telemetry.addData("State", needsReset ? "STOPPED: press X when Pinpoint is ready"
                    : gamepad1.b || reset ? "Stopped"
                    : returning ? "Returning home (release A to cancel)"
                    : !Double.isFinite(heading) ? "Memory faded: X to reset; network disabled"
                    : networkMode ? "Network driving (RB switches to manual)" : "Manual");
            telemetry.addData("Pinpoint", pinpoint.getDeviceStatus());
            telemetry.addData("Recurrence", flySub.isRecurrent());
            telemetry.addData("Neural heading (deg)", Math.toDegrees(heading));
            telemetry.addData("Gyro heading from home (deg)", Math.toDegrees(FlySub.wrap(yaw - homeYaw)));
            telemetry.addData("Neural error vs gyro (deg)", Math.toDegrees(FlySub.wrap(heading - (yaw - homeYaw))));
            telemetry.addData("Bump strength", flySub.getStrength());
            telemetry.addData("Loop (ms)", dt * 1000);
            telemetry.addData("Field X/Y (in)", "%.1f / %.1f", fieldX, fieldY);
            telemetry.addData("Rewards (session)", flySub.getRewards());
            telemetry.addData("Local confidence", "%.0f%%", flySub.getConfidence() * 100);
            telemetry.addData("Network power limit", "%.0f%%", flySub.getNetworkPower() * 100);
            telemetry.addLine("Sticks: drive | Hold A: home | B: stop | X: zero + clear learning | Y: recurrence");
            telemetry.addLine("RB: manual/network toggle | LB: reward last 2 seconds");
            telemetry.update();
        } catch (RuntimeException failure) {
            needsReset = true;
            drive.stop();
            throw failure;
        }
    }

    private static double deadband(double value) {
        return Math.abs(value) < 0.05 ? 0 : value;
    }

    @Override
    public void end() {
        if (drive != null) {
            drive.stop();
        }
    }
}
