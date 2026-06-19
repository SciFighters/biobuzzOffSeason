package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.jetbrains.annotations.NotNull;
import org.firstinspires.ftc.teamcode.Utilities.pid.PIDController;
import org.firstinspires.ftc.teamcode.Utilities.pid.PIDConfig;

public class MotorOut {
    @NotNull
    public final DcMotorEx motor;
    public int planetaryRPM;
    private static final int DEFAULT_RPM = 6000;
    public final int ticksPerRevolutionGear;
    public int ticksPerRevolutionBare = 28;

    public MotorOut(@NotNull DcMotorEx motor,
                    int ticksPerRevolutionGear,
                    int planetaryRPM
    ){
        this.motor = motor;
        this.ticksPerRevolutionGear = ticksPerRevolutionGear;
        this.planetaryRPM = planetaryRPM;
    }

    public void setPower(double power) {
        if (motor != null) {
            motor.setPower(power);
        }
    }

    public int getPosTicks() {
        return motor != null ? motor.getCurrentPosition() : 0;
    }

    public double getPosAngle() {
        if (motor == null || ticksPerRevolutionBare == 0) {
            return 0.0;
        }
        return (double) getPosTicks() * 360.0 / ticksPerRevolutionBare;
    }

    public double getVelocity() {
        return motor != null ? motor.getVelocity() : 0.0;
    }

    public double getAngularVelocity() {
        if (motor == null || ticksPerRevolutionBare == 0) {
            return 0.0;
        }
        return getVelocity() * 360.0 / ticksPerRevolutionBare;
    }

    public double getMotorRPM() {
        if (ticksPerRevolutionBare == 0 || motor == null) {
            return 0.0;
        }
        return (motor.getVelocity() * 60.0) / ticksPerRevolutionBare;
    }

    public double getMotorOutRPM() {
        if (ticksPerRevolutionGear == 0 || motor == null) {
            return 0.0;
        }
        return (motor.getVelocity() * 60.0) / ticksPerRevolutionGear;
    }

    public double getRatio() {
        return planetaryRPM / (double) DEFAULT_RPM;
    }

    public void setRatio(double ratio) {
        if (ratio < 0) {
            ratio = 0;
        }
        planetaryRPM = (int) Math.round(ratio * DEFAULT_RPM);
    }

    public double getPower() {
        return motor.getPower();
    }

    private static final int POS_TOLERANCE = 5;

    private void goToTicks(int targetTicks) {
        if (motor == null) return;
        while (Math.abs(motor.getCurrentPosition() - targetTicks) > POS_TOLERANCE) {
            int current = motor.getCurrentPosition();
            double power = (current < targetTicks) ? 1.0 : -1.0;
            motor.setPower(power);
            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            }
        }
        motor.setPower(0);
    }

    public void goToPos(double targetAngleDeg) {
        if (motor == null || ticksPerRevolutionBare == 0) return;
        int targetTicks = (int) Math.round(targetAngleDeg * ticksPerRevolutionBare / 360.0);
        goToTicks(targetTicks);
    }

    /**
      * Rotates the motor to the specified angle in degrees using a PID controller.
      *
      * @param targetAngleDeg target angle in degrees
      * @param cfg PID configuration to use
      */
    public void goToPosPID(double targetAngleDeg, PIDConfig cfg) {
        if (motor == null || cfg == null || ticksPerRevolutionBare == 0) {
            return;
        }
        int targetTicks = (int) Math.round(targetAngleDeg * ticksPerRevolutionBare / 360.0);

        PIDController pid = new PIDController();
        pid.addProfile(cfg);
        pid.setProfile(cfg.name, true);

        while (!pid.atTarget()) {
            int current = motor.getCurrentPosition();
            double power = pid.calculate(current, targetTicks);
            motor.setPower(power);
            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            }
        }
        motor.setPower(0);
    }

    public void resetEncoder() {
        if (motor != null) {
            motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            try {
                Thread.sleep(250);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
            motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }
    }
}