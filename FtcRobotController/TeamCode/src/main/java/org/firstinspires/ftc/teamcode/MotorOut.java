package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.jetbrains.annotations.NotNull;

/**
 * Utility wrapper for a single motor that handles gear‑ratio calculations.
 * <p>
 * All public methods are null‑safe and return double values where fractional
 * precision matters (RPM, angular velocity).  Integer truncation is applied
 * only when storing the planetary gear RPM.
 */
public class MotorOut {
    @NotNull
    public final DcMotorEx motor;
    public int planetaryRPM;
    private static final int DEFAULT_RPM = 6000;
    public final int ticksPerRevolutionGear;
    public final int ticksPerRevolutionBare;

    public MotorOut(@NotNull DcMotorEx motor,
                    int ticksPerRevolutionGear,
                    int ticksPerRevolutionBare) {
        this.motor = motor;
        this.ticksPerRevolutionGear = ticksPerRevolutionGear;
        this.ticksPerRevolutionBare = ticksPerRevolutionBare;
        this.planetaryRPM = 0; // initialized to “no gear” state
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
}