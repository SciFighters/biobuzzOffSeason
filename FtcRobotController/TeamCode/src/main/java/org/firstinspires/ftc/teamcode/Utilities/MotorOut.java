package org.firstinspires.ftc.teamcode.Utilities.pid;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.jetbrains.annotations.NotNull;



public class MotorOut {
    @NotNull public final DcMotorEx motor;
    private static final int DEFAULT_RPM = 6000;
    private static final int ticksPerRevolutionBare = 28; // motor's native encoder ticks per rev
    private final double planetaryRatio;          // Desired output RPM
    private double systemGearRatio;
    private double totalGearRatio;
    private int ticksPerRevolutionOut; // effective ticks per rev at output after gearing

// No internal PID state; use external PIDController if needed.

     /**
      * ONLY the desired output RPM is required.
      * Gear ratio and tick counts are derived automatically.
      */
     public MotorOut(@NotNull DcMotorEx motor, String planeteryRPM, double systemGearRatio) {
         this.motor = motor;
         this.systemGearRatio = systemGearRatio;
         this.planetaryRatio = GobildaPlanetery.toRatio(planeteryRPM);
         totalGearRatio = systemGearRatio * this.planetaryRatio;
         ticksPerRevolutionOut = (int) Math.round(ticksPerRevolutionBare * totalGearRatio);
     }

     public void setPower(double power) {
         motor.setPower(power);
     }

     public int getPosTicks() {
         return motor.getCurrentPosition();
     }


     public double getPosAngle() {
         return (double) ticksToDegrees(getPosTicks());
     }

     public double getVelocity() {
         return motor.getVelocity();
     }

     public double getAngularVelocity() {
         return ticksToDegrees(getVelocity());
     }

     private double ticksToDegrees(int ticks) {
         return ticksToDegrees((double) ticks);
     }

    private double ticksToDegrees(double ticks) {
        return (ticks / ticksPerRevolutionOut) * 360.0;
    }

     public double getMotorRPM() {
         return (motor.getVelocity() * 60.0) / ticksPerRevolutionBare;
     }

     public double getMotorOutRPM() {
         return (motor.getVelocity() * 60.0) / ticksPerRevolutionOut;
     }

     public double getRatio() {
         return planetaryRatio / (double) DEFAULT_RPM;
     }

     public double getPower() {
         return motor.getPower();
     }

     public void resetEncoder() {
         motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
         try {
             Thread.sleep(250);
         } catch (InterruptedException e) {
             Thread.currentThread().interrupt();
         }
         motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
     }
}