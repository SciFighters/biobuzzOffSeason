package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.jetbrains.annotations.NotNull;

public class MotorOut {
    @NotNull public final DcMotorEx motor;
    private static final int DEFAULT_RPM = 6000;
    public int planetaryRPM;          // Desired output RPM
    private int ticksPerRevolutionBare = 28; // motor's native encoder ticks per rev
    private int ticksPerRevolutionGear; // effective ticks per rev at output after gearing

// No internal PID state; use external PIDController if needed.

     /**
      * ONLY the desired output RPM is required.
      * Gear ratio and tick counts are derived automatically.
      */
     public MotorOut(@NotNull DcMotorEx motor, int outputRpm) {
         this.motor = motor;
         planetaryRPM = outputRpm;
         double gearRatio = (double) DEFAULT_RPM / outputRpm;
         ticksPerRevolutionGear = (int) Math.round(ticksPerRevolutionBare * gearRatio);
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
         if (motor == null || ticksPerRevolutionGear == 0) {
             return 0.0;
         }
         return (double) getPosTicks() * 360.0 / ticksPerRevolutionGear;
     }

     public double getVelocity() {
         return motor != null ? motor.getVelocity() : 0.0;
     }

     public double getAngularVelocity() {
         if (motor == null || ticksPerRevolutionGear == 0) {
             return 0.0;
         }
         return getVelocity() * 360.0 / ticksPerRevolutionGear;
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
         if (ratio < 0) ratio = 0;
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
         if (motor == null || ticksPerRevolutionGear == 0) return;
         int targetTicks = (int) Math.round(targetAngleDeg * ticksPerRevolutionGear / 360.0);
         goToTicks(targetTicks);
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