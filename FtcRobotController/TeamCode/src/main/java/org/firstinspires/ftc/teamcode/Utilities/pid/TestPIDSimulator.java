package org.firstinspires.ftc.teamcode.Utilities.pid;

/**
 * Simple console simulator for PIDController.
 * Does not depend on FTC hardware libraries; can be run with a plain
 * Java runtime (e.g. {@code java -cp ... TestPIDSimulator}).
 *
 * Simulates a motor moving toward a target position using a PID loop.
 * At each iteration prints:
 *   step, measurement, error, P, I, D, Feedforward, total output
 */
public class TestPIDSimulator {
    public static void main(String[] args) {
        // ----- PID configuration -----
        PIDConfig config = PIDConfig.builder()
                .name("SIM")
                .description("Console PID test")
                .kp(0.1)   // proportional gain
                .ki(0.02)  // integral gain
                .kd(0.05)  // derivative gain
                .integralZone(100) // accumulate only when error small enough
                .maxIntegral(50)
                .integralLeakRate(1.0)
                .errorDeadband(0.0)
                .outputDeadband(0.0)
                .derivativeAlpha(1.0) // no filtering
                .tolerance(0.5)
                .minOutput(-1.0)
                .maxOutput( 1.0)
                .build();

        PIDControllerTimingExtension pid = new PIDControllerTimingExtension(config);

        // ----- Simulation parameters -----
        double targetPosition = 100.0; // desired encoder count
        double measurement = 0.0;       // start at zero
        int steps = 200;                // max iterations (will also stop after 10 s)

        System.out.println("Starting PID simulation (updates every 0.5 s, max 10 s)...");
        long startTime = System.nanoTime();
        for (int i = 0; i < steps; i++) {
            double error = targetPosition - measurement;
            // calculate uses real elapsed time (updated each loop)
            double output = pid.calculate(measurement, targetPosition);

            double p = pid.getPContribution();
            double ii = pid.getIContribution(); // avoid shadow i
            double d = pid.getDContribution();
            double ff = pid.getFeedforwardContribution();

            // single‑line realtime display, overwrite previous line
            System.out.print(String.format("\rStep:%3d | Meas:%6.2f | Err:%6.2f | P:%6.2f | I:%6.2f | D:%6.2f | FF:%6.2f | Out:%6.2f",
                    i, measurement, error, p, ii, d, ff, output));
            System.out.flush();

            // simulate arm: output * gain * dt (dt ≈ 0.5 s from sleep)
            double plantGain = 10.0; // units per second at full power
            // clamp output to motor limits (-1..1)
            double clamped = Math.max(-1.0, Math.min(1.0, output));
            measurement += clamped * plantGain * 0.5; // use 0.5 s step

            // pause half‑second for visible update
            try { Thread.sleep(500); } catch (InterruptedException ignored) {}
            // time limit check (10 s)
            if ((System.nanoTime() - startTime) / 1e9 >= 10.0) {
                System.out.println("\nTime limit reached.");
                break;
            }

            if (Math.abs(error) <= config.tolerance) {
                System.out.println("\nTarget reached in " + (i + 1) + " steps.");
                break;
            }
        }
    }
}

class PIDControllerTimingExtension extends PIDController {
    public PIDControllerTimingExtension(PIDConfig cfg) { super(cfg); }
    public void updateTimingManual(double dt) {
        try {
            java.lang.reflect.Field dtField = PIDController.class.getDeclaredField("dt");
            dtField.setAccessible(true);
            dtField.setDouble(this, dt);
            // Also update lastTimeNanos to keep internal consistency.
            java.lang.reflect.Field lastField = PIDController.class.getDeclaredField("lastTimeNanos");
            lastField.setAccessible(true);
            lastField.setLong(this, System.nanoTime());
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }
}
