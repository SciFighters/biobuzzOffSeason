package org.firstinspires.ftc.teamcode.Utilities.pid;

/**
 * Immutable configuration for a PID profile.
 *
 * <p>A PIDConfig contains all tuning parameters for a specific
 * operating mode (FAST, HOLD, SLOW, etc.).</p>
 *
 * <p>Example:</p>
 *
 * <pre>
 * PIDConfig holdConfig = PIDConfig.builder()
 *         .name("HOLD")
 *         .description("Maintains arm position")
 *         .kp(0.08)
 *         .ki(0.01)
 *         .tolerance(5)
 *         .build();
 * </pre>
 *
 * <p>This class is immutable after construction.</p>
 */
public final class PIDConfig {

    /*------------------------------------------------------------------*/
    /* PROFILE INFORMATION                                               */
    /*------------------------------------------------------------------*/

    /** Unique profile name. */
    public final String name;

    /** Optional description for telemetry/debugging. */
    public final String description;

    /*------------------------------------------------------------------*/
    /* PID GAINS                                                         */
    /*------------------------------------------------------------------*/

    /** Proportional gain. */
    public final double kp;

    /** Integral gain. */
    public final double ki;

    /** Derivative gain. */
    public final double kd;

    /*------------------------------------------------------------------*/
    /* OUTPUT LIMITS                                                     */
    /*------------------------------------------------------------------*/

    /** Minimum allowed output. */
    public final double minOutput;

    /** Maximum allowed output. */
    public final double maxOutput;

    /*------------------------------------------------------------------*/
    /* INTEGRAL CONTROL                                                  */
    /*------------------------------------------------------------------*/

    /**
     * Integral accumulation only occurs when:
     *
     * abs(error) <= integralZone
     *
     * Set to POSITIVE_INFINITY to disable.
     */
    public final double integralZone;

    /**
     * Maximum magnitude of accumulated integral.
     *
     * Prevents integral windup.
     */
    public final double maxIntegral;

    /**
     * Integral leakage factor.
     *
     * 1.0 = no leakage
     * 0.999 = slow leakage
     * 0.0 = instantly clears integral
     */
    public final double integralLeakRate;

    /*------------------------------------------------------------------*/
    /* DEADBANDS                                                         */
    /*------------------------------------------------------------------*/

    /**
     * Errors smaller than this are treated as zero.
     */
    public final double errorDeadband;

    /**
     * Outputs smaller than this are treated as zero.
     */
    public final double outputDeadband;

    /*------------------------------------------------------------------*/
    /* DERIVATIVE FILTERING                                              */
    /*------------------------------------------------------------------*/

    /**
     * Exponential moving average coefficient.
     *
     * 1.0 = no filtering
     * 0.1 = heavy filtering
     */
    public final double derivativeAlpha;

    /*------------------------------------------------------------------*/
    /* TARGET DETECTION                                                  */
    /*------------------------------------------------------------------*/

    /**
     * Error threshold used by atTarget().
     */
    public final double tolerance;

    /*------------------------------------------------------------------*/
    /* CONTINUOUS INPUT                                                  */
    /*------------------------------------------------------------------*/

    /**
     * Enables angle wrapping.
     *
     * Example:
     *
     * minimumInput = -180
     * maximumInput = 180
     *
     * Error between 179 and -179 becomes 2 degrees
     * instead of 358 degrees.
     */
    public final boolean continuousInput;

    /** Minimum continuous value. */
    public final double minimumInput;

    /** Maximum continuous value. */
    public final double maximumInput;

    /*------------------------------------------------------------------*/
    /* OUTPUT SLEW RATE LIMITING                                         */
    /*------------------------------------------------------------------*/

    /**
     * Maximum output change per second.
     *
     * Set to POSITIVE_INFINITY to disable.
     */
    public final double maxOutputChangePerSecond;

    /*------------------------------------------------------------------*/
    /* CONSTRUCTION                                                      */
    /*------------------------------------------------------------------*/

    private PIDConfig(Builder builder) {

        this.name = builder.name;
        this.description = builder.description;

        this.kp = builder.kp;
        this.ki = builder.ki;
        this.kd = builder.kd;

        this.minOutput = builder.minOutput;
        this.maxOutput = builder.maxOutput;

        this.integralZone = builder.integralZone;
        this.maxIntegral = builder.maxIntegral;
        this.integralLeakRate = builder.integralLeakRate;

        this.errorDeadband = builder.errorDeadband;
        this.outputDeadband = builder.outputDeadband;

        this.derivativeAlpha = builder.derivativeAlpha;

        this.tolerance = builder.tolerance;

        this.continuousInput = builder.continuousInput;
        this.minimumInput = builder.minimumInput;
        this.maximumInput = builder.maximumInput;

        this.maxOutputChangePerSecond =
                builder.maxOutputChangePerSecond;
    }

    /**
     * Creates a new builder.
     */
    public static Builder builder() {
        return new Builder();
    }

    /*------------------------------------------------------------------*/
    /* BUILDER                                                           */
    /*------------------------------------------------------------------*/

    public static final class Builder {

        /* Profile Information */

        private String name;
        private String description = "";

        /* PID Gains */

        private double kp = 0.0;
        private double ki = 0.0;
        private double kd = 0.0;

        /* Output Limits */

        private double minOutput = -Double.MAX_VALUE;
        private double maxOutput = Double.MAX_VALUE;

        /* Integral */

        private double integralZone =
                Double.POSITIVE_INFINITY;

        private double maxIntegral =
                Double.POSITIVE_INFINITY;

        private double integralLeakRate = 1.0;

        /* Deadbands */

        private double errorDeadband = 0.0;
        private double outputDeadband = 0.0;

        /* Derivative Filtering */

        private double derivativeAlpha = 1.0;

        /* Target Detection */

        private double tolerance = 0.0;

        /* Continuous Input */

        private boolean continuousInput = false;

        private double minimumInput = 0.0;
        private double maximumInput = 0.0;

        /* Slew Rate Limiting */

        private double maxOutputChangePerSecond =
                Double.POSITIVE_INFINITY;

        private Builder() {}

        /*--------------------------------------------------------------*/
        /* PROFILE INFO                                                 */
        /*--------------------------------------------------------------*/

        public Builder name(String name) {
            this.name = name;
            return this;
        }

        public Builder description(String description) {
            this.description = description;
            return this;
        }

        /*--------------------------------------------------------------*/
        /* PID                                                          */
        /*--------------------------------------------------------------*/

        public Builder kp(double kp) {
            this.kp = kp;
            return this;
        }

        public Builder ki(double ki) {
            this.ki = ki;
            return this;
        }

        public Builder kd(double kd) {
            this.kd = kd;
            return this;
        }

        /*--------------------------------------------------------------*/
        /* OUTPUT LIMITS                                                */
        /*--------------------------------------------------------------*/

        public Builder minOutput(double value) {
            this.minOutput = value;
            return this;
        }

        public Builder maxOutput(double value) {
            this.maxOutput = value;
            return this;
        }

        /*--------------------------------------------------------------*/
        /* INTEGRAL                                                     */
        /*--------------------------------------------------------------*/

        public Builder integralZone(double value) {
            this.integralZone = value;
            return this;
        }

        public Builder maxIntegral(double value) {
            this.maxIntegral = value;
            return this;
        }

        public Builder integralLeakRate(double value) {
            this.integralLeakRate = value;
            return this;
        }

        /*--------------------------------------------------------------*/
        /* DEADBANDS                                                    */
        /*--------------------------------------------------------------*/

        public Builder errorDeadband(double value) {
            this.errorDeadband = value;
            return this;
        }

        public Builder outputDeadband(double value) {
            this.outputDeadband = value;
            return this;
        }

        /*--------------------------------------------------------------*/
        /* DERIVATIVE FILTERING                                         */
        /*--------------------------------------------------------------*/

        public Builder derivativeAlpha(double value) {
            this.derivativeAlpha = value;
            return this;
        }

        /*--------------------------------------------------------------*/
        /* TARGET DETECTION                                             */
        /*--------------------------------------------------------------*/

        public Builder tolerance(double value) {
            this.tolerance = value;
            return this;
        }

        /*--------------------------------------------------------------*/
        /* CONTINUOUS INPUT                                             */
        /*--------------------------------------------------------------*/

        public Builder continuousInput(
                double minimumInput,
                double maximumInput) {

            this.continuousInput = true;
            this.minimumInput = minimumInput;
            this.maximumInput = maximumInput;

            return this;
        }

        /*--------------------------------------------------------------*/
        /* SLEW RATE LIMITING                                           */
        /*--------------------------------------------------------------*/

        public Builder maxOutputChangePerSecond(
                double value) {

            this.maxOutputChangePerSecond = value;
            return this;
        }

        /*--------------------------------------------------------------*/
        /* BUILD                                                        */
        /*--------------------------------------------------------------*/

        public PIDConfig build() {

            if (name == null || name.trim().isEmpty()) {
                throw new IllegalArgumentException(
                        "Profile name cannot be null or blank");
            }

            if (maxOutput < minOutput) {
                throw new IllegalArgumentException(
                        "maxOutput must be >= minOutput");
            }

            if (derivativeAlpha < 0.0
                    || derivativeAlpha > 1.0) {

                throw new IllegalArgumentException(
                        "derivativeAlpha must be between 0 and 1");
            }

            if (integralLeakRate < 0.0
                    || integralLeakRate > 1.0) {

                throw new IllegalArgumentException(
                        "integralLeakRate must be between 0 and 1");
            }

            if (continuousInput
                    && maximumInput <= minimumInput) {

                throw new IllegalArgumentException(
                        "maximumInput must be greater than minimumInput");
            }

            if (maxIntegral < 0.0) {
                throw new IllegalArgumentException(
                        "maxIntegral must be non-negative");
            }

            if (tolerance < 0.0) {
                throw new IllegalArgumentException(
                        "tolerance must be non-negative");
            }

            return new PIDConfig(this);
        }
    }
}