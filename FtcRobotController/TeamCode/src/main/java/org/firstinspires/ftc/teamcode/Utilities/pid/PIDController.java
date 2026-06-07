package org.firstinspires.ftc.teamcode.Utilities.pid;

import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;

/**
 * A modular, profile-based PID controller designed for FTC robotics.
 *
 * <p>This controller supports multiple tuning profiles (FAST, HOLD, etc.),
 * feedforward injection, telemetry statistics, and robust control features
 * like integral windup protection, derivative filtering, and slew rate limiting.</p>
 *
 * <p>All heavy logic is broken into small private methods for readability
 * and debugging clarity.</p>
 */
public class PIDController {

    /*------------------------------------------------------------------*/
    /* PROFILES                                                        */
    /*------------------------------------------------------------------*/

    private final Map<String, PIDConfig> profiles = new HashMap<>();
    private String currentProfileName;
    private PIDConfig config;

    /*------------------------------------------------------------------*/
    /* FEEDFORWARD                                                     */
    /*------------------------------------------------------------------*/

    /**
     * Optional external feedforward.
     * Can represent gravity, friction, etc.
     */
    private DoubleSupplier feedforwardSupplier;

    /*------------------------------------------------------------------*/
    /* STATISTICS                                                      */
    /*------------------------------------------------------------------*/

    private final PIDStatistics statistics = new PIDStatistics();

    /*------------------------------------------------------------------*/
    /* RUNTIME STATE                                                   */
    /*------------------------------------------------------------------*/

    private double error;
    private double target;

    private double integralSum;
    private double lastError;

    private double derivative;
    private double filteredDerivative;

    private double lastOutput;

    private long lastTimeNanos;
    private double dt;

    /*==================================================================*/
    /* CONSTRUCTORS                                                    */
    /*==================================================================*/

    public PIDController() {
        this.lastTimeNanos = System.nanoTime();
    }

    public PIDController(PIDConfig config) {
        this();
        addProfile(config);
        setProfile(config.name);
    }

    /*==================================================================*/
    /* PROFILE MANAGEMENT                                              */
    /*==================================================================*/

    /**
     * Adds a PID profile.
     */
    public void addProfile(PIDConfig config) {
        profiles.put(config.name, config);
    }

    /**
     * Removes a profile.
     */
    public void removeProfile(String name) {
        profiles.remove(name);
    }

    /**
     * Sets active profile (preserves state).
     */
    public void setProfile(String name) {
        setProfile(name, false);
    }

    /**
     * Sets active profile.
     *
     * @param resetState if true, clears integral/derivative history
     */
    public void setProfile(String name, boolean resetState) {

        PIDConfig newConfig = profiles.get(name);

        if (newConfig == null) {
            throw new IllegalArgumentException(
                    "Profile not found: " + name);
        }

        this.currentProfileName = name;
        this.config = newConfig;

        if (resetState) {
            reset();
        }
    }

    public String getCurrentProfileName() {
        return currentProfileName;
    }

    public String getCurrentProfileDescription() {
        return config != null ? config.description : "";
    }

    public PIDConfig getCurrentConfig() {
        return config;
    }

    /*==================================================================*/
    /* FEEDFORWARD                                                     */
    /*==================================================================*/

    public void setFeedforwardSupplier(DoubleSupplier supplier) {
        this.feedforwardSupplier = supplier;
    }

    private double computeFeedforward() {
        return (feedforwardSupplier == null)
                ? 0.0
                : feedforwardSupplier.getAsDouble();
    }

    /*==================================================================*/
    /* MAIN CALCULATION API                                            */
    /*==================================================================*/

    public double calculate(double measurement, double target) {

        double error = computeError(measurement, target);
        return calculateError(error);
    }

    public double calculateError(double error) {

        updateTiming();

        this.error = applyErrorDeadband(error);
        this.target = Double.NaN; // unknown unless provided externally

        updateIntegral();
        updateDerivative();

        double output = computeOutput();

        output = applyOutputDeadband(output);
        output = applySlewRate(output);
        output = applyOutputLimits(output);

        lastOutput = output;

        statistics.update(this.error);

        return output;
    }

    /*==================================================================*/
    /* CORE COMPUTATION                                                */
    /*==================================================================*/

    private double computeError(double measurement, double target) {

        this.target = target;

        double error = target - measurement;

        if (config.continuousInput) {
            error = wrapError(error);
        }

        return error;
    }

    private double computeOutput() {
        return computeP()
                + computeI()
                + computeD()
                + computeFeedforward();
    }

    private double computeP() {
        return config.kp * error;
    }

    private double computeI() {
        return integralSum * config.ki;
    }

    private double computeD() {
        return config.kd * filteredDerivative;
    }

    /*==================================================================*/
    /* TIMING                                                          */
    /*==================================================================*/

    private void updateTiming() {

        long now = System.nanoTime();
        dt = (now - lastTimeNanos) / 1e9;

        lastTimeNanos = now;

        if (dt <= 0) {
            dt = 1e-3; // safety fallback
        }
    }

    /*==================================================================*/
    /* INTEGRAL                                                        */
    /*==================================================================*/

    private void updateIntegral() {

        if (Math.abs(error) > config.integralZone) {
            return;
        }

        integralSum += error * dt;

        integralSum *= config.integralLeakRate;

        integralSum = clamp(
                integralSum,
                -config.maxIntegral,
                config.maxIntegral
        );
    }

    public void resetIntegral() {
        integralSum = 0;
    }

    /*==================================================================*/
    /* DERIVATIVE                                                     */
    /*==================================================================*/

    private void updateDerivative() {

        double rawDerivative =
                (error - lastError) / dt;

        filteredDerivative =
                config.derivativeAlpha * rawDerivative
                        + (1 - config.derivativeAlpha)
                        * filteredDerivative;

        lastError = error;
    }

    public void resetDerivative() {
        lastError = 0;
        filteredDerivative = 0;
    }

    /*==================================================================*/
    /* LIMITERS                                                       */
    /*==================================================================*/

    private double applyErrorDeadband(double error) {
        return Math.abs(error) <= config.errorDeadband
                ? 0.0
                : error;
    }

    private double applyOutputDeadband(double output) {
        return Math.abs(output) <= config.outputDeadband
                ? 0.0
                : output;
    }

    private double applyOutputLimits(double output) {
        return clamp(output, config.minOutput, config.maxOutput);
    }

    private double applySlewRate(double output) {

        double maxChange =
                config.maxOutputChangePerSecond * dt;

        double delta = output - lastOutput;

        if (Math.abs(delta) > maxChange) {
            output = lastOutput
                    + Math.signum(delta)
                    * maxChange;
        }

        return output;
    }

    private double wrapError(double error) {

        double range =
                config.maximumInput - config.minimumInput;

        while (error > range / 2) {
            error -= range;
        }

        while (error < -range / 2) {
            error += range;
        }

        return error;
    }

    private double clamp(
            double value,
            double min,
            double max) {

        return Math.max(min,
                Math.min(max, value));
    }

    /*==================================================================*/
    /* RESET                                                           */
    /*==================================================================*/

    public void reset() {
        resetIntegral();
        resetDerivative();
        resetTiming();
    }

    public void resetTiming() {
        lastTimeNanos = System.nanoTime();
        dt = 0;
    }

    /*==================================================================*/
    /* STATUS                                                          */
    /*==================================================================*/

    public boolean atTarget() {
        return Math.abs(error) <= config.tolerance;
    }

    /*==================================================================*/
    /* STATISTICS                                                     */
    /*==================================================================*/

    public double getCalculationsPerSecond() {
        return statistics.getCalculationsPerSecond();
    }

    /*==================================================================*/
    /* TELEMETRY GETTERS                                             */
    /*==================================================================*/

    public double getError() {
        return error;
    }

    public double getTarget() {
        return target;
    }

    public double getIntegralSum() {
        return integralSum;
    }

    public double getDerivative() {
        return filteredDerivative;
    }

    public double getLastOutput() {
        return lastOutput;
    }

    public String getProfileName() {
        return currentProfileName;
    }

    /*==================================================================*/
    /* CONTRIBUTIONS                                                  */
    /*==================================================================*/

    public double getPContribution() {
        return computeP();
    }

    public double getIContribution() {
        return computeI();
    }

    public double getDContribution() {
        return computeD();
    }

    public double getFeedforwardContribution() {
        return computeFeedforward();
    }
}